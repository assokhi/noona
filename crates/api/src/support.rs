//! Search-context pool, metrics, typed errors, and polyline encoding.

use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::Arc;
use std::time::Duration;

use axum::http::StatusCode;
use axum::response::{IntoResponse, Response};
use axum::Json;
use parking_lot::Mutex;
use routing::Search;
use serde_json::json;
use tokio::sync::{OwnedSemaphorePermit, Semaphore};

// ---------------------------------------------------------------------------
// search context pool
// ---------------------------------------------------------------------------

/// A bounded pool of reusable search contexts.
///
/// Phase 2 made `Search` reuse its scratch arrays and reset only the entries a
/// query touched, because memsetting two 40k-entry arrays per query dominated
/// p50. That optimisation is exactly what stops the context being shareable
/// across concurrent requests. Allocating one per request would silently give
/// the optimisation back and the API p50 would stop matching the bench p50, so
/// requests wait for a context instead.
///
/// Each context is around 800 KB on this graph, so a pool of 16 is ~13 MB and
/// caps concurrent searches at a sensible number anyway.
pub struct Pool {
    slots: Mutex<Vec<Search>>,
    permits: Arc<Semaphore>,
    pub size: usize,
    pub context_bytes: usize,
}

impl Pool {
    pub fn new(size: usize, n_nodes: usize) -> Arc<Pool> {
        let slots: Vec<Search> = (0..size).map(|_| Search::new(n_nodes)).collect();
        let context_bytes = slots.first().map(|s| s.bytes()).unwrap_or(0);
        Arc::new(Pool {
            slots: Mutex::new(slots),
            permits: Arc::new(Semaphore::new(size)),
            size,
            context_bytes,
        })
    }

    /// Waits for a free context. The returned lease puts it back on drop.
    pub async fn acquire(self: &Arc<Self>) -> (Lease, Duration) {
        let start = std::time::Instant::now();
        let permit = self
            .permits
            .clone()
            .acquire_owned()
            .await
            .expect("pool semaphore is never closed");
        let waited = start.elapsed();
        let search = self
            .slots
            .lock()
            .pop()
            .expect("a permit guarantees a free context");
        (
            Lease {
                pool: self.clone(),
                search: Some(search),
                _permit: permit,
            },
            waited,
        )
    }
}

pub struct Lease {
    pool: Arc<Pool>,
    search: Option<Search>,
    _permit: OwnedSemaphorePermit,
}

impl Lease {
    /// Runs the search on this thread.
    ///
    /// A search here is on the order of a millisecond of pure CPU. Handing that
    /// to `spawn_blocking` costs a thread wake-up and two scheduler hops, which
    /// measured at ~0.5 ms - a third of the work itself. The pool already caps
    /// concurrent searches, so at most `size` worker threads are ever busy and
    /// the runtime cannot be starved by this path.
    pub fn with<T>(&mut self, f: impl FnOnce(&mut Search) -> T) -> T {
        f(self.search.as_mut().expect("lease still holds its context"))
    }

    /// Hands the context to a blocking task and takes it back afterwards. Kept
    /// for work that is long enough to be worth the handoff.
    #[allow(dead_code)]
    pub async fn run<T, F>(mut self, f: F) -> T
    where
        F: FnOnce(&mut Search) -> T + Send + 'static,
        T: Send + 'static,
    {
        let mut search = self.search.take().expect("lease used once");
        let (out, search) = tokio::task::spawn_blocking(move || {
            let out = f(&mut search);
            (out, search)
        })
        .await
        .expect("search task panicked");
        self.search = Some(search);
        out
    }
}

impl Drop for Lease {
    fn drop(&mut self) {
        if let Some(s) = self.search.take() {
            self.pool.slots.lock().push(s);
        }
    }
}

// ---------------------------------------------------------------------------
// errors
// ---------------------------------------------------------------------------

/// Stable machine-readable codes, not stringly typed messages: a client has to
/// be able to branch on these without matching prose.
#[derive(Debug, Clone)]
pub enum ApiError {
    MalformedCoordinate {
        detail: String,
    },
    PointOutsideBbox {
        which: &'static str,
        lon: f64,
        lat: f64,
    },
    NoRoadWithinRadius {
        which: &'static str,
        radius_m: f64,
    },
    Unreachable,
    UnknownAlgorithm {
        got: String,
    },
}

impl ApiError {
    pub fn code(&self) -> &'static str {
        match self {
            ApiError::MalformedCoordinate { .. } => "MALFORMED_COORDINATE",
            ApiError::PointOutsideBbox { .. } => "POINT_OUTSIDE_BBOX",
            ApiError::NoRoadWithinRadius { .. } => "NO_ROAD_WITHIN_RADIUS",
            ApiError::Unreachable => "UNREACHABLE",
            ApiError::UnknownAlgorithm { .. } => "UNKNOWN_ALGORITHM",
        }
    }
    pub fn status(&self) -> StatusCode {
        match self {
            ApiError::MalformedCoordinate { .. }
            | ApiError::PointOutsideBbox { .. }
            | ApiError::UnknownAlgorithm { .. } => StatusCode::BAD_REQUEST,
            ApiError::NoRoadWithinRadius { .. } | ApiError::Unreachable => StatusCode::NOT_FOUND,
        }
    }
}

impl IntoResponse for ApiError {
    fn into_response(self) -> Response {
        let mut body = json!({ "error": { "code": self.code() } });
        let e = body["error"].as_object_mut().expect("just built");
        match &self {
            ApiError::MalformedCoordinate { detail } => {
                e.insert("message".into(), json!(detail));
            }
            ApiError::PointOutsideBbox { which, lon, lat } => {
                e.insert(
                    "message".into(),
                    json!(format!("{which} point is outside the served area")),
                );
                e.insert("which".into(), json!(which));
                e.insert("lon".into(), json!(lon));
                e.insert("lat".into(), json!(lat));
            }
            ApiError::NoRoadWithinRadius { which, radius_m } => {
                e.insert(
                    "message".into(),
                    json!(format!(
                        "no road within {radius_m:.0} m of the {which} point"
                    )),
                );
                e.insert("which".into(), json!(which));
                e.insert("radius_m".into(), json!(radius_m));
            }
            ApiError::Unreachable => {
                e.insert(
                    "message".into(),
                    json!("no legal route between those points"),
                );
            }
            ApiError::UnknownAlgorithm { got } => {
                e.insert(
                    "message".into(),
                    json!(format!(
                        "unknown algorithm {got:?}, expected dijkstra, astar or bidir"
                    )),
                );
            }
        }
        (self.status(), Json(body)).into_response()
    }
}

/// `lon,lat` in that order, matching GeoJSON and the internal convention.
pub fn parse_lonlat(raw: &str) -> Result<(f64, f64), ApiError> {
    let (a, b) = raw
        .split_once(',')
        .ok_or_else(|| ApiError::MalformedCoordinate {
            detail: format!("expected lon,lat, got {raw:?}"),
        })?;
    let lon: f64 = a
        .trim()
        .parse()
        .map_err(|_| ApiError::MalformedCoordinate {
            detail: format!("longitude {:?} is not a number", a.trim()),
        })?;
    let lat: f64 = b
        .trim()
        .parse()
        .map_err(|_| ApiError::MalformedCoordinate {
            detail: format!("latitude {:?} is not a number", b.trim()),
        })?;
    check_range(lon, lat)?;
    Ok((lon, lat))
}

/// The order trap: Chandigarh is 76.7, 30.7 and both numbers are plausible as
/// either. A swap will not throw anywhere else, it will just put the route in
/// the Arabian Sea, so the range check is the boundary that catches it.
pub fn check_range(lon: f64, lat: f64) -> Result<(), ApiError> {
    if !(-180.0..=180.0).contains(&lon) || !(-90.0..=90.0).contains(&lat) {
        return Err(ApiError::MalformedCoordinate {
            detail: format!("lon {lon} / lat {lat} is out of range - is the order swapped?"),
        });
    }
    Ok(())
}

// ---------------------------------------------------------------------------
// polyline
// ---------------------------------------------------------------------------

/// Google encoded polyline, precision 5.
pub fn encode_polyline(points: &[[f32; 2]]) -> String {
    let mut out = String::with_capacity(points.len() * 6);
    let (mut prev_lat, mut prev_lon) = (0i32, 0i32);
    for p in points {
        let lat = (p[1] as f64 * 1e5).round() as i32;
        let lon = (p[0] as f64 * 1e5).round() as i32;
        encode_signed(lat - prev_lat, &mut out);
        encode_signed(lon - prev_lon, &mut out);
        prev_lat = lat;
        prev_lon = lon;
    }
    out
}

fn encode_signed(v: i32, out: &mut String) {
    let mut u = (v << 1) ^ (v >> 31);
    while u >= 0x20 {
        out.push((((0x20 | (u & 0x1f)) + 63) as u8) as char);
        u >>= 5;
    }
    out.push(((u + 63) as u8) as char);
}

// ---------------------------------------------------------------------------
// metrics
// ---------------------------------------------------------------------------

/// Upper bounds in seconds. Chosen around the numbers this service actually
/// produces: a search is single-digit milliseconds.
const LATENCY_BUCKETS: [f64; 9] = [0.001, 0.0025, 0.005, 0.01, 0.025, 0.05, 0.1, 0.25, 1.0];
const SETTLED_BUCKETS: [f64; 7] = [100.0, 1000.0, 5000.0, 10000.0, 20000.0, 40000.0, 100000.0];

#[derive(Default)]
struct Histogram {
    buckets: Vec<AtomicU64>,
    count: AtomicU64,
    sum: AtomicU64,
}

impl Histogram {
    fn new(n: usize) -> Histogram {
        Histogram {
            buckets: (0..=n).map(|_| AtomicU64::new(0)).collect(),
            count: AtomicU64::new(0),
            sum: AtomicU64::new(0),
        }
    }
    fn observe(&self, v: f64, bounds: &[f64]) {
        let i = bounds.iter().position(|b| v <= *b).unwrap_or(bounds.len());
        self.buckets[i].fetch_add(1, Ordering::Relaxed);
        self.count.fetch_add(1, Ordering::Relaxed);
        // Micro-units so the sum stays an integer without a float atomic.
        self.sum.fetch_add((v * 1e6) as u64, Ordering::Relaxed);
    }
    fn render(&self, name: &str, bounds: &[f64], out: &mut String) {
        let mut cumulative = 0u64;
        for (i, b) in bounds.iter().enumerate() {
            cumulative += self.buckets[i].load(Ordering::Relaxed);
            out.push_str(&format!("{name}_bucket{{le=\"{b}\"}} {cumulative}\n"));
        }
        cumulative += self.buckets[bounds.len()].load(Ordering::Relaxed);
        out.push_str(&format!("{name}_bucket{{le=\"+Inf\"}} {cumulative}\n"));
        out.push_str(&format!(
            "{name}_sum {}\n",
            self.sum.load(Ordering::Relaxed) as f64 / 1e6
        ));
        out.push_str(&format!(
            "{name}_count {}\n",
            self.count.load(Ordering::Relaxed)
        ));
    }
}

pub struct Metrics {
    requests: Mutex<std::collections::BTreeMap<(&'static str, u16), u64>>,
    latency: Histogram,
    settled: Histogram,
    pool_wait: Histogram,
}

impl Default for Metrics {
    fn default() -> Self {
        Metrics {
            requests: Mutex::new(Default::default()),
            latency: Histogram::new(LATENCY_BUCKETS.len()),
            settled: Histogram::new(SETTLED_BUCKETS.len()),
            pool_wait: Histogram::new(LATENCY_BUCKETS.len()),
        }
    }
}

impl Metrics {
    pub fn request(&self, endpoint: &'static str, status: u16, seconds: f64) {
        *self.requests.lock().entry((endpoint, status)).or_insert(0) += 1;
        self.latency.observe(seconds, &LATENCY_BUCKETS);
    }
    pub fn settled(&self, n: u32) {
        self.settled.observe(n as f64, &SETTLED_BUCKETS);
    }
    pub fn pool_wait(&self, seconds: f64) {
        self.pool_wait.observe(seconds, &LATENCY_BUCKETS);
    }

    pub fn render(&self) -> String {
        let mut out = String::new();
        out.push_str("# TYPE chd_requests_total counter\n");
        for ((endpoint, status), n) in self.requests.lock().iter() {
            out.push_str(&format!(
                "chd_requests_total{{endpoint=\"{endpoint}\",status=\"{status}\"}} {n}\n"
            ));
        }
        out.push_str("# TYPE chd_request_duration_seconds histogram\n");
        self.latency
            .render("chd_request_duration_seconds", &LATENCY_BUCKETS, &mut out);
        out.push_str("# TYPE chd_nodes_settled histogram\n");
        self.settled
            .render("chd_nodes_settled", &SETTLED_BUCKETS, &mut out);
        out.push_str("# TYPE chd_pool_wait_seconds histogram\n");
        self.pool_wait
            .render("chd_pool_wait_seconds", &LATENCY_BUCKETS, &mut out);
        out
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn polyline_matches_the_reference_example() {
        // The example from Google's own encoded polyline documentation.
        let pts = [
            [-120.2f32, 38.5f32],
            [-120.95f32, 40.7f32],
            [-126.453f32, 43.252f32],
        ];
        assert_eq!(encode_polyline(&pts), "_p~iF~ps|U_ulLnnqC_mqNvxq`@");
    }

    #[test]
    fn lonlat_parsing_rejects_the_swap() {
        assert_eq!(parse_lonlat("76.78,30.74").unwrap(), (76.78, 30.74));
        assert_eq!(parse_lonlat(" 76.78 , 30.74 ").unwrap(), (76.78, 30.74));
        // A latitude in the longitude slot is in range and cannot be caught
        // here, but anything genuinely out of range is.
        assert!(parse_lonlat("200,30").is_err());
        assert!(parse_lonlat("76.78,100").is_err());
        assert!(parse_lonlat("76.78").is_err());
        assert!(parse_lonlat("a,b").is_err());
    }

    #[test]
    fn histogram_buckets_are_cumulative() {
        let h = Histogram::new(LATENCY_BUCKETS.len());
        h.observe(0.0005, &LATENCY_BUCKETS);
        h.observe(0.02, &LATENCY_BUCKETS);
        h.observe(10.0, &LATENCY_BUCKETS);
        let mut s = String::new();
        h.render("x", &LATENCY_BUCKETS, &mut s);
        assert!(s.contains("x_bucket{le=\"0.001\"} 1"));
        assert!(s.contains("x_bucket{le=\"0.025\"} 2"));
        assert!(s.contains("x_bucket{le=\"+Inf\"} 3"));
        assert!(s.contains("x_count 3"));
    }
}
