//! HTTP API over the routing graph.
//!
//! Graph and grid load once at startup behind an `Arc` and are never reloaded.
//! Search contexts come from a bounded pool - see `support::Pool` for why they
//! cannot simply be allocated per request.
//!
//! Coordinates are `lon,lat` everywhere, matching GeoJSON and the internal
//! convention. Clients holding lat,lon convert once, at their own boundary.

mod support;

use std::collections::HashMap;
use std::path::PathBuf;
use std::sync::Arc;
use std::time::{Duration, Instant};

use axum::extract::{Query, State};
use axum::http::StatusCode;
use axum::response::IntoResponse;
use axum::routing::get;
use axum::{Json, Router};
use graph::grid::{Grid, Metric, Snap};
use graph::Graph;
use serde::Serialize;
use serde_json::{json, Value};
use tower_http::cors::CorsLayer;
use tower_http::limit::RequestBodyLimitLayer;
use tower_http::timeout::TimeoutLayer;
use tower_http::trace::TraceLayer;

use support::{check_range, encode_polyline, parse_lonlat, ApiError, Metrics, Pool};

/// How far from a coordinate we are willing to look for a road.
const SNAP_RADIUS_M: f64 = 1000.0;

struct AppState {
    graph: Arc<Graph>,
    grid: Arc<Grid>,
    /// Present only if landmarks.bin was found. `alg=alt` is refused rather
    /// than silently falling back when it is missing.
    landmarks: Option<Arc<routing::alt::Landmarks>>,
    /// The contracted graph, if ch.bin was found.
    ch: Option<Arc<routing::ch::Ch>>,
    metric: Metric,
    pool: Arc<Pool>,
    metrics: Metrics,
    /// Served area, from the graph geometry.
    bbox: (f64, f64, f64, f64),
}

type Shared = Arc<AppState>;

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    tracing_subscriber::fmt()
        .with_env_filter(
            tracing_subscriber::EnvFilter::try_from_default_env()
                .unwrap_or_else(|_| "info,tower_http=info".into()),
        )
        .init();

    let mut graph_path = PathBuf::from("data/build/graph.bin");
    let mut landmarks_path = PathBuf::from("data/build/landmarks.bin");
    let mut ch_path = PathBuf::from("data/build/ch.bin");
    let mut addr = "127.0.0.1:8080".to_string();
    let mut pool_size = std::thread::available_parallelism()
        .map_or(8, |n| n.get())
        .max(4);
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut it = args.iter();
    while let Some(flag) = it.next() {
        let v = it.next().ok_or_else(|| format!("{flag} needs a value"))?;
        match flag.as_str() {
            "--graph" => graph_path = PathBuf::from(v),
            "--landmarks" => landmarks_path = PathBuf::from(v),
            "--ch" => ch_path = PathBuf::from(v),
            "--addr" => addr = v.clone(),
            "--pool" => pool_size = v.parse()?,
            other => return Err(format!("unknown flag {other}").into()),
        }
    }

    let load = Instant::now();
    let (g, source_hash) = Graph::load(&graph_path)?;
    let grid = Grid::build(&g);
    let metric = grid.metric();
    let (mut min_lon, mut min_lat) = (f64::MAX, f64::MAX);
    let (mut max_lon, mut max_lat) = (f64::MIN, f64::MIN);
    for p in &g.geom {
        min_lon = min_lon.min(p[0] as f64);
        max_lon = max_lon.max(p[0] as f64);
        min_lat = min_lat.min(p[1] as f64);
        max_lat = max_lat.max(p[1] as f64);
    }
    // Optional: the server is useful without it, and a stale table is worse
    // than none, so a hash mismatch is refused rather than tolerated.
    let landmarks = match routing::alt::Landmarks::load(&landmarks_path) {
        Ok((lm, hash)) if hash == source_hash => {
            tracing::info!(count = lm.count(), "landmarks loaded");
            Some(Arc::new(lm))
        }
        Ok(_) => {
            tracing::warn!(
                path = %landmarks_path.display(),
                "landmarks were built for a different graph, ignoring them"
            );
            None
        }
        Err(e) => {
            tracing::info!("no landmarks ({e}); alg=alt will be refused");
            None
        }
    };

    let ch = match routing::ch::Ch::load(&ch_path) {
        Ok((ch, hash)) if hash == source_hash => {
            tracing::info!(shortcuts = ch.n_shortcuts(), "contraction hierarchy loaded");
            Some(Arc::new(ch))
        }
        Ok(_) => {
            tracing::warn!(path = %ch_path.display(), "CH was built for a different graph, ignoring it");
            None
        }
        Err(e) => {
            tracing::info!("no contraction hierarchy ({e}); alg=ch will be refused");
            None
        }
    };

    let pool = Pool::new(pool_size, g.n_nodes());
    tracing::info!(
        nodes = g.n_nodes(),
        edges = g.n_edges(),
        grid_entries = grid.n_entries(),
        pool = pool.size,
        context_kb = pool.context_bytes / 1024,
        source = format!("{source_hash:#018x}"),
        ms = load.elapsed().as_millis() as u64,
        "loaded"
    );

    let state: Shared = Arc::new(AppState {
        graph: Arc::new(g),
        grid: Arc::new(grid),
        landmarks,
        ch,
        metric,
        pool,
        metrics: Metrics::default(),
        bbox: (min_lon, min_lat, max_lon, max_lat),
    });

    let app = Router::new()
        .route("/v1/nearest", get(nearest))
        .route("/v1/route", get(route))
        .route("/healthz", get(healthz))
        .route("/metrics", get(metrics))
        .layer(TraceLayer::new_for_http())
        .layer(TimeoutLayer::with_status_code(
            StatusCode::GATEWAY_TIMEOUT,
            Duration::from_secs(10),
        ))
        .layer(RequestBodyLimitLayer::new(64 * 1024))
        // The map is served from a different origin in development.
        .layer(CorsLayer::permissive())
        .with_state(state);

    let listener = tokio::net::TcpListener::bind(&addr).await?;
    tracing::info!("listening on http://{addr}");
    axum::serve(listener, app)
        .with_graceful_shutdown(async {
            let _ = tokio::signal::ctrl_c().await;
        })
        .await?;
    Ok(())
}

async fn healthz(State(s): State<Shared>) -> impl IntoResponse {
    Json(json!({
        "status": "ok",
        "nodes": s.graph.n_nodes(),
        "edges": s.graph.n_edges(),
        "pool": s.pool.size,
        "landmarks": s.landmarks.as_ref().map(|l| l.count()),
        "ch_shortcuts": s.ch.as_ref().map(|c| c.n_shortcuts()),
    }))
}

async fn metrics(State(s): State<Shared>) -> impl IntoResponse {
    (
        [("content-type", "text/plain; version=0.0.4")],
        s.metrics.render(),
    )
}

/// Snap once, with the range and bbox checks that make a lon/lat swap loud.
fn snap_point(s: &AppState, which: &'static str, lon: f64, lat: f64) -> Result<Snap, ApiError> {
    check_range(lon, lat)?;
    let (min_lon, min_lat, max_lon, max_lat) = s.bbox;
    // A generous margin: the graph extends past the clip bbox along ways that
    // crossed it, and a point just outside is a legitimate query.
    let pad = 0.05;
    if lon < min_lon - pad || lon > max_lon + pad || lat < min_lat - pad || lat > max_lat + pad {
        return Err(ApiError::PointOutsideBbox { which, lon, lat });
    }
    s.grid
        .nearest(&s.graph, (lon, lat), SNAP_RADIUS_M)
        .ok_or(ApiError::NoRoadWithinRadius {
            which,
            radius_m: SNAP_RADIUS_M,
        })
}

#[derive(Serialize)]
struct NearestBody {
    edge_id: u32,
    snapped: [f64; 2],
    distance_m: f64,
    way_name: Option<String>,
    offset_m: f64,
}

async fn nearest(
    State(s): State<Shared>,
    Query(q): Query<HashMap<String, String>>,
) -> Result<Json<NearestBody>, ApiError> {
    let t0 = Instant::now();
    let out = nearest_inner(&s, &q);
    s.metrics.request(
        "/v1/nearest",
        out.as_ref().map_or_else(|e| e.status().as_u16(), |_| 200),
        t0.elapsed().as_secs_f64(),
    );
    out.map(Json)
}

fn nearest_inner(s: &AppState, q: &HashMap<String, String>) -> Result<NearestBody, ApiError> {
    let num = |k: &str| -> Result<f64, ApiError> {
        q.get(k)
            .ok_or_else(|| ApiError::MalformedCoordinate {
                detail: format!("missing {k}"),
            })?
            .parse()
            .map_err(|_| ApiError::MalformedCoordinate {
                detail: format!("{k} is not a number"),
            })
    };
    let snap = snap_point(s, "query", num("lon")?, num("lat")?)?;
    Ok(NearestBody {
        edge_id: snap.edge,
        snapped: [snap.point.0, snap.point.1],
        distance_m: snap.distance_m,
        way_name: s.graph.name(snap.edge as usize).map(str::to_string),
        offset_m: snap.offset_m,
    })
}

async fn route(
    State(s): State<Shared>,
    Query(q): Query<HashMap<String, String>>,
) -> Result<Json<Value>, ApiError> {
    let t0 = Instant::now();
    let out = route_inner(&s, q).await;
    s.metrics.request(
        "/v1/route",
        out.as_ref().map_or_else(|e| e.status().as_u16(), |_| 200),
        t0.elapsed().as_secs_f64(),
    );
    out.map(Json)
}

async fn route_inner(s: &AppState, q: HashMap<String, String>) -> Result<Value, ApiError> {
    let get = |k: &str| -> Result<&String, ApiError> {
        q.get(k).ok_or_else(|| ApiError::MalformedCoordinate {
            detail: format!("missing {k}"),
        })
    };
    let (flon, flat) = parse_lonlat(get("from")?)?;
    let (tlon, tlat) = parse_lonlat(get("to")?)?;
    // Kept permanently, not a dev flag: being able to A/B the same request
    // across algorithms against live traffic is how a regression gets caught.
    let alg_name = q.get("alg").map(String::as_str).unwrap_or("bidir");
    let alg: routing::coord::Alg = alg_name.parse().map_err(|_| ApiError::UnknownAlgorithm {
        got: alg_name.to_string(),
    })?;
    let as_polyline = q.get("geometry").map(String::as_str) == Some("polyline");

    let snap_start = Instant::now();
    let from = snap_point(s, "origin", flon, flat)?;
    let to = snap_point(s, "destination", tlon, tlat)?;
    let snap_ms = snap_start.elapsed().as_secs_f64() * 1000.0;

    let (mut lease, waited) = s.pool.acquire().await;
    s.metrics.pool_wait(waited.as_secs_f64());

    let search_start = Instant::now();
    // Refuse rather than silently answering with a different algorithm.
    let missing = match alg {
        routing::coord::Alg::Alt if s.landmarks.is_none() => Some("alt (no landmarks.bin)"),
        routing::coord::Alg::Ch if s.ch.is_none() => Some("ch (no ch.bin)"),
        _ => None,
    };
    if let Some(got) = missing {
        return Err(ApiError::UnknownAlgorithm { got: got.into() });
    }
    let prepared = routing::coord::Prepared {
        landmarks: s.landmarks.as_deref(),
        ch: s.ch.as_deref(),
    };
    let found = lease.with(|search| {
        routing::coord::route_with(search, &s.graph, &s.metric, prepared, from, to, alg)
    });
    let search_ms = search_start.elapsed().as_secs_f64() * 1000.0;

    let r = found.ok_or(ApiError::Unreachable)?;
    s.metrics.settled(r.stats.nodes_settled);

    let geometry = if as_polyline {
        json!(encode_polyline(&r.geometry))
    } else {
        json!({
            "type": "LineString",
            "coordinates": r.geometry.iter()
                .map(|p| json!([round6(p[0] as f64), round6(p[1] as f64)]))
                .collect::<Vec<_>>(),
        })
    };

    Ok(json!({
        "distance_m": round1(r.distance_m),
        "duration_s": round1(r.duration_s()),
        "geometry": geometry,
        "legs": [{
            "distance_m": round1(r.distance_m),
            "duration_s": round1(r.duration_s()),
        }],
        "debug": {
            "alg": alg.name(),
            "nodes_settled": r.stats.nodes_settled,
            "edges_relaxed": r.stats.edges_relaxed,
            "search_ms": round3(search_ms),
            "snap_ms": round3(snap_ms),
            "pool_wait_ms": round3(waited.as_secs_f64() * 1000.0),
            "same_edge": r.same_edge,
            "from_snap": {
                "edge_id": r.from.edge,
                "distance_m": round1(r.from.distance_m),
                "snapped": [round6(r.from.point.0), round6(r.from.point.1)],
                "way_name": s.graph.name(r.from.edge as usize),
            },
            "to_snap": {
                "edge_id": r.to.edge,
                "distance_m": round1(r.to.distance_m),
                "snapped": [round6(r.to.point.0), round6(r.to.point.1)],
                "way_name": s.graph.name(r.to.edge as usize),
            },
        },
    }))
}

fn round1(v: f64) -> f64 {
    (v * 10.0).round() / 10.0
}
fn round3(v: f64) -> f64 {
    (v * 1000.0).round() / 1000.0
}
fn round6(v: f64) -> f64 {
    (v * 1e6).round() / 1e6
}
