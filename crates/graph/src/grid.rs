//! Uniform grid over *edges*, and nearest-edge snapping.
//!
//! Over edges, not nodes: a GPS fix in the middle of a 400 m V3 segment is
//! nowhere near either endpoint.
//!
//! Only one direction of each two-way road is indexed - the canonical one, the
//! lower of an edge and its twin. Both directions share a polyline, so they are
//! exactly equidistant from any point, and indexing both would double the grid
//! to return an arbitrary one of a tied pair. Coordinate routing reaches the
//! other direction through `Graph::twin`.

use crate::{haversine, Graph, NO_TWIN};

/// Target cell size. 200 m is a few edges across at this density.
pub const CELL_M: f64 = 200.0;

/// Local flat-earth scale. Fixed for the whole graph rather than recomputed per
/// point, so the grid and the brute-force oracle cannot disagree by a rounding
/// step: the correctness gate compares distances at 1e-6.
#[derive(Clone, Copy, Debug)]
pub struct Metric {
    pub m_per_deg_lon: f64,
    pub m_per_deg_lat: f64,
}

impl Metric {
    pub fn for_graph(g: &Graph) -> Metric {
        let mid_lat = if g.lat.is_empty() {
            0.0
        } else {
            let (mut lo, mut hi) = (f64::MAX, f64::MIN);
            for v in &g.lat {
                lo = lo.min(*v);
                hi = hi.max(*v);
            }
            (lo + hi) / 2.0
        };
        Metric {
            m_per_deg_lon: haversine((0.0, mid_lat), (1.0, mid_lat)),
            m_per_deg_lat: haversine((0.0, mid_lat - 0.5), (0.0, mid_lat + 0.5)),
        }
    }
    /// Degrees to metres, relative to an origin. Good to well under a metre
    /// over a city-sized extent.
    fn xy(&self, p: (f64, f64), origin: (f64, f64)) -> (f64, f64) {
        (
            (p.0 - origin.0) * self.m_per_deg_lon,
            (p.1 - origin.1) * self.m_per_deg_lat,
        )
    }
}

/// Where a coordinate landed on the road network.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Snap {
    /// Canonical edge id: the lower of the edge and its twin.
    pub edge: u32,
    /// The perpendicular projection, `(lon, lat)`.
    pub point: (f64, f64),
    /// How far the query point was from the road.
    pub distance_m: f64,
    /// Distance along the edge from its source to the projection.
    pub offset_m: f64,
    /// Travel time from the edge source to the projection, and from the
    /// projection to the edge head. These sum to the edge weight.
    pub cost_to_source_ms: u32,
    pub cost_to_head_ms: u32,
}

/// Project `at` onto one edge polyline. Returns `(distance_m, offset_m, point)`.
///
/// Walks the geometry through `Graph::geometry`, so it respects the
/// invert-on-read flag: on a reversed edge the offset is measured from *that*
/// direction's source, which is what seeding a search needs.
fn project(g: &Graph, edge: usize, at: (f64, f64), m: &Metric) -> (f64, f64, (f64, f64)) {
    let q = (0.0, 0.0); // the query point is the origin of the local frame
    let mut best = (f64::MAX, 0.0, at);
    let mut travelled = 0.0f64;
    let mut prev: Option<(f64, f64)> = None;

    for p in g.geometry(edge) {
        let cur = (p[0] as f64, p[1] as f64);
        let Some(a) = prev else {
            prev = Some(cur);
            continue;
        };
        let (ax, ay) = m.xy(a, at);
        let (bx, by) = m.xy(cur, at);
        let (dx, dy) = (bx - ax, by - ay);
        let len2 = dx * dx + dy * dy;
        let t = if len2 <= f64::EPSILON {
            0.0
        } else {
            (((q.0 - ax) * dx + (q.1 - ay) * dy) / len2).clamp(0.0, 1.0)
        };
        let (px, py) = (ax + t * dx, ay + t * dy);
        let d = (px * px + py * py).sqrt();
        let seg_m = len2.sqrt();
        if d < best.0 {
            best = (
                d,
                travelled + t * seg_m,
                (a.0 + t * (cur.0 - a.0), a.1 + t * (cur.1 - a.1)),
            );
        }
        travelled += seg_m;
        prev = Some(cur);
    }
    best
}

fn snap_from(g: &Graph, edge: usize, at: (f64, f64), m: &Metric) -> Snap {
    let (distance_m, offset_m, point) = project(g, edge, at, m);
    let len = g.length[edge] as f64;
    let frac = if len > 0.0 {
        (offset_m / len).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let w = g.weight[edge] as f64;
    let to_source = (w * frac).round() as u32;
    Snap {
        edge: edge as u32,
        point,
        distance_m,
        offset_m,
        cost_to_source_ms: to_source,
        // Subtract so the two always sum to the edge weight exactly; rounding
        // both independently would drift by a millisecond and the coordinate
        // routing gate compares costs exactly.
        cost_to_head_ms: g.weight[edge] - to_source,
    }
}

/// Every edge that is its own canonical id - one per undirected road.
fn canonical_edges(g: &Graph) -> impl Iterator<Item = usize> + '_ {
    (0..g.n_edges()).filter(|e| {
        let t = g.twin[*e];
        t == NO_TWIN || (*e as u32) < t
    })
}

/// Nearest edge by linear scan. The oracle the grid is checked against, and the
/// thing the grid has to beat.
/// Takes the metric rather than deriving it, so the oracle and the grid cannot
/// disagree by a rounding step - and so a 1000-point sweep is not recomputing
/// the graph bounds a thousand times.
pub fn nearest_brute(g: &Graph, at: (f64, f64), max_radius_m: f64, m: &Metric) -> Option<Snap> {
    let mut best: Option<Snap> = None;
    for e in canonical_edges(g) {
        let s = snap_from(g, e, at, m);
        if s.distance_m > max_radius_m {
            continue;
        }
        // Ties broken by lowest edge id so the grid can match exactly.
        if best.is_none_or(|b| s.distance_m < b.distance_m) {
            best = Some(s);
        }
    }
    best
}

pub struct Grid {
    metric: Metric,
    origin: (f64, f64),
    /// Cell size in degrees, derived from `CELL_M` at this latitude.
    cell_lon: f64,
    cell_lat: f64,
    cols: u32,
    rows: u32,
    /// Length `cols * rows + 1`.
    cell_offsets: Vec<u32>,
    cell_edges: Vec<u32>,
}

impl Grid {
    pub fn build(g: &Graph) -> Grid {
        let metric = Metric::for_graph(g);
        let cell_lon = CELL_M / metric.m_per_deg_lon;
        let cell_lat = CELL_M / metric.m_per_deg_lat;

        // Bounds over geometry, not nodes: a polyline can bulge past both its
        // endpoints.
        let (mut min_lon, mut min_lat) = (f64::MAX, f64::MAX);
        let (mut max_lon, mut max_lat) = (f64::MIN, f64::MIN);
        for p in &g.geom {
            min_lon = min_lon.min(p[0] as f64);
            max_lon = max_lon.max(p[0] as f64);
            min_lat = min_lat.min(p[1] as f64);
            max_lat = max_lat.max(p[1] as f64);
        }
        if g.geom.is_empty() {
            min_lon = 0.0;
            min_lat = 0.0;
            max_lon = 0.0;
            max_lat = 0.0;
        }
        let origin = (min_lon, min_lat);
        let cols = (((max_lon - min_lon) / cell_lon).ceil() as u32 + 1).max(1);
        let rows = (((max_lat - min_lat) / cell_lat).ceil() as u32 + 1).max(1);

        // One entry per cell each edge touches. Per *segment* bounding box
        // rather than per edge: a long curving edge whose endpoints sit in
        // opposite corners would otherwise miss every cell along its middle.
        //
        // ponytail: a segment bbox is a superset of the cells the segment
        // actually crosses, so a diagonal segment inserts a few cells it only
        // grazes. Never a miss, so queries stay correct; upgrade to a proper
        // line rasteriser only if the entry count becomes a problem.
        let mut entries: Vec<(u32, u32)> = Vec::new();
        for e in canonical_edges(g) {
            let mut prev: Option<[f32; 2]> = None;
            for p in g.geometry(e) {
                let Some(a) = prev else {
                    prev = Some(p);
                    continue;
                };
                let (lo_lon, hi_lon) = minmax(a[0] as f64, p[0] as f64);
                let (lo_lat, hi_lat) = minmax(a[1] as f64, p[1] as f64);
                let c0 = ((lo_lon - origin.0) / cell_lon).floor().max(0.0) as u32;
                let c1 = (((hi_lon - origin.0) / cell_lon).floor().max(0.0) as u32).min(cols - 1);
                let r0 = ((lo_lat - origin.1) / cell_lat).floor().max(0.0) as u32;
                let r1 = (((hi_lat - origin.1) / cell_lat).floor().max(0.0) as u32).min(rows - 1);
                for r in r0..=r1 {
                    for c in c0..=c1 {
                        entries.push((r * cols + c, e as u32));
                    }
                }
                prev = Some(p);
            }
        }
        entries.sort_unstable();
        entries.dedup();

        let n_cells = (cols as usize) * (rows as usize);
        let mut cell_offsets = vec![0u32; n_cells + 1];
        for (cell, _) in &entries {
            cell_offsets[*cell as usize + 1] += 1;
        }
        for i in 1..=n_cells {
            cell_offsets[i] += cell_offsets[i - 1];
        }
        Grid {
            metric,
            origin,
            cell_lon,
            cell_lat,
            cols,
            rows,
            cell_offsets,
            cell_edges: entries.into_iter().map(|(_, e)| e).collect(),
        }
    }

    pub fn dimensions(&self) -> (u32, u32) {
        (self.cols, self.rows)
    }
    pub fn n_entries(&self) -> usize {
        self.cell_edges.len()
    }
    pub fn metric(&self) -> Metric {
        self.metric
    }

    fn cell_of(&self, at: (f64, f64)) -> (i64, i64) {
        (
            ((at.0 - self.origin.0) / self.cell_lon).floor() as i64,
            ((at.1 - self.origin.1) / self.cell_lat).floor() as i64,
        )
    }

    /// Nearest edge to `at`, or `None` if nothing is within `max_radius_m`.
    ///
    /// Rings expand outward until the closest point the *next* ring could
    /// possibly hold is already further than the best found. Stopping at the
    /// first ring that yields a hit is the classic bug: the true nearest edge
    /// can sit in a diagonal cell of the next ring out.
    pub fn nearest(&self, g: &Graph, at: (f64, f64), max_radius_m: f64) -> Option<Snap> {
        let (cx, cy) = self.cell_of(at);
        // Conservative: use the smaller cell dimension for the ring bound.
        let step_m = CELL_M.min(self.cell_lon * self.metric.m_per_deg_lon);
        let mut best: Option<Snap> = None;
        // Enough rings to cover the whole grid even from outside it, capped by
        // the radius the caller asked for.
        let max_ring = self.cols as i64
            + self.rows as i64
            + (cx.abs() + cy.abs())
            + (max_radius_m / step_m).ceil() as i64
            + 2;

        for r in 0..=max_ring {
            // A cell r rings out has its nearest edge at least (r-1) cells away.
            let ring_floor_m = ((r - 1).max(0)) as f64 * step_m;
            if ring_floor_m > max_radius_m {
                break;
            }
            if let Some(b) = best {
                if ring_floor_m > b.distance_m {
                    break;
                }
            }
            for c in self.ring_cells(cx, cy, r) {
                for slot in self.cell_offsets[c] as usize..self.cell_offsets[c + 1] as usize {
                    let e = self.cell_edges[slot] as usize;
                    let s = snap_from(g, e, at, &self.metric);
                    if s.distance_m > max_radius_m {
                        continue;
                    }
                    let better = match best {
                        None => true,
                        Some(b) => {
                            s.distance_m < b.distance_m
                                || (s.distance_m == b.distance_m && s.edge < b.edge)
                        }
                    };
                    if better {
                        best = Some(s);
                    }
                }
            }
        }
        best
    }

    /// Flat indices of the in-bounds cells at Chebyshev distance `r`.
    fn ring_cells(&self, cx: i64, cy: i64, r: i64) -> Vec<usize> {
        let mut out = Vec::new();
        let push = |c: i64, y: i64, out: &mut Vec<usize>| {
            if c >= 0 && y >= 0 && c < self.cols as i64 && y < self.rows as i64 {
                out.push((y as usize) * self.cols as usize + c as usize);
            }
        };
        if r == 0 {
            push(cx, cy, &mut out);
            return out;
        }
        for c in (cx - r)..=(cx + r) {
            push(c, cy - r, &mut out);
            push(c, cy + r, &mut out);
        }
        for y in (cy - r + 1)..=(cy + r - 1) {
            push(cx - r, y, &mut out);
            push(cx + r, y, &mut out);
        }
        out
    }
}

fn minmax(a: f64, b: f64) -> (f64, f64) {
    if a <= b {
        (a, b)
    } else {
        (b, a)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Two parallel east-west roads 200 m apart, each split into two edges.
    fn two_roads() -> Graph {
        let coords = vec![
            (76.700, 30.700),
            (76.705, 30.700),
            (76.710, 30.700),
            (76.700, 30.7018),
            (76.705, 30.7018),
            (76.710, 30.7018),
        ];
        let edges = vec![
            (0, 1, 1000),
            (1, 0, 1000),
            (1, 2, 1000),
            (2, 1, 1000),
            (3, 4, 1000),
            (4, 3, 1000),
            (4, 5, 1000),
            (5, 4, 1000),
        ];
        Graph::from_edges(&coords, &edges)
    }

    #[test]
    fn grid_matches_brute_force_on_a_sweep() {
        let g = two_roads();
        let grid = Grid::build(&g);
        let mut checked = 0;
        for i in 0..40 {
            for j in 0..40 {
                let at = (76.6980 + i as f64 * 0.0004, 30.6990 + j as f64 * 0.0001_5);
                let a = grid.nearest(&g, at, 5000.0);
                let b = nearest_brute(&g, at, 5000.0, &grid.metric());
                match (a, b) {
                    (Some(a), Some(b)) => {
                        assert_eq!(a.edge, b.edge, "edge differs at {at:?}");
                        assert!(
                            (a.distance_m - b.distance_m).abs() < 1e-6,
                            "distance differs at {at:?}"
                        );
                    }
                    (None, None) => {}
                    _ => panic!("one found a road and the other did not at {at:?}"),
                }
                checked += 1;
            }
        }
        assert_eq!(checked, 1600);
    }

    #[test]
    fn only_canonical_edges_are_indexed() {
        let g = two_roads();
        let grid = Grid::build(&g);
        for e in grid.cell_edges.iter() {
            assert_eq!(g.pair_of(*e as usize), *e, "a reverse twin got indexed");
        }
    }

    #[test]
    fn far_away_returns_nothing_rather_than_a_road_40km_off() {
        let g = two_roads();
        let grid = Grid::build(&g);
        let far = (77.5, 31.5);
        assert_eq!(grid.nearest(&g, far, 100.0), None);
        assert_eq!(nearest_brute(&g, far, 100.0, &grid.metric()), None);
        // With a wide enough radius it does find one, and the same one.
        let a = grid.nearest(&g, far, 500_000.0).unwrap();
        let b = nearest_brute(&g, far, 500_000.0, &grid.metric()).unwrap();
        assert_eq!(a.edge, b.edge);
    }

    #[test]
    fn snapping_onto_a_node_gives_a_zero_offset_end() {
        let g = two_roads();
        let grid = Grid::build(&g);
        let s = grid.nearest(&g, (76.700, 30.700), 100.0).unwrap();
        // Not zero: geometry is stored as f32, whose ulp near 76.7 is about
        // 7.6e-6 degrees, so a point exactly on a node is still up to ~0.9 m
        // from the stored polyline. That quantisation is the floor on snap
        // precision and is far below GPS accuracy.
        assert!(s.distance_m < 1.0, "{}", s.distance_m);
        // The projection lands on an endpoint, so it sits at one end of the edge.
        let len = g.length[s.edge as usize] as f64;
        assert!(
            s.offset_m < 1.0 || (len - s.offset_m) < 1.0,
            "snap {s:?} is not at an end of a {len} m edge"
        );
        assert_eq!(
            s.cost_to_source_ms + s.cost_to_head_ms,
            g.weight[s.edge as usize]
        );
    }

    #[test]
    fn split_costs_always_sum_to_the_edge_weight() {
        let g = two_roads();
        let grid = Grid::build(&g);
        for i in 0..50 {
            let at = (76.700 + i as f64 * 0.0002, 30.7004);
            let s = grid.nearest(&g, at, 1000.0).unwrap();
            assert_eq!(
                s.cost_to_source_ms + s.cost_to_head_ms,
                g.weight[s.edge as usize]
            );
        }
    }
}
