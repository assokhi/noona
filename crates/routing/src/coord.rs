//! Routing between coordinates rather than node ids.
//!
//! Snapping puts a point *on an edge*; the search runs on nodes. Bridging the
//! two is the whole job here, and it has three cases: the same edge, two edges
//! that share a node, and everything else. Nothing is ever inserted into the
//! graph - the endpoints become seed costs on an ordinary search.

use graph::grid::{sub_polyline, Metric, Snap};
use graph::{Graph, NO_TWIN};

use crate::{Search, SearchStats, Seed};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Alg {
    Dijkstra,
    Astar,
    Bidir,
    /// A* with landmark bounds. Needs the landmark tables loaded.
    Alt,
    /// Contraction hierarchies. Needs the contracted graph loaded.
    Ch,
}

impl Alg {
    /// Everything routable without extra preprocessing. ALT is excluded
    /// because it needs landmark tables that may not be built.
    pub const ALL: [Alg; 3] = [Alg::Dijkstra, Alg::Astar, Alg::Bidir];
    pub fn name(self) -> &'static str {
        match self {
            Alg::Dijkstra => "dijkstra",
            Alg::Astar => "astar",
            Alg::Bidir => "bidir",
            Alg::Alt => "alt",
            Alg::Ch => "ch",
        }
    }
}

impl std::str::FromStr for Alg {
    type Err = ();
    fn from_str(s: &str) -> Result<Alg, ()> {
        match s {
            "dijkstra" => Ok(Alg::Dijkstra),
            "astar" | "a*" => Ok(Alg::Astar),
            "bidir" | "bidirectional" => Ok(Alg::Bidir),
            "alt" => Ok(Alg::Alt),
            "ch" => Ok(Alg::Ch),
            _ => Err(()),
        }
    }
}

#[derive(Clone, Debug)]
pub struct CoordRoute {
    pub cost_ms: u32,
    pub distance_m: f64,
    pub geometry: Vec<[f32; 2]>,
    pub from: Snap,
    pub to: Snap,
    pub stats: SearchStats,
    /// True when both ends landed on one edge and the graph was never searched.
    pub same_edge: bool,
}

impl CoordRoute {
    pub fn duration_s(&self) -> f64 {
        self.cost_ms as f64 / 1000.0
    }
}

/// Nodes a search may *leave from* after starting at this snap point.
///
/// Direction matters: on a one-way edge only the head is reachable, because
/// there is no legal way to drive backwards from the snap point to the tail.
/// Seeding both ends of a two-way edge permits an immediate U-turn at the snap
/// point, which is accepted for now - Phase 8 turn restrictions is where that
/// gets revisited.
fn source_seeds(g: &Graph, s: &Snap) -> Vec<Seed> {
    let e = s.edge as usize;
    // The snap landed on a junction rather than mid-edge. Then the endpoint is
    // simply a node, and leaving it is not constrained by the edge it happened
    // to be found on - which for a one-way is the difference between routing
    // from the junction and being forced to drive the whole edge first.
    if at_head(g, s) {
        return vec![(g.head[e], 0)];
    }
    if at_source(s) {
        return vec![(g.edge_source(e), 0)];
    }
    let mut out = vec![(g.head[e], s.cost_to_head_ms)];
    if g.twin[e] != NO_TWIN {
        out.push((g.edge_source(e), s.cost_to_source_ms));
    }
    out
}

/// Nodes a search may *arrive at* and still reach this snap point legally.
fn target_seeds(g: &Graph, s: &Snap) -> Vec<Seed> {
    let e = s.edge as usize;
    if at_head(g, s) {
        return vec![(g.head[e], 0)];
    }
    if at_source(s) {
        return vec![(g.edge_source(e), 0)];
    }
    let mut out = vec![(g.edge_source(e), s.cost_to_source_ms)];
    if g.twin[e] != NO_TWIN {
        out.push((g.head[e], s.cost_to_head_ms));
    }
    out
}

/// The snap sits on the edge source. Exact after the endpoint clamping in
/// `graph::grid`, which pulls sub-metre offsets onto the end they belong to.
fn at_source(s: &Snap) -> bool {
    s.offset_m <= 0.0
}
fn at_head(g: &Graph, s: &Snap) -> bool {
    s.offset_m >= g.length[s.edge as usize] as f64
}

/// Both ends on one edge: the answer is a slice of that polyline and the graph
/// is never touched.
///
/// The twin case falls out for free. The grid only ever returns the canonical
/// edge of a two-way road, so two points on opposite carriageways of the same
/// road already carry the same edge id here.
fn same_edge_route(g: &Graph, from: &Snap, to: &Snap, m: &Metric) -> Option<CoordRoute> {
    if from.edge != to.edge {
        return None;
    }
    let e = from.edge as usize;
    let forward = to.offset_m >= from.offset_m;
    if !forward && g.twin[e] == NO_TWIN {
        // Destination is behind us on a one-way. The search has to go round.
        return None;
    }
    let (cost_ms, distance_m, mut geometry) = if forward {
        (
            to.cost_to_source_ms.saturating_sub(from.cost_to_source_ms),
            to.offset_m - from.offset_m,
            sub_polyline(g, e, from.offset_m, to.offset_m, m),
        )
    } else {
        (
            from.cost_to_source_ms.saturating_sub(to.cost_to_source_ms),
            from.offset_m - to.offset_m,
            sub_polyline(g, e, to.offset_m, from.offset_m, m),
        )
    };
    if !forward {
        geometry.reverse();
    }
    Some(CoordRoute {
        cost_ms,
        distance_m,
        geometry,
        from: *from,
        to: *to,
        stats: SearchStats::default(),
        same_edge: true,
    })
}

/// Route from one snapped point to another.
pub fn route(
    search: &mut Search,
    g: &Graph,
    m: &Metric,
    from: Snap,
    to: Snap,
    alg: Alg,
) -> Option<CoordRoute> {
    route_with(search, g, m, Prepared::default(), from, to, alg)
}

/// Preprocessed structures a query may need. Each algorithm that requires one
/// refuses to run without it rather than silently answering with another.
#[derive(Clone, Copy, Default)]
pub struct Prepared<'a> {
    pub landmarks: Option<&'a crate::alt::Landmarks>,
    pub ch: Option<&'a crate::ch::Ch>,
}

/// As `route`, with preprocessed structures available.
pub fn route_with(
    search: &mut Search,
    g: &Graph,
    m: &Metric,
    prepared: Prepared<'_>,
    from: Snap,
    to: Snap,
    alg: Alg,
) -> Option<CoordRoute> {
    if let Some(r) = same_edge_route(g, &from, &to, m) {
        return Some(r);
    }

    let sources = source_seeds(g, &from);
    let targets = target_seeds(g, &to);
    let r = match alg {
        Alg::Dijkstra => search.dijkstra_multi(g, &sources, &targets),
        Alg::Astar => search.astar_multi(g, &sources, &targets, to.point),
        Alg::Bidir => search.bidirectional_multi(g, &sources, &targets),
        Alg::Alt => search.alt_multi(
            g,
            prepared.landmarks.expect("Alg::Alt needs landmark tables"),
            &sources,
            &targets,
        ),
        // CH returns original edge ids after unpacking, so the rest of this
        // function treats it exactly like any other search result.
        Alg::Ch => search
            .ch_multi(
                prepared.ch.expect("Alg::Ch needs a contracted graph"),
                &sources,
                &targets,
            )
            .map(|(cost_ms, edges, stats)| {
                let distance_m = edges.iter().map(|e| g.length[*e as usize] as f64).sum();
                let from_node = edges
                    .first()
                    .map_or(sources[0].0, |e| g.edge_source(*e as usize));
                let to_node = edges
                    .last()
                    .map_or(targets[0].0, |e| g.head[*e as usize]);
                crate::Route {
                    cost_ms,
                    distance_m,
                    edges,
                    from_node,
                    to_node,
                    stats,
                }
            }),
    }?;

    // The adjacent-edge trap: when the two snapped edges share a node, going
    // straight through it is always available, so the answer can never be worse.
    // A flipped sign in the seeding shows up here before anywhere else.
    let trivial = sources
        .iter()
        .filter_map(|(n, sc)| {
            targets
                .iter()
                .find(|(tn, _)| tn == n)
                .map(|(_, tc)| sc.saturating_add(*tc))
        })
        .min();
    if let Some(t) = trivial {
        assert!(
            r.cost_ms <= t,
            "route costs {} ms but the shared node is reachable for {t} ms",
            r.cost_ms
        );
    }

    // Lead in from the snap point to the node the search actually left by, and
    // lead out from the node it arrived at to the destination snap point.
    let fe = from.edge as usize;
    let te = to.edge as usize;
    let from_len = g.length[fe] as f64;
    let to_len = g.length[te] as f64;

    let (lead_in_m, mut lead_in) = if r.from_node == g.head[fe] {
        (
            from_len - from.offset_m,
            sub_polyline(g, fe, from.offset_m, from_len, m),
        )
    } else {
        (from.offset_m, sub_polyline(g, fe, 0.0, from.offset_m, m))
    };
    if r.from_node != g.head[fe] {
        lead_in.reverse();
    }

    let (lead_out_m, mut lead_out) = if r.to_node == g.edge_source(te) {
        (to.offset_m, sub_polyline(g, te, 0.0, to.offset_m, m))
    } else {
        (
            to_len - to.offset_m,
            sub_polyline(g, te, to.offset_m, to_len, m),
        )
    };
    if r.to_node != g.edge_source(te) {
        lead_out.reverse();
    }

    let mut geometry = lead_in;
    geometry.extend(r.geometry(g));
    geometry.extend(lead_out);
    // A degenerate lead leaves a repeated point where it met the first edge.
    geometry.dedup();

    Some(CoordRoute {
        cost_ms: r.cost_ms,
        distance_m: lead_in_m + r.distance_m + lead_out_m,
        geometry,
        from,
        to,
        stats: r.stats,
        same_edge: false,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use graph::grid::Grid;

    fn grid_graph() -> Graph {
        crate::tests::grid_graph()
    }

    /// Snapping a node's own coordinate must route identically to routing by
    /// node id. Any drift is a seeding bug.
    #[test]
    fn coordinate_routing_matches_node_routing() {
        let g = grid_graph();
        let grid = Grid::build(&g);
        let m = grid.metric();
        let mut s = Search::new(g.n_nodes());
        let mut checked = 0;
        for a in 0..36u32 {
            for b in 0..36u32 {
                if a == b {
                    continue;
                }
                let want = s.dijkstra(&g, a, b).map(|r| r.cost_ms);
                let (sa, sb) = (
                    grid.nearest(&g, g.coord(a), 100.0).unwrap(),
                    grid.nearest(&g, g.coord(b), 100.0).unwrap(),
                );
                for alg in Alg::ALL {
                    let got = route(&mut s, &g, &m, sa, sb, alg).map(|r| r.cost_ms);
                    assert_eq!(want, got, "{} disagrees on {a} -> {b}", alg.name());
                }
                checked += 1;
            }
        }
        assert_eq!(checked, 36 * 35);
    }

    #[test]
    fn two_points_on_one_edge_never_touch_the_graph() {
        let g = grid_graph();
        let grid = Grid::build(&g);
        let m = grid.metric();
        let mut s = Search::new(g.n_nodes());
        // Two points a quarter and three quarters along the row-0 edge 0-1.
        let a = g.coord(0);
        let b = g.coord(1);
        let q = |t: f64| (a.0 + t * (b.0 - a.0), a.1 + t * (b.1 - a.1));
        let s1 = grid.nearest(&g, q(0.25), 100.0).unwrap();
        let s2 = grid.nearest(&g, q(0.75), 100.0).unwrap();
        assert_eq!(
            s1.edge, s2.edge,
            "both should land on the same canonical edge"
        );

        let fwd = route(&mut s, &g, &m, s1, s2, Alg::Dijkstra).unwrap();
        assert!(fwd.same_edge);
        assert_eq!(
            fwd.stats.nodes_settled, 0,
            "the graph should not be searched"
        );
        // Half of a 1000 ms edge, give or take the f32 geometry.
        assert!((fwd.cost_ms as i64 - 500).abs() < 40, "{}", fwd.cost_ms);

        // The other way round is the same road travelled backwards.
        let back = route(&mut s, &g, &m, s2, s1, Alg::Dijkstra).unwrap();
        assert!(back.same_edge);
        assert_eq!(back.cost_ms, fwd.cost_ms);
        let mut r = back.geometry.clone();
        r.reverse();
        assert_eq!(r, fwd.geometry);
    }

    #[test]
    fn a_one_way_edge_seeds_only_its_head() {
        // 0 -> 1 one way, 1 <-> 2 two way.
        let g = Graph::from_edges(
            &[(76.70, 30.70), (76.71, 30.70), (76.72, 30.70)],
            &[(0, 1, 1000), (1, 2, 1000), (2, 1, 1000)],
        );
        let e = (0..g.n_edges())
            .find(|e| g.edge_source(*e) == 0 && g.head[*e] == 1)
            .unwrap();
        assert_eq!(g.twin[e], NO_TWIN, "0->1 should be one-way");
        let snap = Snap {
            edge: e as u32,
            point: (76.705, 30.70),
            distance_m: 0.0,
            offset_m: g.length[e] as f64 / 2.0,
            cost_to_source_ms: 500,
            cost_to_head_ms: 500,
        };
        let seeds = source_seeds(&g, &snap);
        assert_eq!(seeds, vec![(1, 500)], "a one-way must not seed its tail");
        let targets = target_seeds(&g, &snap);
        assert_eq!(
            targets,
            vec![(0, 500)],
            "a one-way is only reachable from its tail"
        );
    }

    #[test]
    fn adjacent_edges_do_not_beat_the_shared_node() {
        // The assertion inside `route` is the real test; this exercises it on
        // every adjacent pair in the grid.
        let g = grid_graph();
        let grid = Grid::build(&g);
        let m = grid.metric();
        let mut s = Search::new(g.n_nodes());
        for n in 0..36u32 {
            for e in g.out_edges(n) {
                let head = g.head[e];
                let mid = |a: (f64, f64), b: (f64, f64)| ((a.0 + b.0) / 2.0, (a.1 + b.1) / 2.0);
                let p1 = mid(g.coord(n), g.coord(head));
                for e2 in g.out_edges(head) {
                    let p2 = mid(g.coord(head), g.coord(g.head[e2]));
                    let s1 = grid.nearest(&g, p1, 200.0).unwrap();
                    let s2 = grid.nearest(&g, p2, 200.0).unwrap();
                    let _ = route(&mut s, &g, &m, s1, s2, Alg::Bidir);
                }
            }
        }
    }
}
