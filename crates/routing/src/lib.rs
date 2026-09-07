//! Shortest paths over the CSR graph.
//!
//! Costs are milliseconds (u32) so comparisons are exact - the correctness
//! gate is integer equality against Dijkstra, not equality within an epsilon.

use graph::Graph;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

pub const UNREACHED: u32 = u32::MAX;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SearchStats {
    /// Nodes popped with a key that was not stale.
    pub nodes_settled: u32,
    pub edges_relaxed: u32,
}

#[derive(Clone, Debug)]
pub struct Route {
    /// Travel time in milliseconds. This is the value the correctness gate
    /// compares, and it must match Dijkstra exactly.
    pub cost_ms: u32,
    pub distance_m: f64,
    /// Edge ids in travel order.
    pub edges: Vec<u32>,
    pub stats: SearchStats,
}

impl Route {
    pub fn duration_s(&self) -> f64 {
        self.cost_ms as f64 / 1000.0
    }
    /// The route as `(lon, lat)` points, ready for a GeoJSON LineString.
    pub fn geometry(&self, g: &Graph) -> Vec<[f32; 2]> {
        let mut pts: Vec<[f32; 2]> = Vec::new();
        for (i, e) in self.edges.iter().enumerate() {
            // Each edge starts on the node the previous one ended on.
            let skip = usize::from(i > 0);
            pts.extend(g.geometry(*e as usize).skip(skip));
        }
        pts
    }
}

/// Reusable scratch space. Allocating and clearing 40k-entry arrays per query
/// would dominate p50 and make every later speedup look smaller than it is, so
/// only the entries a search actually touched get reset.
pub struct Search {
    dist: Vec<u32>,
    parent: Vec<u32>,
    touched: Vec<u32>,
    heap: BinaryHeap<Reverse<(u32, u32)>>,
    /// Haversine is trigonometry; without this the heuristic gets recomputed
    /// on every push and every pop and A* loses more to trig than it saves in
    /// settled nodes.
    h_cache: Vec<u32>,
    dist_b: Vec<u32>,
    parent_b: Vec<u32>,
    touched_b: Vec<u32>,
    heap_b: BinaryHeap<Reverse<(u32, u32)>>,
}

impl Search {
    pub fn new(n_nodes: usize) -> Self {
        Search {
            dist: vec![UNREACHED; n_nodes],
            parent: vec![UNREACHED; n_nodes],
            touched: Vec::new(),
            heap: BinaryHeap::new(),
            h_cache: vec![UNREACHED; n_nodes],
            dist_b: vec![UNREACHED; n_nodes],
            parent_b: vec![UNREACHED; n_nodes],
            touched_b: Vec::new(),
            heap_b: BinaryHeap::new(),
        }
    }

    fn reset(&mut self) {
        for v in self.touched.drain(..) {
            self.dist[v as usize] = UNREACHED;
            self.parent[v as usize] = UNREACHED;
            self.h_cache[v as usize] = UNREACHED;
        }
        for v in self.touched_b.drain(..) {
            self.dist_b[v as usize] = UNREACHED;
            self.parent_b[v as usize] = UNREACHED;
        }
        self.heap.clear();
        self.heap_b.clear();
    }

    /// Plain Dijkstra. Lazy deletion rather than decrease-key: duplicates get
    /// pushed and stale pops are skipped, which is less code and faster in
    /// practice on a graph this sparse.
    pub fn dijkstra(&mut self, g: &Graph, source: u32, target: u32) -> Option<Route> {
        self.reset();
        let mut stats = SearchStats::default();
        self.dist[source as usize] = 0;
        self.touched.push(source);
        self.heap.push(Reverse((0, source)));

        while let Some(Reverse((d, u))) = self.heap.pop() {
            if d > self.dist[u as usize] {
                continue; // stale
            }
            stats.nodes_settled += 1;
            if u == target {
                return Some(self.build_route(g, source, target, stats));
            }
            for e in g.out_edges(u) {
                stats.edges_relaxed += 1;
                let v = g.head[e];
                let nd = d + g.weight[e];
                if nd < self.dist[v as usize] {
                    if self.dist[v as usize] == UNREACHED {
                        self.touched.push(v);
                    }
                    self.dist[v as usize] = nd;
                    self.parent[v as usize] = e as u32;
                    self.heap.push(Reverse((nd, v)));
                }
            }
        }
        None
    }

    /// A* with a haversine heuristic.
    ///
    /// The heuristic divides by the *maximum* edge speed in the graph, taken
    /// from the file header. Dividing by the average instead is the classic
    /// bug: the heuristic stops being admissible, A* stops being optimal, and
    /// it fails silently - plausible routes that are quietly a little wrong.
    /// Comparing costs against Dijkstra is what catches it.
    pub fn astar(&mut self, g: &Graph, source: u32, target: u32) -> Option<Route> {
        self.reset();
        let mut stats = SearchStats::default();
        let goal = g.coord(target);
        // Metres per millisecond inverted to milliseconds per metre; floor()
        // keeps the estimate at or below the truth after the cast.
        let per_m = 1.0 / g.max_speed_m_per_ms;

        self.dist[source as usize] = 0;
        self.touched.push(source);
        let h0 = Self::heuristic(g, &mut self.h_cache, goal, per_m, source);
        self.heap.push(Reverse((h0, source)));

        while let Some(Reverse((f, u))) = self.heap.pop() {
            let d = self.dist[u as usize];
            // Each re-push of a node carries a strictly smaller dist, so any
            // entry above the current f is stale.
            if f > d.saturating_add(Self::heuristic(g, &mut self.h_cache, goal, per_m, u)) {
                continue;
            }
            stats.nodes_settled += 1;
            if u == target {
                let route = self.build_route(g, source, target, stats);
                self.assert_admissible(g, &route, source, goal, per_m);
                return Some(route);
            }
            for e in g.out_edges(u) {
                stats.edges_relaxed += 1;
                let v = g.head[e];
                let nd = d + g.weight[e];
                if nd < self.dist[v as usize] {
                    if self.dist[v as usize] == UNREACHED {
                        self.touched.push(v);
                    }
                    self.dist[v as usize] = nd;
                    self.parent[v as usize] = e as u32;
                    let hv = Self::heuristic(g, &mut self.h_cache, goal, per_m, v);
                    self.heap.push(Reverse((nd.saturating_add(hv), v)));
                }
            }
        }
        None
    }

    /// Debug-only: the heuristic must never over-estimate the remaining cost at
    /// any node on the route it returned. Catches a unit-conversion slip
    /// directly, where the harness would only surface it as a mismatch count.
    fn assert_admissible(
        &self,
        g: &Graph,
        route: &Route,
        source: u32,
        goal: (f64, f64),
        per_m: f64,
    ) {
        if !cfg!(debug_assertions) {
            return;
        }
        let mut spent = 0u32;
        let mut v = source;
        for e in &route.edges {
            let remaining = route.cost_ms - spent;
            let h = (graph::haversine(g.coord(v), goal) * per_m).floor() as u32;
            debug_assert!(
                h <= remaining,
                "heuristic {h} exceeds the true remaining {remaining} ms at node {v}"
            );
            spent += g.weight[*e as usize];
            v = g.head[*e as usize];
        }
    }

    /// Milliseconds from `v` to the goal at the fastest speed anything in the
    /// graph travels. Memoised per query.
    fn heuristic(g: &Graph, cache: &mut [u32], goal: (f64, f64), per_m: f64, v: u32) -> u32 {
        let slot = &mut cache[v as usize];
        if *slot == UNREACHED {
            *slot = (graph::haversine(g.coord(v), goal) * per_m).floor() as u32;
        }
        *slot
    }

    /// Bidirectional Dijkstra: forward on the graph, backward on the reverse
    /// graph, alternating by whichever queue has the smaller key.
    ///
    /// It does *not* stop at the first meeting node. That is the standard wrong
    /// answer: it returns a plausible route that is not the shortest one. Both
    /// searches keep going until `forward_min + backward_min >= mu`, where mu is
    /// the best meeting cost found so far.
    pub fn bidirectional(&mut self, g: &Graph, source: u32, target: u32) -> Option<Route> {
        self.reset();
        let stats = SearchStats::default();
        if source == target {
            return Some(Route {
                cost_ms: 0,
                distance_m: 0.0,
                edges: Vec::new(),
                stats,
            });
        }
        let mut stats = stats;

        self.dist[source as usize] = 0;
        self.touched.push(source);
        self.heap.push(Reverse((0, source)));
        self.dist_b[target as usize] = 0;
        self.touched_b.push(target);
        self.heap_b.push(Reverse((0, target)));

        let mut mu = u32::MAX;
        let mut meet = UNREACHED;

        loop {
            let fmin = self.heap.peek().map_or(u32::MAX, |Reverse((k, _))| *k);
            let bmin = self.heap_b.peek().map_or(u32::MAX, |Reverse((k, _))| *k);
            if fmin == u32::MAX && bmin == u32::MAX {
                break;
            }
            // The stopping criterion. Not the first meeting.
            if fmin.saturating_add(bmin) >= mu {
                break;
            }

            if fmin <= bmin {
                let Reverse((d, u)) = self.heap.pop().expect("fmin came from a non-empty heap");
                if d > self.dist[u as usize] {
                    continue;
                }
                stats.nodes_settled += 1;
                if self.dist_b[u as usize] != UNREACHED {
                    let cand = d.saturating_add(self.dist_b[u as usize]);
                    if cand < mu {
                        mu = cand;
                        meet = u;
                    }
                }
                for e in g.out_edges(u) {
                    stats.edges_relaxed += 1;
                    let v = g.head[e];
                    let nd = d + g.weight[e];
                    if nd < self.dist[v as usize] {
                        if self.dist[v as usize] == UNREACHED {
                            self.touched.push(v);
                        }
                        self.dist[v as usize] = nd;
                        self.parent[v as usize] = e as u32;
                        self.heap.push(Reverse((nd, v)));
                    }
                    if self.dist_b[v as usize] != UNREACHED {
                        let cand = self.dist[v as usize].saturating_add(self.dist_b[v as usize]);
                        if cand < mu {
                            mu = cand;
                            meet = v;
                        }
                    }
                }
            } else {
                let Reverse((d, v)) = self.heap_b.pop().expect("bmin came from a non-empty heap");
                if d > self.dist_b[v as usize] {
                    continue;
                }
                stats.nodes_settled += 1;
                if self.dist[v as usize] != UNREACHED {
                    let cand = d.saturating_add(self.dist[v as usize]);
                    if cand < mu {
                        mu = cand;
                        meet = v;
                    }
                }
                for j in g.in_edges(v) {
                    stats.edges_relaxed += 1;
                    let e = g.r_edge[j];
                    let u = g.r_head[j];
                    let nd = d + g.weight[e as usize];
                    if nd < self.dist_b[u as usize] {
                        if self.dist_b[u as usize] == UNREACHED {
                            self.touched_b.push(u);
                        }
                        self.dist_b[u as usize] = nd;
                        // The forward edge u -> v: one step towards the target.
                        self.parent_b[u as usize] = e;
                        self.heap_b.push(Reverse((nd, u)));
                    }
                    if self.dist[u as usize] != UNREACHED {
                        let cand = self.dist[u as usize].saturating_add(self.dist_b[u as usize]);
                        if cand < mu {
                            mu = cand;
                            meet = u;
                        }
                    }
                }
            }
        }

        if meet == UNREACHED {
            return None;
        }

        // Forward half back to the source, then the backward parents out to
        // the target.
        let mut edges = Vec::new();
        let mut v = meet;
        while v != source {
            let e = self.parent[v as usize];
            edges.push(e);
            v = g.edge_source(e as usize);
        }
        edges.reverse();
        let mut v = meet;
        while v != target {
            let e = self.parent_b[v as usize];
            edges.push(e);
            v = g.head[e as usize];
        }
        let distance_m = edges.iter().map(|e| g.length[*e as usize] as f64).sum();
        Some(Route {
            cost_ms: mu,
            distance_m,
            edges,
            stats,
        })
    }

    fn build_route(&self, g: &Graph, source: u32, target: u32, stats: SearchStats) -> Route {
        let mut edges = Vec::new();
        let mut v = target;
        while v != source {
            let e = self.parent[v as usize];
            debug_assert_ne!(e, UNREACHED, "no parent edge on the settled path");
            edges.push(e);
            v = g.edge_source(e as usize);
        }
        edges.reverse();
        let distance_m = edges.iter().map(|e| g.length[*e as usize] as f64).sum();
        Route {
            cost_ms: self.dist[target as usize],
            distance_m,
            edges,
            stats,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 0 -> 1 -> 2 in two hops, or 0 -> 2 direct but slower.
    fn line() -> Graph {
        Graph::from_edges(
            &[(76.70, 30.70), (76.71, 30.70), (76.72, 30.70)],
            &[(0, 1, 1000), (1, 2, 1000), (0, 2, 2500)],
        )
    }

    /// A grid with ties everywhere, plus one row that is one-way eastbound so
    /// the reverse graph genuinely differs from the forward one.
    fn grid() -> Graph {
        let mut coords = Vec::new();
        for row in 0..6 {
            for col in 0..6 {
                coords.push((76.70 + col as f64 * 0.004, 30.70 + row as f64 * 0.004));
            }
        }
        let idx = |r: u32, c: u32| r * 6 + c;
        let mut edges = Vec::new();
        for r in 0..6u32 {
            for c in 0..6u32 {
                if c + 1 < 6 {
                    edges.push((idx(r, c), idx(r, c + 1), 1000));
                    if r != 2 {
                        edges.push((idx(r, c + 1), idx(r, c), 1000));
                    }
                }
                if r + 1 < 6 {
                    edges.push((idx(r, c), idx(r + 1, c), 1200));
                    edges.push((idx(r + 1, c), idx(r, c), 1200));
                }
            }
        }
        Graph::from_edges(&coords, &edges)
    }

    #[test]
    fn dijkstra_takes_the_cheaper_two_hop() {
        let g = line();
        let mut s = Search::new(g.n_nodes());
        let r = s.dijkstra(&g, 0, 2).unwrap();
        assert_eq!(r.cost_ms, 2000);
        assert_eq!(r.edges.len(), 2);
        assert_eq!(r.duration_s(), 2.0);
    }

    #[test]
    fn unreachable_is_none() {
        let g = Graph::from_edges(&[(76.70, 30.70), (76.71, 30.70)], &[(0, 1, 1000)]);
        let mut s = Search::new(g.n_nodes());
        assert!(s.dijkstra(&g, 1, 0).is_none());
        assert!(s.astar(&g, 1, 0).is_none());
        assert!(s.bidirectional(&g, 1, 0).is_none());
    }

    #[test]
    fn astar_and_bidir_agree_with_dijkstra_on_every_grid_pair() {
        let g = grid();
        let mut s = Search::new(g.n_nodes());
        let mut checked = 0;
        for a in 0..36u32 {
            for b in 0..36u32 {
                if a == b {
                    continue;
                }
                let d = s.dijkstra(&g, a, b).map(|r| r.cost_ms);
                let ast = s.astar(&g, a, b).map(|r| r.cost_ms);
                let bi = s.bidirectional(&g, a, b).map(|r| r.cost_ms);
                assert_eq!(d, ast, "astar disagrees on {a} -> {b}");
                assert_eq!(d, bi, "bidirectional disagrees on {a} -> {b}");
                checked += 1;
            }
        }
        assert_eq!(checked, 36 * 35);
    }

    #[test]
    fn every_route_is_a_walkable_chain_of_edges() {
        let g = grid();
        let mut s = Search::new(g.n_nodes());
        for (a, b) in [(0u32, 35u32), (35, 0), (5, 30), (17, 3)] {
            for r in [
                s.dijkstra(&g, a, b).unwrap(),
                s.astar(&g, a, b).unwrap(),
                s.bidirectional(&g, a, b).unwrap(),
            ] {
                let mut v = a;
                let mut cost = 0;
                for e in &r.edges {
                    assert_eq!(g.edge_source(*e as usize), v, "edges do not chain");
                    cost += g.weight[*e as usize];
                    v = g.head[*e as usize];
                }
                assert_eq!(v, b, "route does not end at the target");
                assert_eq!(cost, r.cost_ms, "reported cost is not the path cost");
            }
        }
    }

    #[test]
    fn scratch_space_survives_reuse() {
        let g = line();
        let mut s = Search::new(g.n_nodes());
        let first = s.dijkstra(&g, 0, 2).unwrap().cost_ms;
        let _ = s.dijkstra(&g, 1, 2);
        let _ = s.bidirectional(&g, 1, 2);
        // A stale entry left behind would make the repeat disagree.
        assert_eq!(s.dijkstra(&g, 0, 2).unwrap().cost_ms, first);
        assert_eq!(s.astar(&g, 0, 2).unwrap().cost_ms, first);
    }
}
