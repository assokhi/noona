//! Shortest paths over the CSR graph.
//!
//! Costs are milliseconds (u32) so comparisons are exact - the correctness
//! gate is integer equality against Dijkstra, not equality within an epsilon.
//!
//! Every search is seeded: it starts from a set of `(node, cost_already_spent)`
//! pairs and ends at another such set. Node-to-node routing is the case where
//! both sets hold one entry at cost zero; coordinate routing is the case where
//! they hold the endpoints of a snapped edge at their partial costs. Nothing is
//! ever inserted into the graph.

pub mod coord;

use graph::Graph;
use std::cmp::Reverse;
use std::collections::BinaryHeap;

pub const UNREACHED: u32 = u32::MAX;

/// A search entry point: a node, and the cost already spent getting to it (or,
/// on the target side, the cost still to spend after leaving it).
pub type Seed = (u32, u32);

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SearchStats {
    /// Nodes popped with a key that was not stale.
    pub nodes_settled: u32,
    pub edges_relaxed: u32,
}

#[derive(Clone, Debug)]
pub struct Route {
    /// Travel time in milliseconds, including any seed costs. This is the value
    /// the correctness gate compares, and it must match Dijkstra exactly.
    pub cost_ms: u32,
    /// Length of the graph edges walked. Excludes partial seed legs; coordinate
    /// routing adds those.
    pub distance_m: f64,
    /// Edge ids in travel order.
    pub edges: Vec<u32>,
    /// The seed nodes the route actually entered and left by.
    pub from_node: u32,
    pub to_node: u32,
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

fn seed_cost(seeds: &[Seed], v: u32) -> Option<u32> {
    seeds.iter().find(|(n, _)| *n == v).map(|(_, c)| *c)
}

/// Reusable scratch space. Allocating and clearing 40k-entry arrays per query
/// would dominate p50 and make every later speedup look smaller than it is, so
/// only the entries a search actually touched get reset.
///
/// Not shareable across concurrent requests - the API keeps a pool.
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

    /// Roughly 20 bytes per node of scratch space.
    pub fn bytes(&self) -> usize {
        (self.dist.len() + self.parent.len() + self.h_cache.len()) * 4
            + (self.dist_b.len() + self.parent_b.len()) * 4
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

    fn seed_forward(&mut self, sources: &[Seed]) {
        for (v, c) in sources {
            if *c < self.dist[*v as usize] {
                if self.dist[*v as usize] == UNREACHED {
                    self.touched.push(*v);
                }
                self.dist[*v as usize] = *c;
                self.heap.push(Reverse((*c, *v)));
            }
        }
    }

    // -- node to node, the thin wrappers -----------------------------------

    pub fn dijkstra(&mut self, g: &Graph, source: u32, target: u32) -> Option<Route> {
        self.dijkstra_multi(g, &[(source, 0)], &[(target, 0)])
    }
    pub fn astar(&mut self, g: &Graph, source: u32, target: u32) -> Option<Route> {
        self.astar_multi(g, &[(source, 0)], &[(target, 0)], g.coord(target))
    }
    pub fn bidirectional(&mut self, g: &Graph, source: u32, target: u32) -> Option<Route> {
        self.bidirectional_multi(g, &[(source, 0)], &[(target, 0)])
    }

    // -- seeded searches ----------------------------------------------------

    /// Plain Dijkstra. Lazy deletion rather than decrease-key: duplicates get
    /// pushed and stale pops are skipped, which is less code and faster in
    /// practice on a graph this sparse.
    pub fn dijkstra_multi(
        &mut self,
        g: &Graph,
        sources: &[Seed],
        targets: &[Seed],
    ) -> Option<Route> {
        self.reset();
        let mut stats = SearchStats::default();
        self.seed_forward(sources);

        let (mut best, mut best_node) = (u32::MAX, UNREACHED);
        while let Some(Reverse((d, u))) = self.heap.pop() {
            if d > self.dist[u as usize] {
                continue; // stale
            }
            // Nothing settled from here on can beat what we already have.
            if d >= best {
                break;
            }
            stats.nodes_settled += 1;
            if let Some(tc) = seed_cost(targets, u) {
                let total = d.saturating_add(tc);
                if total < best {
                    best = total;
                    best_node = u;
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
            }
        }
        (best_node != UNREACHED).then(|| self.build_route(g, sources, best, best_node, stats))
    }

    /// A* with a haversine heuristic toward `goal`.
    ///
    /// `goal` must lower-bound every target seed: for each target `t` with seed
    /// cost `tc`, the straight-line time from any node to `goal` has to be at
    /// most `dist(node, t) + tc`. Coordinate routing satisfies this because the
    /// targets are the two ends of the destination edge and the goal is the
    /// snapped point between them. Passing an unrelated goal makes the
    /// heuristic inadmissible and the answer wrong.
    ///
    /// The heuristic divides by the *maximum* edge speed in the graph, taken
    /// from the file header. Dividing by the average instead is the classic
    /// bug: the heuristic stops being admissible, A* stops being optimal, and
    /// it fails silently - plausible routes that are quietly a little wrong.
    /// Comparing costs against Dijkstra is what catches it.
    pub fn astar_multi(
        &mut self,
        g: &Graph,
        sources: &[Seed],
        targets: &[Seed],
        goal: (f64, f64),
    ) -> Option<Route> {
        self.reset();
        let mut stats = SearchStats::default();
        // Metres per millisecond inverted to milliseconds per metre; floor()
        // keeps the estimate at or below the truth after the cast.
        let per_m = 1.0 / g.max_speed_m_per_ms;
        for (v, c) in sources {
            if *c < self.dist[*v as usize] {
                if self.dist[*v as usize] == UNREACHED {
                    self.touched.push(*v);
                }
                self.dist[*v as usize] = *c;
                let h = Self::heuristic(g, &mut self.h_cache, goal, per_m, *v);
                self.heap.push(Reverse((c.saturating_add(h), *v)));
            }
        }

        let (mut best, mut best_node) = (u32::MAX, UNREACHED);
        while let Some(Reverse((f, u))) = self.heap.pop() {
            let d = self.dist[u as usize];
            // Each re-push of a node carries a strictly smaller dist, so any
            // entry above the current f is stale.
            if f > d.saturating_add(Self::heuristic(g, &mut self.h_cache, goal, per_m, u)) {
                continue;
            }
            // f bounds the total cost of anything reachable through u.
            if f >= best {
                break;
            }
            stats.nodes_settled += 1;
            if let Some(tc) = seed_cost(targets, u) {
                let total = d.saturating_add(tc);
                if total < best {
                    best = total;
                    best_node = u;
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
                    let hv = Self::heuristic(g, &mut self.h_cache, goal, per_m, v);
                    self.heap.push(Reverse((nd.saturating_add(hv), v)));
                }
            }
        }
        let route =
            (best_node != UNREACHED).then(|| self.build_route(g, sources, best, best_node, stats));
        if let Some(r) = &route {
            self.assert_admissible(g, sources, r, goal, per_m);
        }
        route
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

    /// The heuristic must never over-estimate the remaining cost at any node on
    /// the route it returned. Catches a unit-conversion slip directly, where the
    /// harness would only surface it as a mismatch count.
    ///
    /// Left debug-only deliberately: this walks the whole path and calls
    /// haversine per node, on every query. The `release-assert` profile is what
    /// makes sure it still runs somewhere - see `make assert`.
    fn assert_admissible(
        &self,
        g: &Graph,
        sources: &[Seed],
        route: &Route,
        goal: (f64, f64),
        per_m: f64,
    ) {
        if !cfg!(debug_assertions) {
            return;
        }
        // The seed cost was already spent before the first edge, so it is not
        // part of what remains from any node on the path.
        let mut spent = seed_cost(sources, route.from_node).unwrap_or(0);
        let mut v = route.from_node;
        for e in &route.edges {
            let remaining = route.cost_ms.saturating_sub(spent);
            let h = (graph::haversine(g.coord(v), goal) * per_m).floor() as u32;
            debug_assert!(
                h <= remaining,
                "heuristic {h} exceeds the true remaining {remaining} ms at node {v}"
            );
            spent += g.weight[*e as usize];
            v = g.head[*e as usize];
        }
    }

    /// Bidirectional Dijkstra: forward on the graph, backward on the reverse
    /// graph, alternating by whichever queue has the smaller key.
    ///
    /// It does *not* stop at the first meeting node. That is the standard wrong
    /// answer: it returns a plausible route that is not the shortest one. Both
    /// searches keep going until `forward_min + backward_min >= mu`, where mu is
    /// the best meeting cost found so far.
    pub fn bidirectional_multi(
        &mut self,
        g: &Graph,
        sources: &[Seed],
        targets: &[Seed],
    ) -> Option<Route> {
        self.reset();
        let mut stats = SearchStats::default();
        self.seed_forward(sources);
        for (v, c) in targets {
            if *c < self.dist_b[*v as usize] {
                if self.dist_b[*v as usize] == UNREACHED {
                    self.touched_b.push(*v);
                }
                self.dist_b[*v as usize] = *c;
                self.heap_b.push(Reverse((*c, *v)));
            }
        }

        let mut mu = u32::MAX;
        let mut meet = UNREACHED;
        // A source that is also a target closes the route with no search at all.
        for (v, c) in sources {
            if let Some(tc) = seed_cost(targets, *v) {
                let total = c.saturating_add(tc);
                if total < mu {
                    mu = total;
                    meet = *v;
                }
            }
        }

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

        // Forward half back to a source seed, then the backward parents out to
        // a target seed.
        let mut edges = Vec::new();
        let mut v = meet;
        while self.parent[v as usize] != UNREACHED {
            let e = self.parent[v as usize];
            edges.push(e);
            v = g.edge_source(e as usize);
        }
        let from_node = v;
        edges.reverse();
        let mut v = meet;
        while self.parent_b[v as usize] != UNREACHED {
            let e = self.parent_b[v as usize];
            edges.push(e);
            v = g.head[e as usize];
        }
        let to_node = v;
        let distance_m = edges.iter().map(|e| g.length[*e as usize] as f64).sum();
        Some(Route {
            cost_ms: mu,
            distance_m,
            edges,
            from_node,
            to_node,
            stats,
        })
    }

    fn build_route(
        &self,
        g: &Graph,
        sources: &[Seed],
        cost_ms: u32,
        target_node: u32,
        stats: SearchStats,
    ) -> Route {
        let mut edges = Vec::new();
        let mut v = target_node;
        while self.parent[v as usize] != UNREACHED {
            let e = self.parent[v as usize];
            edges.push(e);
            v = g.edge_source(e as usize);
        }
        assert!(
            seed_cost(sources, v).is_some(),
            "path walked back to {v}, which is not a source seed"
        );
        edges.reverse();
        let distance_m = edges.iter().map(|e| g.length[*e as usize] as f64).sum();
        Route {
            cost_ms,
            distance_m,
            edges,
            from_node: v,
            to_node: target_node,
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
    pub(crate) fn grid_graph() -> Graph {
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
        let g = grid_graph();
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
        let g = grid_graph();
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
    fn seeds_are_added_to_the_cost() {
        let g = line();
        let mut s = Search::new(g.n_nodes());
        // Half an edge already spent at each end.
        let r = s.dijkstra_multi(&g, &[(0, 400)], &[(2, 600)]).unwrap();
        assert_eq!(r.cost_ms, 400 + 2000 + 600);
        assert_eq!(r.from_node, 0);
        assert_eq!(r.to_node, 2);
    }

    #[test]
    fn the_cheaper_of_two_seeds_wins() {
        let g = line();
        let mut s = Search::new(g.n_nodes());
        // Entering at node 1 costs 5000 up front; entering at 0 costs nothing.
        let r = s
            .dijkstra_multi(&g, &[(0, 0), (1, 5000)], &[(2, 0)])
            .unwrap();
        assert_eq!(r.cost_ms, 2000);
        assert_eq!(r.from_node, 0);
        // Now make node 1 the bargain.
        let r = s
            .dijkstra_multi(&g, &[(0, 3000), (1, 10)], &[(2, 0)])
            .unwrap();
        assert_eq!(r.cost_ms, 1010);
        assert_eq!(r.from_node, 1);
    }

    #[test]
    fn seeded_searches_agree_across_algorithms() {
        let g = grid_graph();
        let mut s = Search::new(g.n_nodes());
        // Targets are the two ends of one edge and the goal sits between them,
        // which is the shape coordinate routing always produces. An arbitrary
        // goal would make the heuristic inadmissible - see astar_multi.
        let (t0, t1) = (34u32, 35u32);
        let (c0, c1) = (g.coord(t0), g.coord(t1));
        let goal = ((c0.0 + c1.0) / 2.0, (c0.1 + c1.1) / 2.0);
        for (sa, sb) in [(0u32, 7u32), (13, 20), (2, 9)] {
            let sources = [(sa, 250), (sb, 900)];
            let targets = [(t0, 500), (t1, 500)];
            let d = s.dijkstra_multi(&g, &sources, &targets).unwrap();
            let a = s.astar_multi(&g, &sources, &targets, goal).unwrap();
            let b = s.bidirectional_multi(&g, &sources, &targets).unwrap();
            assert_eq!(d.cost_ms, a.cost_ms, "astar disagrees on seeded {sa}/{sb}");
            assert_eq!(d.cost_ms, b.cost_ms, "bidir disagrees on seeded {sa}/{sb}");
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
