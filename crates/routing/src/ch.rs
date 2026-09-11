//! Contraction Hierarchies.
//!
//! Every node gets a rank. Contracting node `v` removes it and adds a shortcut
//! `(u, w)` for each incoming `(u, v)` and outgoing `(v, w)` whose path through
//! `v` is the *only* shortest one - a witness search decides that. The query
//! then relaxes only edges to higher-ranked nodes in each direction, which
//! turns a search over the whole city into a short climb up the hierarchy and
//! back down.
//!
//! Three things here are easy to get wrong, and all three are guarded:
//!
//! - The witness search must be bounded or preprocessing never finishes.
//!   Stopping early is *safe*: it inserts a shortcut that was not needed, so
//!   the answer stays correct and only the query gets slightly slower.
//! - Neither direction of the query stops at the meeting node. The meeting node
//!   is the top of the hierarchy for that route, not a node on the shortest
//!   path in the original graph.
//! - Unpacking is iterative. Shortcut nesting is deeper than it looks and a
//!   recursive unpack overflows the stack on real data.

use crate::{Search, SearchStats, Seed, UNREACHED};
use graph::Graph;
use std::cmp::Reverse;
use std::collections::BinaryHeap;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

const MAGIC: &[u8; 8] = b"CHDCHIER";
/// Marks an arc that is an original graph edge rather than a shortcut.
pub const NO_MIDDLE: u32 = u32::MAX;

/// Witness search limits. Whichever bites first ends the search.
#[derive(Clone, Copy, Debug)]
pub struct Limits {
    pub hops: u32,
    pub settled: u32,
}

impl Default for Limits {
    fn default() -> Self {
        Limits {
            hops: 5,
            settled: 500,
        }
    }
}

/// One directed arc in the working graph during contraction.
#[derive(Clone, Copy)]
struct Arc {
    to: u32,
    weight: u32,
    middle: u32,
    /// Original graph edge id, or `u32::MAX` on a shortcut.
    edge: u32,
    /// How many original edges this arc stands for.
    original: u32,
    /// Arcs are cleared rather than removed: contraction only deletes whole
    /// nodes, and compacting every adjacency each time would dominate the run.
    dead: bool,
}

/// The contracted graph: for each node, arcs to strictly higher-ranked nodes.
/// `down` is the same on the reverse graph, so the backward search climbs too.
pub struct Ch {
    pub rank: Vec<u32>,
    pub up_offsets: Vec<u32>,
    pub up_head: Vec<u32>,
    pub up_weight: Vec<u32>,
    pub up_middle: Vec<u32>,
    pub up_edge: Vec<u32>,
    pub down_offsets: Vec<u32>,
    /// Tail of an incoming edge: `down_head[i]` has an edge *to* the node that
    /// owns slot `i`.
    pub down_head: Vec<u32>,
    pub down_weight: Vec<u32>,
    pub down_middle: Vec<u32>,
    pub down_edge: Vec<u32>,
}

#[derive(Default, Debug, Clone, Copy)]
pub struct ChStats {
    pub nodes: usize,
    pub original_edges: usize,
    pub shortcuts: usize,
    pub witness_searches: u64,
    pub witnesses_found: u64,
    pub prep_ms: f64,
    pub arcs_per_node: f64,
    pub max_level: u32,
}

type Pending = (u32, u32, u32, u32); // from, to, weight, original count

/// Shortcuts that contracting `v` would require. Shared by the trial run and
/// the real one so they can never disagree.
fn shortcuts_for(
    v: u32,
    out: &[Vec<Arc>],
    inc: &[Vec<Arc>],
    contracted: &[bool],
    w: &mut Witness,
    limits: Limits,
    stats: &mut ChStats,
) -> (Vec<Pending>, usize, usize) {
    let live = |list: &Vec<Arc>| -> Vec<Arc> {
        list.iter()
            .filter(|a| !a.dead && !contracted[a.to as usize])
            .copied()
            .collect()
    };
    let ins = live(&inc[v as usize]);
    let outs = live(&out[v as usize]);
    let mut needed = Vec::new();

    for a in &ins {
        // The longest hop out of v bounds every witness search from this u, so
        // one Dijkstra per incoming arc covers all its partners.
        let max_out = outs
            .iter()
            .filter(|b| b.to != a.to)
            .map(|b| b.weight)
            .max();
        let Some(max_out) = max_out else { continue };
        stats.witness_searches += 1;
        w.run(
            out,
            contracted,
            a.to,
            v,
            a.weight.saturating_add(max_out),
            limits,
        );

        for b in &outs {
            if b.to == a.to {
                continue; // a shortcut to yourself is never a shortest path
            }
            let direct = a.weight.saturating_add(b.weight);
            if w.dist_to(b.to) <= direct {
                stats.witnesses_found += 1;
                continue; // something else is at least as good
            }
            needed.push((a.to, b.to, direct, a.original + b.original));
        }
    }
    (needed, ins.len(), outs.len())
}

/// Edge difference dominates; contracted neighbours spread the contraction out
/// spatially instead of eating one neighbourhood at a time; level keeps the
/// hierarchy shallow; original edges keep unpacking cheap.
fn priority(needed: usize, ins: usize, outs: usize, neighbours: u32, level: u32, orig: i64) -> i64 {
    let edge_difference = needed as i64 - ins as i64 - outs as i64;
    5 * edge_difference + 2 * neighbours as i64 + level as i64 + orig / 4
}

pub fn build(g: &Graph, limits: Limits) -> (Ch, ChStats) {
    let n = g.n_nodes();
    let t0 = std::time::Instant::now();

    let mut out: Vec<Vec<Arc>> = vec![Vec::new(); n];
    let mut inc: Vec<Vec<Arc>> = vec![Vec::new(); n];
    for u in 0..n as u32 {
        for e in g.out_edges(u) {
            let v = g.head[e];
            let arc = Arc {
                to: v,
                weight: g.weight[e],
                middle: NO_MIDDLE,
                edge: e as u32,
                original: 1,
                dead: false,
            };
            out[u as usize].push(arc);
            inc[v as usize].push(Arc { to: u, ..arc });
        }
    }

    let mut stats = ChStats {
        nodes: n,
        original_edges: g.n_edges(),
        ..Default::default()
    };
    let mut contracted = vec![false; n];
    let mut level = vec![0u32; n];
    let mut neighbours = vec![0u32; n];
    let mut rank = vec![u32::MAX; n];
    let mut w = Witness::new(n);

    let mut queue: BinaryHeap<Reverse<(i64, u32)>> = BinaryHeap::with_capacity(n);
    for v in 0..n as u32 {
        let (needed, i, o) = shortcuts_for(v, &out, &inc, &contracted, &mut w, limits, &mut stats);
        let orig: i64 = needed.iter().map(|s| s.3 as i64).sum();
        queue.push(Reverse((priority(needed.len(), i, o, 0, 0, orig), v)));
    }

    let mut next_rank = 0u32;
    while let Some(Reverse((p, v))) = queue.pop() {
        if contracted[v as usize] {
            continue;
        }
        // Lazy update: recompute now that neighbours have moved, and if this is
        // no longer the minimum put it back. Rebuilding every priority after
        // each contraction is quadratic and does not finish.
        let (needed, i, o) = shortcuts_for(v, &out, &inc, &contracted, &mut w, limits, &mut stats);
        let orig: i64 = needed.iter().map(|s| s.3 as i64).sum();
        let fresh = priority(
            needed.len(),
            i,
            o,
            neighbours[v as usize],
            level[v as usize],
            orig,
        );
        if fresh > p {
            if let Some(&Reverse((next, _))) = queue.peek() {
                if fresh > next {
                    queue.push(Reverse((fresh, v)));
                    continue;
                }
            }
        }

        // Contract for real.
        for (from, to, weight, original) in needed {
            let arc = Arc {
                to,
                weight,
                middle: v,
                edge: u32::MAX,
                original,
                dead: false,
            };
            // A parallel shortcut can already exist from an earlier
            // contraction; keep the cheaper one rather than growing the graph.
            if let Some(old) = out[from as usize]
                .iter_mut()
                .find(|a| !a.dead && a.to == to && a.weight >= weight)
            {
                *old = arc;
                if let Some(back) = inc[to as usize]
                    .iter_mut()
                    .find(|a| !a.dead && a.to == from && a.weight >= weight)
                {
                    *back = Arc { to: from, ..arc };
                }
                continue;
            }
            if out[from as usize]
                .iter()
                .any(|a| !a.dead && a.to == to && a.weight <= weight)
            {
                continue;
            }
            out[from as usize].push(arc);
            inc[to as usize].push(Arc { to: from, ..arc });
            stats.shortcuts += 1;
        }

        contracted[v as usize] = true;
        rank[v as usize] = next_rank;
        next_rank += 1;

        for a in out[v as usize].iter().chain(inc[v as usize].iter()) {
            if a.dead || contracted[a.to as usize] {
                continue;
            }
            neighbours[a.to as usize] += 1;
            level[a.to as usize] = level[a.to as usize].max(level[v as usize] + 1);
        }
        stats.max_level = stats.max_level.max(level[v as usize]);
    }

    // Split into the two upward graphs the query uses.
    let mut up: Vec<Vec<Arc>> = vec![Vec::new(); n];
    let mut down: Vec<Vec<Arc>> = vec![Vec::new(); n];
    for u in 0..n {
        for a in &out[u] {
            if !a.dead && rank[a.to as usize] > rank[u] {
                up[u].push(*a);
            }
        }
        for a in &inc[u] {
            if !a.dead && rank[a.to as usize] > rank[u] {
                down[u].push(*a);
            }
        }
    }

    let (up_offsets, up_head, up_weight, up_middle, up_edge) = flatten(&up);
    let (down_offsets, down_head, down_weight, down_middle, down_edge) = flatten(&down);
    stats.prep_ms = t0.elapsed().as_secs_f64() * 1000.0;
    stats.arcs_per_node = (up_head.len() + down_head.len()) as f64 / n as f64;

    (
        Ch {
            rank,
            up_offsets,
            up_head,
            up_weight,
            up_middle,
            up_edge,
            down_offsets,
            down_head,
            down_weight,
            down_middle,
            down_edge,
        },
        stats,
    )
}

type Flat = (Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>);

fn flatten(adj: &[Vec<Arc>]) -> Flat {
    let mut offsets = Vec::with_capacity(adj.len() + 1);
    let (mut head, mut weight, mut middle, mut edge) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for list in adj {
        offsets.push(head.len() as u32);
        for a in list {
            head.push(a.to);
            weight.push(a.weight);
            middle.push(a.middle);
            edge.push(a.edge);
        }
    }
    offsets.push(head.len() as u32);
    (offsets, head, weight, middle, edge)
}

/// Bounded forward Dijkstra over uncontracted nodes only.
struct Witness {
    dist: Vec<u32>,
    hops: Vec<u32>,
    touched: Vec<u32>,
    heap: BinaryHeap<Reverse<(u32, u32)>>,
}

impl Witness {
    fn new(n: usize) -> Witness {
        Witness {
            dist: vec![UNREACHED; n],
            hops: vec![0; n],
            touched: Vec::new(),
            heap: BinaryHeap::new(),
        }
    }
    fn dist_to(&self, v: u32) -> u32 {
        self.dist[v as usize]
    }
    fn run(
        &mut self,
        out: &[Vec<Arc>],
        contracted: &[bool],
        source: u32,
        avoid: u32,
        budget: u32,
        limits: Limits,
    ) {
        for v in self.touched.drain(..) {
            self.dist[v as usize] = UNREACHED;
            self.hops[v as usize] = 0;
        }
        self.heap.clear();
        self.dist[source as usize] = 0;
        self.touched.push(source);
        self.heap.push(Reverse((0, source)));

        let mut settled = 0u32;
        while let Some(Reverse((d, u))) = self.heap.pop() {
            if d > self.dist[u as usize] || d > budget {
                continue;
            }
            settled += 1;
            if settled > limits.settled {
                return; // safe: we simply fail to find a witness
            }
            if self.hops[u as usize] >= limits.hops {
                continue;
            }
            for a in &out[u as usize] {
                if a.dead || contracted[a.to as usize] || a.to == avoid {
                    continue;
                }
                let nd = d.saturating_add(a.weight);
                if nd <= budget && nd < self.dist[a.to as usize] {
                    if self.dist[a.to as usize] == UNREACHED {
                        self.touched.push(a.to);
                    }
                    self.dist[a.to as usize] = nd;
                    self.hops[a.to as usize] = self.hops[u as usize] + 1;
                    self.heap.push(Reverse((nd, a.to)));
                }
            }
        }
    }
}

impl Ch {
    pub fn n_nodes(&self) -> usize {
        self.rank.len()
    }
    pub fn n_shortcuts(&self) -> usize {
        self.up_middle.iter().filter(|m| **m != NO_MIDDLE).count()
            + self.down_middle.iter().filter(|m| **m != NO_MIDDLE).count()
    }
    /// The node that owns upward slot `i`, recovered from the offsets.
    fn up_source(&self, i: usize) -> u32 {
        (self.up_offsets.partition_point(|o| *o as usize <= i) - 1) as u32
    }
    fn down_source(&self, i: usize) -> u32 {
        (self.down_offsets.partition_point(|o| *o as usize <= i) - 1) as u32
    }

    /// Original graph edges behind an arc `from -> to`, in travel order.
    ///
    /// Iterative with an explicit stack: shortcut nesting gets deeper than it
    /// looks and the recursive form overflows on real data.
    fn unpack(&self, from: u32, to: u32, middle: u32, edge: u32, out: &mut Vec<u32>) {
        let mut stack = vec![(from, to, middle, edge)];
        while let Some((a, b, m, e)) = stack.pop() {
            if m == NO_MIDDLE {
                out.push(e);
                continue;
            }
            // m was contracted before both ends, so a -> m is stored as an
            // incoming arc of m from a higher-ranked node, and m -> b as an
            // outgoing one. Push in reverse so they pop in travel order.
            let second = self
                .arc_up(m, b)
                .expect("second half of a shortcut must exist");
            let first = self
                .arc_down(m, a)
                .expect("first half of a shortcut must exist");
            stack.push(second);
            stack.push(first);
        }
    }

    /// The arc `m -> b` among m's upward arcs.
    fn arc_up(&self, m: u32, b: u32) -> Option<(u32, u32, u32, u32)> {
        (self.up_offsets[m as usize] as usize..self.up_offsets[m as usize + 1] as usize)
            .filter(|i| self.up_head[*i] == b)
            .min_by_key(|i| self.up_weight[*i])
            .map(|i| (m, b, self.up_middle[i], self.up_edge[i]))
    }
    /// The arc `a -> m`, stored on m's downward list as tail `a`.
    fn arc_down(&self, m: u32, a: u32) -> Option<(u32, u32, u32, u32)> {
        (self.down_offsets[m as usize] as usize..self.down_offsets[m as usize + 1] as usize)
            .filter(|i| self.down_head[*i] == a)
            .min_by_key(|i| self.down_weight[*i])
            .map(|i| (a, m, self.down_middle[i], self.down_edge[i]))
    }

    pub fn save(&self, path: &Path, source_hash: u64) -> io::Result<()> {
        let mut w = BufWriter::new(File::create(path)?);
        w.write_all(MAGIC)?;
        w.write_all(&source_hash.to_le_bytes())?;
        for v in self.arrays() {
            w.write_all(&(v.len() as u32).to_le_bytes())?;
            let mut buf = Vec::with_capacity(v.len() * 4);
            for x in v.iter() {
                buf.extend_from_slice(&x.to_le_bytes());
            }
            w.write_all(&buf)?;
        }
        w.flush()
    }

    fn arrays(&self) -> [&Vec<u32>; 11] {
        [
            &self.rank,
            &self.up_offsets,
            &self.up_head,
            &self.up_weight,
            &self.up_middle,
            &self.up_edge,
            &self.down_offsets,
            &self.down_head,
            &self.down_weight,
            &self.down_middle,
            &self.down_edge,
        ]
    }

    pub fn load(path: &Path) -> io::Result<(Ch, u64)> {
        let mut r = BufReader::new(File::open(path)?);
        let mut magic = [0u8; 8];
        r.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(io::Error::new(io::ErrorKind::InvalidData, "not a CH file"));
        }
        let mut b8 = [0u8; 8];
        r.read_exact(&mut b8)?;
        let source_hash = u64::from_le_bytes(b8);
        let mut next = || -> io::Result<Vec<u32>> {
            let mut b4 = [0u8; 4];
            r.read_exact(&mut b4)?;
            let len = u32::from_le_bytes(b4) as usize;
            let mut buf = vec![0u8; len * 4];
            r.read_exact(&mut buf)?;
            Ok(buf
                .as_chunks::<4>()
                .0
                .iter()
                .map(|c| u32::from_le_bytes(*c))
                .collect())
        };
        Ok((
            Ch {
                rank: next()?,
                up_offsets: next()?,
                up_head: next()?,
                up_weight: next()?,
                up_middle: next()?,
                up_edge: next()?,
                down_offsets: next()?,
                down_head: next()?,
                down_weight: next()?,
                down_middle: next()?,
                down_edge: next()?,
            },
            source_hash,
        ))
    }
}

impl Search {
    /// Bidirectional CH query, returning cost and the unpacked original edges.
    ///
    /// Neither direction stops when it meets the other: the meeting node is the
    /// top of the hierarchy for this route, not a node on the shortest path in
    /// the original graph.
    pub fn ch_multi(
        &mut self,
        ch: &Ch,
        sources: &[Seed],
        targets: &[Seed],
    ) -> Option<(u32, Vec<u32>, SearchStats)> {
        self.reset();
        let mut stats = SearchStats::default();
        for (v, c) in sources {
            if *c < self.dist[*v as usize] {
                if self.dist[*v as usize] == UNREACHED {
                    self.touched.push(*v);
                }
                self.dist[*v as usize] = *c;
                self.heap.push(Reverse((*c, *v)));
            }
        }
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
        for (v, c) in sources {
            if let Some(tc) = targets.iter().find(|(t, _)| t == v).map(|(_, c)| *c) {
                if c.saturating_add(tc) < mu {
                    mu = c.saturating_add(tc);
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
            if fmin.min(bmin) >= mu {
                break;
            }

            if fmin <= bmin {
                let Reverse((d, u)) = self.heap.pop().expect("non-empty heap");
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
                for i in ch.up_offsets[u as usize] as usize..ch.up_offsets[u as usize + 1] as usize {
                    stats.edges_relaxed += 1;
                    let (v, nd) = (ch.up_head[i], d.saturating_add(ch.up_weight[i]));
                    if nd < self.dist[v as usize] {
                        if self.dist[v as usize] == UNREACHED {
                            self.touched.push(v);
                        }
                        self.dist[v as usize] = nd;
                        self.parent[v as usize] = i as u32;
                        self.heap.push(Reverse((nd, v)));
                    }
                }
            } else {
                let Reverse((d, u)) = self.heap_b.pop().expect("non-empty heap");
                if d > self.dist_b[u as usize] {
                    continue;
                }
                stats.nodes_settled += 1;
                if self.dist[u as usize] != UNREACHED {
                    let cand = d.saturating_add(self.dist[u as usize]);
                    if cand < mu {
                        mu = cand;
                        meet = u;
                    }
                }
                for i in
                    ch.down_offsets[u as usize] as usize..ch.down_offsets[u as usize + 1] as usize
                {
                    stats.edges_relaxed += 1;
                    let (v, nd) = (ch.down_head[i], d.saturating_add(ch.down_weight[i]));
                    if nd < self.dist_b[v as usize] {
                        if self.dist_b[v as usize] == UNREACHED {
                            self.touched_b.push(v);
                        }
                        self.dist_b[v as usize] = nd;
                        self.parent_b[v as usize] = i as u32;
                        self.heap_b.push(Reverse((nd, v)));
                    }
                }
            }
        }

        if meet == UNREACHED {
            return None;
        }

        // Climb back down both halves, unpacking every shortcut on the way.
        let mut up_arcs: Vec<usize> = Vec::new();
        let mut v = meet;
        while self.parent[v as usize] != UNREACHED {
            let i = self.parent[v as usize] as usize;
            up_arcs.push(i);
            v = ch.up_source(i);
        }
        let mut edges = Vec::new();
        for i in up_arcs.into_iter().rev() {
            ch.unpack(
                ch.up_source(i),
                ch.up_head[i],
                ch.up_middle[i],
                ch.up_edge[i],
                &mut edges,
            );
        }
        let mut v = meet;
        while self.parent_b[v as usize] != UNREACHED {
            let i = self.parent_b[v as usize] as usize;
            // down slot i is owned by the *head* of the original edge.
            ch.unpack(
                v,
                ch.down_source(i),
                ch.down_middle[i],
                ch.down_edge[i],
                &mut edges,
            );
            v = ch.down_source(i);
        }
        Some((mu, edges, stats))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ch_agrees_with_dijkstra_on_every_grid_pair() {
        let g = crate::tests::grid_graph();
        let (ch, stats) = build(&g, Limits::default());
        assert_eq!(stats.nodes, g.n_nodes());
        // Every node got a rank, and ranks are a permutation.
        let mut seen = ch.rank.clone();
        seen.sort_unstable();
        assert_eq!(seen, (0..g.n_nodes() as u32).collect::<Vec<_>>());

        let mut s = Search::new(g.n_nodes());
        let mut checked = 0;
        for a in 0..36u32 {
            for b in 0..36u32 {
                if a == b {
                    continue;
                }
                let want = s.dijkstra(&g, a, b).map(|r| r.cost_ms);
                let got = s.ch_multi(&ch, &[(a, 0)], &[(b, 0)]).map(|(c, _, _)| c);
                assert_eq!(want, got, "CH disagrees on {a} -> {b}");
                checked += 1;
            }
        }
        assert_eq!(checked, 36 * 35);
    }

    #[test]
    fn unpacked_path_is_a_real_walk_of_the_original_graph() {
        let g = crate::tests::grid_graph();
        let (ch, _) = build(&g, Limits::default());
        let mut s = Search::new(g.n_nodes());
        for (a, b) in [(0u32, 35u32), (35, 0), (5, 30), (17, 3), (12, 23)] {
            let (cost, edges, _) = s.ch_multi(&ch, &[(a, 0)], &[(b, 0)]).unwrap();
            let mut v = a;
            let mut total = 0;
            for e in &edges {
                assert_eq!(g.edge_source(*e as usize), v, "unpacked edges do not chain");
                total += g.weight[*e as usize];
                v = g.head[*e as usize];
            }
            assert_eq!(v, b, "unpacked path does not end at the target");
            assert_eq!(total, cost, "unpacked path cost differs from the CH cost");
        }
    }

    #[test]
    fn a_tight_witness_bound_is_still_correct() {
        // Cutting the witness search short may add needless shortcuts, but it
        // must never change an answer.
        let g = crate::tests::grid_graph();
        let (tight, _) = build(
            &g,
            Limits {
                hops: 1,
                settled: 1,
            },
        );
        let mut s = Search::new(g.n_nodes());
        for a in 0..36u32 {
            for b in 0..36u32 {
                if a == b {
                    continue;
                }
                let want = s.dijkstra(&g, a, b).map(|r| r.cost_ms);
                let got = s.ch_multi(&tight, &[(a, 0)], &[(b, 0)]).map(|(c, _, _)| c);
                assert_eq!(want, got, "tight-witness CH disagrees on {a} -> {b}");
            }
        }
    }

    #[test]
    fn ch_round_trips() {
        let g = crate::tests::grid_graph();
        let (ch, _) = build(&g, Limits::default());
        let dir = std::env::temp_dir().join(format!("chd-ch-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("ch.bin");
        ch.save(&path, 7).unwrap();
        let (back, hash) = Ch::load(&path).unwrap();
        assert_eq!(hash, 7);
        assert_eq!(back.rank, ch.rank);
        assert_eq!(back.up_head, ch.up_head);
        assert_eq!(back.down_weight, ch.down_weight);
        std::fs::remove_dir_all(&dir).ok();
    }
}
