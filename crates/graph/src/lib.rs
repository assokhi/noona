//! In-memory CSR routing graph: construction from OSM ways, largest-SCC
//! filtering, and the versioned on-disk format.
//!
//! Conventions, asserted at every boundary:
//! - coordinates are `(lon, lat)`, matching GeoJSON
//! - distances are metres (f64), accumulated in f64 and only stored as f32
//! - edge weights are milliseconds (u32), so the priority queue stays integral
//! - node ids are dense u32 indices, NOT OSM ids (`osm_id` is a debug side table)

use osm_parse::{Oneway, Way};
use std::collections::{HashMap, HashSet};
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

/// IUGG mean earth radius.
pub const EARTH_RADIUS_M: f64 = 6_371_008.8;

/// Great-circle distance in metres between two `(lon, lat)` points.
pub fn haversine(a: (f64, f64), b: (f64, f64)) -> f64 {
    let (lon1, lat1) = (a.0.to_radians(), a.1.to_radians());
    let (lon2, lat2) = (b.0.to_radians(), b.1.to_radians());
    let (dlon, dlat) = (lon2 - lon1, lat2 - lat1);
    let h = (dlat / 2.0).sin().powi(2) + lat1.cos() * lat2.cos() * (dlon / 2.0).sin().powi(2);
    2.0 * EARTH_RADIUS_M * h.sqrt().asin()
}

/// Sentinel in `name_id` for an edge whose way had no `name` tag.
pub const NO_NAME: u32 = u32::MAX;
/// Sentinel in `twin` for a one-way edge with no opposite direction.
pub const NO_TWIN: u32 = u32::MAX;

/// Set on the reverse direction of a two-way road: it shares its twin's
/// geometry span and walks it backwards. Sits above the class bits that
/// `osm-parse` owns.
pub const FLAG_GEOM_REVERSED: u8 = 0x40;

/// `node_flags` bits.
pub const NODE_SIGNALS: u8 = 0x01;
pub const NODE_BARRIER: u8 = 0x02;

pub struct Graph {
    // --- per node ---
    pub lon: Vec<f64>,
    pub lat: Vec<f64>,
    /// Internal id -> OSM node id. Debugging only; nothing routes on this.
    pub osm_id: Vec<i64>,
    /// Routing-relevant node tags, so the contraction pass knows not to splice
    /// them away. See `NODE_SIGNALS` / `NODE_BARRIER`.
    pub node_flags: Vec<u8>,

    // --- forward CSR, indexed by node then by edge ---
    /// Length `n_nodes + 1`.
    pub offsets: Vec<u32>,
    pub head: Vec<u32>,
    /// Travel time in milliseconds.
    pub weight: Vec<u32>,
    /// Length in metres, summed along the polyline.
    pub length: Vec<f32>,
    /// Start of this edge's polyline in `geom`. Two directions of one road
    /// share a span, so this is a start/len pair rather than a prefix array.
    pub geom_start: Vec<u32>,
    pub geom_len: Vec<u32>,
    pub flags: Vec<u8>,
    pub name_id: Vec<u32>,

    // --- reverse CSR: at node v, the edges arriving at v ---
    /// Length `n_nodes + 1`.
    pub r_offsets: Vec<u32>,
    /// Tail node of the incoming edge.
    pub r_head: Vec<u32>,
    /// Forward edge id, so weight and geometry are one indirection away
    /// instead of duplicated.
    pub r_edge: Vec<u32>,

    /// The opposite direction of the same road segment, or `NO_TWIN` for a
    /// one-way. Lets road length be counted once per segment, and is what the
    /// geometry arena will share a span on.
    pub twin: Vec<u32>,

    /// Flat `(lon, lat)` polyline points, both endpoints included. Stored once
    /// per road, forward-oriented.
    pub geom: Vec<[f32; 2]>,
    pub names: Vec<String>,

    /// Fastest edge in the graph, metres per millisecond, fixed at build time
    /// and carried in the file header. A* divides by this, so it must be the
    /// value the weights were actually built with rather than something
    /// recomputed on a load path that could later drift.
    pub max_speed_m_per_ms: f64,
}

impl Graph {
    pub fn n_nodes(&self) -> usize {
        self.lon.len()
    }
    pub fn n_edges(&self) -> usize {
        self.head.len()
    }
    /// Edge ids leaving `v`.
    pub fn out_edges(&self, v: u32) -> std::ops::Range<usize> {
        self.offsets[v as usize] as usize..self.offsets[v as usize + 1] as usize
    }
    /// Reverse-CSR slots arriving at `v`.
    pub fn in_edges(&self, v: u32) -> std::ops::Range<usize> {
        self.r_offsets[v as usize] as usize..self.r_offsets[v as usize + 1] as usize
    }
    /// The edge's polyline in travel order.
    pub fn geometry(&self, edge: usize) -> Polyline<'_> {
        let start = self.geom_start[edge] as usize;
        Polyline {
            span: &self.geom[start..start + self.geom_len[edge] as usize],
            reversed: self.flags[edge] & FLAG_GEOM_REVERSED != 0,
            next: 0,
        }
    }
    pub fn name(&self, edge: usize) -> Option<&str> {
        match self.name_id[edge] {
            NO_NAME => None,
            i => Some(&self.names[i as usize]),
        }
    }
    pub fn coord(&self, v: u32) -> (f64, f64) {
        (self.lon[v as usize], self.lat[v as usize])
    }
    /// A stable id for the undirected road segment behind a directed edge:
    /// the lower of the edge and its twin.
    pub fn pair_of(&self, edge: usize) -> u32 {
        match self.twin[edge] {
            NO_TWIN => edge as u32,
            t => (edge as u32).min(t),
        }
    }
    /// Polyline segments (point pairs) across distinct roads. Splicing joins
    /// polylines end to end without adding or removing a segment, so this is
    /// exactly invariant under contraction - unlike the raw point count, which
    /// drops by one per joint because the joint was stored twice.
    pub fn geom_segments(&self) -> usize {
        (0..self.n_edges())
            .filter(|e| self.flags[*e] & FLAG_GEOM_REVERSED == 0)
            .map(|e| self.geom_len[e] as usize - 1)
            .sum()
    }
    /// Road length in metres, counting a two-way segment once.
    pub fn road_length_m(&self) -> f64 {
        (0..self.n_edges())
            .filter(|e| self.twin[*e] == NO_TWIN || (*e as u32) < self.twin[*e])
            .map(|e| self.length[e] as f64)
            .sum()
    }
}

/// An edge's points in travel order. The reverse direction of a two-way road
/// shares its twin's span and yields it backwards, so this cannot be a slice.
pub struct Polyline<'a> {
    span: &'a [[f32; 2]],
    reversed: bool,
    next: usize,
}

impl Iterator for Polyline<'_> {
    type Item = [f32; 2];
    fn next(&mut self) -> Option<[f32; 2]> {
        let p = self.span.get(if self.reversed {
            self.span.len().checked_sub(self.next + 1)?
        } else {
            self.next
        })?;
        self.next += 1;
        Some(*p)
    }
    fn size_hint(&self) -> (usize, Option<usize>) {
        let n = self.span.len() - self.next;
        (n, Some(n))
    }
}

impl ExactSizeIterator for Polyline<'_> {}

// ---------------------------------------------------------------------------
// topology
// ---------------------------------------------------------------------------

/// A degree-2 node and the edges that would be spliced if it were contracted.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Chain {
    pub u: u32,
    pub w: u32,
    /// `(edge u->v, edge v->w)`, in travel order.
    pub fwd: (u32, u32),
    /// `(edge w->v, edge v->u)`, present only when the chain is two-way.
    pub bwd: Option<(u32, u32)>,
}

/// A node's local topology.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Shape {
    /// `u -> v -> w` and nothing else.
    OneWayChain,
    /// `u <-> v <-> w` and nothing else.
    TwoWayChain,
    /// Exactly one undirected neighbour: the tip of a cul-de-sac. Legitimate.
    CulDeSac,
    /// No way in, or no way out. Cannot occur inside the largest SCC; counted
    /// separately so it is never confused with a chain interior.
    Stub,
    /// A real junction, or an asymmetry that encodes a directional constraint.
    Junction,
}

/// The chain through `v`, if `v` is a topologically degree-2 node that can be
/// spliced away without changing any shortest path.
///
/// Deliberately strict. `u->v`, `v->w`, `w->v` with no `v->u` is *not* a chain:
/// that asymmetry is a real one-way constraint and splicing it would invent a
/// turn that does not exist.
pub fn chain_at(g: &Graph, v: u32) -> Option<Chain> {
    let outs: Vec<(u32, u32)> = g.out_edges(v).map(|e| (e as u32, g.head[e])).collect();
    let ins: Vec<(u32, u32)> = g.in_edges(v).map(|j| (g.r_edge[j], g.r_head[j])).collect();
    // A self-loop is never contractible.
    if outs.iter().any(|(_, h)| *h == v) || ins.iter().any(|(_, t)| *t == v) {
        return None;
    }
    match (ins.len(), outs.len()) {
        (1, 1) => {
            let ((e_in, u), (e_out, w)) = (ins[0], outs[0]);
            // u == w is a cul-de-sac tip, not a chain: splicing makes a self-loop.
            (u != w).then_some(Chain {
                u,
                w,
                fwd: (e_in, e_out),
                bwd: None,
            })
        }
        (2, 2) => {
            let mut nb: Vec<u32> = ins.iter().map(|(_, t)| *t).collect();
            nb.sort_unstable();
            nb.dedup();
            if nb.len() != 2 {
                return None;
            }
            let (u, w) = (nb[0], nb[1]);
            // Every edge must pair with a reverse, or the node is a constraint.
            let e_uv = ins.iter().find(|(_, t)| *t == u)?.0;
            let e_wv = ins.iter().find(|(_, t)| *t == w)?.0;
            let e_vu = outs.iter().find(|(_, h)| *h == u)?.0;
            let e_vw = outs.iter().find(|(_, h)| *h == w)?.0;
            Some(Chain {
                u,
                w,
                fwd: (e_uv, e_vw),
                bwd: Some((e_wv, e_vu)),
            })
        }
        _ => None,
    }
}

pub fn shape_at(g: &Graph, v: u32) -> Shape {
    if g.in_edges(v).is_empty() || g.out_edges(v).is_empty() {
        return Shape::Stub;
    }
    if let Some(c) = chain_at(g, v) {
        return if c.bwd.is_some() {
            Shape::TwoWayChain
        } else {
            Shape::OneWayChain
        };
    }
    let mut nb: Vec<u32> = g
        .out_edges(v)
        .map(|e| g.head[e])
        .chain(g.in_edges(v).map(|j| g.r_head[j]))
        .collect();
    nb.sort_unstable();
    nb.dedup();
    if nb.len() == 1 && nb[0] != v {
        return Shape::CulDeSac;
    }
    Shape::Junction
}

// ---------------------------------------------------------------------------
// construction
// ---------------------------------------------------------------------------

/// One directed edge as emitted from a way, still keyed by OSM node id.
struct PendingEdge {
    a: i64,
    b: i64,
    weight: u32,
    length: f32,
    flags: u8,
    name_id: u32,
    geom: Vec<[f32; 2]>,
}

/// One directed edge before it is packed into CSR.
struct RawEdge {
    src: u32,
    dst: u32,
    weight: u32,
    length: f32,
    flags: u8,
    name_id: u32,
    geom: Vec<[f32; 2]>,
}

#[derive(Default, Debug)]
pub struct BuildStats {
    pub parse: osm_parse::ParseStats,
    /// Distinct OSM nodes referenced by retained ways.
    pub referenced_nodes: usize,
    /// Of those, the ones that survive degree-2 contraction.
    pub intersection_nodes: usize,
    pub nodes_kept: usize,
    pub edges_before_scc: usize,
    pub edges: usize,
    pub scc_count: usize,
    pub scc_node_fraction: f64,
    pub geometry_points: usize,
    pub road_length_km: f64,
    pub contract: ContractStats,
    /// Way node refs whose coordinates were not in the extract. Should be 0
    /// with complete_ways; anything else means the clip was wrong.
    pub missing_coords: usize,
}

impl BuildStats {
    /// How many referenced nodes collapse into one intersection node.
    pub fn contraction_ratio(&self) -> f64 {
        self.referenced_nodes as f64 / self.intersection_nodes.max(1) as f64
    }
}

pub fn build(pbf: &Path) -> Result<(Graph, BuildStats), Box<dyn std::error::Error>> {
    let (ways, parse) = osm_parse::read_ways(pbf)?;
    let mut stats = BuildStats {
        parse,
        ..Default::default()
    };

    // Pass 1: reference counts over retained ways decide what is a junction.
    let mut refc: HashMap<i64, u32> = HashMap::new();
    for w in &ways {
        for n in &w.nodes {
            *refc.entry(*n).or_insert(0) += 1;
        }
    }
    stats.referenced_nodes = refc.len();
    let needed: HashSet<i64> = refc.keys().copied().collect();

    // Pass 2: coordinates for those nodes only, plus routing-relevant node tags.
    let (coords, tagged) = osm_parse::read_nodes(pbf, &needed)?;

    // A junction is shared by two or more ways, or is a way endpoint, or carries
    // a tag that routing cares about. Everything else is just geometry.
    let mut is_junction: HashSet<i64> = refc
        .iter()
        .filter(|(_, c)| **c >= 2)
        .map(|(k, _)| *k)
        .collect();
    for w in &ways {
        is_junction.insert(w.nodes[0]);
        is_junction.insert(*w.nodes.last().unwrap());
    }
    is_junction.extend(&tagged.signals);
    is_junction.extend(&tagged.barriers);
    is_junction.retain(|id| coords.contains_key(id));
    stats.intersection_nodes = is_junction.len();

    // Pass 3: split each way into runs of geometry between consecutive junctions.
    let mut raw: Vec<PendingEdge> = Vec::new();
    let mut names: Vec<String> = Vec::new();
    let mut name_ids: HashMap<String, u32> = HashMap::new();

    for w in &ways {
        let name_id = match &w.name {
            Some(n) => *name_ids.entry(n.clone()).or_insert_with(|| {
                names.push(n.clone());
                (names.len() - 1) as u32
            }),
            None => NO_NAME,
        };
        let mps = w.speed_kmh / 3.6;
        emit_way(w, name_id, mps, &coords, &is_junction, &mut raw, &mut stats);
    }
    stats.edges_before_scc = raw.len();

    // Dense ids, assigned in sorted OSM-id order so a rebuild is deterministic.
    let mut used: Vec<i64> = raw.iter().flat_map(|e| [e.a, e.b]).collect();
    used.sort_unstable();
    used.dedup();
    let dense: HashMap<i64, u32> = used
        .iter()
        .enumerate()
        .map(|(i, id)| (*id, i as u32))
        .collect();
    let n = used.len();

    let edges: Vec<RawEdge> = raw
        .into_iter()
        .map(|e| RawEdge {
            src: dense[&e.a],
            dst: dense[&e.b],
            weight: e.weight,
            length: e.length,
            flags: e.flags,
            name_id: e.name_id,
            geom: e.geom,
        })
        .collect();

    // Largest strongly connected component. OSM is full of isolated fragments -
    // a service road with a mistagged oneway, a way whose only connection sat
    // outside the bbox. Without this you get unroutable pairs at random.
    let offsets = csr_offsets(n, edges.iter().map(|e| e.src));
    let mut by_src: Vec<u32> = (0..edges.len() as u32).collect();
    by_src.sort_by_key(|i| edges[*i as usize].src);
    let head: Vec<u32> = by_src.iter().map(|i| edges[*i as usize].dst).collect();
    let (comp, sizes) = scc(n, &offsets, &head);
    let largest = sizes
        .iter()
        .enumerate()
        .max_by_key(|(_, s)| **s)
        .map(|(i, _)| i as u32)
        .unwrap_or(0);
    stats.scc_count = sizes.len();
    stats.scc_node_fraction =
        sizes.get(largest as usize).copied().unwrap_or(0) as f64 / n.max(1) as f64;

    let keep: Vec<bool> = comp.iter().map(|c| *c == largest).collect();
    let mut remap = vec![u32::MAX; n];
    let mut lon = Vec::new();
    let mut lat = Vec::new();
    let mut osm_id = Vec::new();
    let mut node_flags = Vec::new();
    for (old, osm) in used.iter().enumerate() {
        if keep[old] {
            remap[old] = lon.len() as u32;
            let c = coords[osm];
            lon.push(c.0);
            lat.push(c.1);
            osm_id.push(*osm);
            let mut f = 0u8;
            if tagged.signals.contains(osm) {
                f |= NODE_SIGNALS;
            }
            if tagged.barriers.contains(osm) {
                f |= NODE_BARRIER;
            }
            node_flags.push(f);
        }
    }
    let kept: Vec<RawEdge> = edges
        .into_iter()
        .filter(|e| keep[e.src as usize] && keep[e.dst as usize])
        .map(|mut e| {
            e.src = remap[e.src as usize];
            e.dst = remap[e.dst as usize];
            e
        })
        .collect();

    stats.nodes_kept = lon.len();
    let g = assemble(lon, lat, osm_id, node_flags, kept, names);

    // OSM splits ways at every tagging change, so plenty of what the refcount
    // rule called a junction is topologically degree 2. Splice those away.
    let (g, c) = contract(g);
    stats.contract = c;
    stats.edges = g.n_edges();
    stats.geometry_points = g.geom.len();
    stats.road_length_km = c.road_m_after / 1000.0;
    Ok((g, stats))
}

/// Walk one way, emitting an edge per run of geometry between two consecutive
/// junction nodes. Edge length is summed along the polyline, never taken from
/// the straight line between endpoints.
#[allow(clippy::too_many_arguments)]
fn emit_way(
    w: &Way,
    name_id: u32,
    mps: f64,
    coords: &HashMap<i64, (f64, f64)>,
    is_junction: &HashSet<i64>,
    out: &mut Vec<PendingEdge>,
    stats: &mut BuildStats,
) {
    let mut start: Option<i64> = None;
    let mut poly: Vec<[f32; 2]> = Vec::new();
    let mut prev: Option<(f64, f64)> = None;
    let mut acc = 0.0f64;

    for &nid in &w.nodes {
        let Some(&c) = coords.get(&nid) else {
            // Should not happen with complete_ways; break the run if it does.
            stats.missing_coords += 1;
            start = None;
            poly.clear();
            prev = None;
            acc = 0.0;
            continue;
        };
        let p = [c.0 as f32, c.1 as f32];

        if start.is_none() {
            if is_junction.contains(&nid) {
                start = Some(nid);
                poly.clear();
                poly.push(p);
                prev = Some(c);
                acc = 0.0;
            }
            continue;
        }

        acc += haversine(prev.unwrap(), c);
        poly.push(p);
        prev = Some(c);

        if !is_junction.contains(&nid) {
            continue;
        }

        let a = start.unwrap();
        if a != nid && acc > 0.0 {
            let ms = ((acc / mps) * 1000.0 * w.penalty as f64).round().max(1.0) as u32;
            let forward = w.oneway != Oneway::Reverse;
            let reverse = w.oneway != Oneway::Forward;
            if forward {
                out.push(PendingEdge {
                    a,
                    b: nid,
                    weight: ms,
                    length: acc as f32,
                    flags: w.flags,
                    name_id,
                    geom: poly.clone(),
                });
            }
            if reverse {
                let mut back = poly.clone();
                back.reverse();
                out.push(PendingEdge {
                    a: nid,
                    b: a,
                    weight: ms,
                    length: acc as f32,
                    flags: w.flags,
                    name_id,
                    geom: back,
                });
            }
        }
        start = Some(nid);
        poly.clear();
        poly.push(p);
        acc = 0.0;
    }
}

#[derive(Default, Debug, Clone, Copy)]
pub struct ContractStats {
    pub rounds: usize,
    pub nodes_before: usize,
    pub nodes_after: usize,
    pub edges_before: usize,
    pub edges_after: usize,
    pub geom_before: usize,
    pub geom_after: usize,
    pub segments_before: usize,
    pub segments_after: usize,
    pub road_m_before: f64,
    pub road_m_after: f64,
    /// Joints spliced out. Each removes exactly one duplicated polyline point.
    pub splices: usize,
    /// Degree-2 rings and lollipops that kept one node so nothing collapsed
    /// to a self-loop.
    pub rings_kept: usize,
    /// Degree-2 nodes kept because they carry a signal or barrier tag.
    pub tagged_kept: usize,
    /// Degree-2 nodes kept because their two segments disagree about being
    /// two-way. See `twins_agree`.
    pub mixed_kept: usize,
}

/// Splice out every topologically degree-2 node, preserving length, geometry
/// and every shortest path.
///
/// Works by walking maximal runs of contractible nodes from their boundary
/// rather than contracting one node at a time, so a single round already
/// reaches the fixpoint; the loop is belt and braces and reports its count.
pub fn contract(g: Graph) -> (Graph, ContractStats) {
    let mut stats = ContractStats {
        nodes_before: g.n_nodes(),
        edges_before: g.n_edges(),
        geom_before: g.geom.len(),
        segments_before: g.geom_segments(),
        road_m_before: g.road_length_m(),
        ..Default::default()
    };
    let n = g.n_nodes();

    let chain: Vec<Option<Chain>> = (0..n as u32)
        .map(|v| {
            // A tagged node is a real feature even when it is topologically
            // degree 2 - a gate, or a signal we will want to charge time for.
            if contractible_at(&g, v) {
                chain_at(&g, v)
            } else {
                None
            }
        })
        .collect();
    let mut drop: Vec<bool> = chain.iter().map(|c| c.is_some()).collect();
    for v in 0..n {
        match chain_at(&g, v as u32) {
            Some(_) if g.node_flags[v] != 0 => stats.tagged_kept += 1,
            Some(c) if !twins_agree(&g, &c) => stats.mixed_kept += 1,
            _ => {}
        }
    }

    // A run of contractible nodes whose two ends meet at the same node - or
    // that closes into a ring with no ends at all - would splice down to a
    // self-loop. Keep one node of each so it stays a real piece of road.
    let mut seen = vec![false; n];
    for v in 0..n {
        if !drop[v] || seen[v] {
            continue;
        }
        seen[v] = true;
        let c = chain[v].unwrap();
        let mut ends = [u32::MAX; 2];
        for (i, start) in [c.u, c.w].into_iter().enumerate() {
            let (mut prev, mut cur) = (v as u32, start);
            loop {
                if !drop[cur as usize] {
                    ends[i] = cur;
                    break;
                }
                if seen[cur as usize] {
                    break; // wrapped around a ring
                }
                seen[cur as usize] = true;
                let cc = chain[cur as usize].unwrap();
                let next = if cc.u == prev { cc.w } else { cc.u };
                prev = cur;
                cur = next;
            }
        }
        if ends[0] == ends[1] {
            drop[v] = false;
            stats.rings_kept += 1;
        }
    }

    let mut remap = vec![u32::MAX; n];
    let (mut lon, mut lat, mut osm_id, mut node_flags) =
        (Vec::new(), Vec::new(), Vec::new(), Vec::new());
    for v in 0..n {
        if !drop[v] {
            remap[v] = lon.len() as u32;
            lon.push(g.lon[v]);
            lat.push(g.lat[v]);
            osm_id.push(g.osm_id[v]);
            node_flags.push(g.node_flags[v]);
        }
    }

    // Walk out of every surviving node, swallowing contracted nodes until the
    // next survivor. Each original edge is consumed by exactly one walk.
    struct Spliced {
        src: u32,
        dst: u32,
        weight: u32,
        length: f32,
        flags: u8,
        name_id: u32,
        geom: Vec<[f32; 2]>,
    }
    let mut spliced: Vec<Spliced> = Vec::new();
    for u in 0..n as u32 {
        if drop[u as usize] {
            continue;
        }
        for e in g.out_edges(u) {
            let mut weight = g.weight[e] as u64;
            let mut length = g.length[e] as f64;
            let mut geom: Vec<[f32; 2]> = g.geometry(e).collect();
            let mut cur = g.head[e];
            let mut cur_edge = e as u32;
            while drop[cur as usize] {
                let c = chain[cur as usize].expect("dropped node must be a chain");
                let next = if cur_edge == c.fwd.0 {
                    c.fwd.1
                } else {
                    let bwd = c.bwd.expect("arrived at a one-way chain from behind");
                    debug_assert_eq!(cur_edge, bwd.0);
                    bwd.1
                };
                weight += g.weight[next as usize] as u64;
                length += g.length[next as usize] as f64;
                // Drop the duplicated joint: the previous edge already ended
                // on this node.
                geom.extend(g.geometry(next as usize).skip(1));
                cur = g.head[next as usize];
                cur_edge = next;
                stats.splices += 1;
            }
            debug_assert_ne!(remap[cur as usize], u32::MAX);
            // Silently reversed geometry is the classic bug here: it costs the
            // right amount and draws the wrong line.
            // One comparison per edge, kept in release: silently reversed
            // geometry costs the right amount and draws the wrong line, and a
            // debug_assert here would never run in the build that matters.
            assert!(
                geom[0] == node_point(&g, u) && geom[geom.len() - 1] == node_point(&g, cur),
                "spliced polyline does not run from {u} to {cur}"
            );
            spliced.push(Spliced {
                src: remap[u as usize],
                dst: remap[cur as usize],
                weight: weight.min(u32::MAX as u64) as u32,
                length: length as f32,
                flags: g.flags[e],
                name_id: g.name_id[e],
                geom,
            });
        }
    }

    let edges: Vec<RawEdge> = spliced
        .into_iter()
        .map(|s| RawEdge {
            src: s.src,
            dst: s.dst,
            weight: s.weight,
            length: s.length,
            flags: s.flags,
            name_id: s.name_id,
            geom: s.geom,
        })
        .collect();

    let out = assemble(lon, lat, osm_id, node_flags, edges, g.names);
    stats.rounds = 1;
    stats.nodes_after = out.n_nodes();
    stats.edges_after = out.n_edges();
    stats.geom_after = out.geom.len();
    stats.segments_after = out.geom_segments();
    stats.road_m_after = out.road_length_m();
    (out, stats)
}

/// Both segments either pair with a reverse or neither does.
///
/// A node joining an ordinary two-way road to a stretch mapped as two separate
/// one-way carriageways is topologically degree 2, but splicing it produces one
/// edge per direction that each contain the shared two-way half - so its length
/// gets counted twice and total road length quietly grows. Rare (14 nodes here)
/// and not worth the complication of splitting: leave the node in place.
pub fn twins_agree(g: &Graph, c: &Chain) -> bool {
    match c.bwd {
        // A one-way chain has no reverse edges at all, so nothing can disagree.
        None => true,
        Some((wv, vu)) => g.twin[c.fwd.0 as usize] == vu && g.twin[c.fwd.1 as usize] == wv,
    }
}

/// Would the contraction pass splice this node away? The ring guard is not
/// included: that one depends on the whole run, not on the node.
pub fn contractible_at(g: &Graph, v: u32) -> bool {
    g.node_flags[v as usize] == 0 && chain_at(g, v).is_some_and(|c| twins_agree(g, &c))
}

/// Polyline endpoints are stored as the node coordinate narrowed to f32, so
/// this compares exactly rather than with a tolerance.
fn node_point(g: &Graph, v: u32) -> [f32; 2] {
    [g.lon[v as usize] as f32, g.lat[v as usize] as f32]
}

/// Identity of a directed edge as a physical piece of road.
type EdgeShape = (u32, u32, Vec<(u32, u32)>);

fn shape_key<'a>(src: u32, dst: u32, points: impl Iterator<Item = &'a [f32; 2]>) -> EdgeShape {
    (
        src,
        dst,
        points.map(|p| (p[0].to_bits(), p[1].to_bits())).collect(),
    )
}

/// CSR offsets from an iterator of source ids. Length `n + 1`.
fn csr_offsets(n: usize, srcs: impl Iterator<Item = u32>) -> Vec<u32> {
    let mut off = vec![0u32; n + 1];
    for s in srcs {
        off[s as usize + 1] += 1;
    }
    for i in 1..=n {
        off[i] += off[i - 1];
    }
    off
}

fn assemble(
    lon: Vec<f64>,
    lat: Vec<f64>,
    osm_id: Vec<i64>,
    node_flags: Vec<u8>,
    mut edges: Vec<RawEdge>,
    names: Vec<String>,
) -> Graph {
    let n = lon.len();
    // Deterministic order: by source, then target, then weight.
    edges.sort_by_key(|e| (e.src, e.dst, e.weight));

    let offsets = csr_offsets(n, edges.iter().map(|e| e.src));
    let m = edges.len();
    let mut head = Vec::with_capacity(m);
    let mut weight = Vec::with_capacity(m);
    let mut length = Vec::with_capacity(m);
    let mut flags = Vec::with_capacity(m);
    let mut name_id = Vec::with_capacity(m);
    for e in &edges {
        head.push(e.dst);
        weight.push(e.weight);
        length.push(e.length);
        flags.push(e.flags & !FLAG_GEOM_REVERSED);
        name_id.push(e.name_id);
    }

    // Reverse CSR: bucket every edge by its target.
    let mut rev: Vec<(u32, u32)> = edges
        .iter()
        .enumerate()
        .map(|(i, e)| (e.dst, i as u32))
        .collect();
    rev.sort_unstable();
    let r_offsets = csr_offsets(n, rev.iter().map(|(d, _)| *d));
    let r_head = rev.iter().map(|(_, i)| edges[*i as usize].src).collect();
    let r_edge = rev.iter().map(|(_, i)| *i).collect();

    // Twins, decided by shape rather than by provenance: two directed edges are
    // the same road iff they run between the same nodes over the same polyline,
    // the other way round. Deriving this from the OSM way instead silently fails
    // on a street mapped as two separate one-way ways - a real case here - and
    // that made total road length depend on how a mapper split the geometry.
    // Dual carriageways keep distinct polylines and correctly stay unpaired.
    let mut twin = vec![NO_TWIN; m];
    let mut index: HashMap<EdgeShape, Vec<u32>> = HashMap::with_capacity(m);
    for (i, e) in edges.iter().enumerate() {
        index
            .entry(shape_key(e.src, e.dst, e.geom.iter()))
            .or_default()
            .push(i as u32);
    }
    for (i, e) in edges.iter().enumerate() {
        if twin[i] != NO_TWIN {
            continue;
        }
        let back = shape_key(e.dst, e.src, e.geom.iter().rev());
        let Some(slot) = index.get_mut(&back) else {
            continue;
        };
        // Parallel edges are legal, so take an unclaimed one rather than assuming.
        let Some(pos) = slot.iter().position(|j| twin[*j as usize] == NO_TWIN) else {
            continue;
        };
        let j = slot[pos];
        if j as usize == i {
            continue;
        }
        twin[i] = j;
        twin[j as usize] = i as u32;
    }

    // Store each road's polyline once, forward-oriented, and point the reverse
    // direction at the same span with an invert-on-read flag. Phase 5 roughly
    // doubles the edge count with shortcuts and the geometry arena should not
    // follow.
    let mut geom_start = vec![0u32; m];
    let mut geom_len = vec![0u32; m];
    let mut geom: Vec<[f32; 2]> = Vec::new();
    for (i, e) in edges.iter().enumerate() {
        let t = twin[i];
        if t != NO_TWIN && (t as usize) < i {
            // The twin already wrote the span; walk it the other way.
            geom_start[i] = geom_start[t as usize];
            geom_len[i] = geom_len[t as usize];
            flags[i] |= FLAG_GEOM_REVERSED;
            continue;
        }
        geom_start[i] = geom.len() as u32;
        geom_len[i] = e.geom.len() as u32;
        geom.extend_from_slice(&e.geom);
    }

    let max_speed_m_per_ms = edges
        .iter()
        .map(|e| e.length as f64 / e.weight as f64)
        .fold(0.0, f64::max);
    debug_assert!(
        edges
            .iter()
            .all(|e| e.length as f64 / e.weight as f64 <= max_speed_m_per_ms),
        "an edge is faster than the recorded maximum, which makes A* inadmissible"
    );

    Graph {
        lon,
        lat,
        osm_id,
        node_flags,
        twin,
        offsets,
        head,
        weight,
        length,
        geom_start,
        geom_len,
        flags,
        name_id,
        r_offsets,
        r_head,
        r_edge,
        geom,
        names,
        max_speed_m_per_ms,
    }
}

/// Iterative Tarjan. Returns `(component per node, size per component)`.
/// Iterative because 10^5 nodes will overflow the stack in the recursive form.
pub fn scc(n: usize, offsets: &[u32], head: &[u32]) -> (Vec<u32>, Vec<u32>) {
    const UNVISITED: u32 = u32::MAX;
    let mut index = vec![UNVISITED; n];
    let mut low = vec![0u32; n];
    let mut on_stack = vec![false; n];
    let mut comp = vec![UNVISITED; n];
    let mut sizes: Vec<u32> = Vec::new();
    let mut stack: Vec<u32> = Vec::new();
    let mut call: Vec<(u32, u32)> = Vec::new(); // (node, next edge cursor)
    let mut next_index = 0u32;

    for s in 0..n as u32 {
        if index[s as usize] != UNVISITED {
            continue;
        }
        index[s as usize] = next_index;
        low[s as usize] = next_index;
        next_index += 1;
        stack.push(s);
        on_stack[s as usize] = true;
        call.push((s, offsets[s as usize]));

        while let Some(&(v, ei)) = call.last() {
            if ei < offsets[v as usize + 1] {
                call.last_mut().unwrap().1 = ei + 1;
                let w = head[ei as usize];
                if index[w as usize] == UNVISITED {
                    index[w as usize] = next_index;
                    low[w as usize] = next_index;
                    next_index += 1;
                    stack.push(w);
                    on_stack[w as usize] = true;
                    call.push((w, offsets[w as usize]));
                } else if on_stack[w as usize] {
                    low[v as usize] = low[v as usize].min(index[w as usize]);
                }
                continue;
            }

            call.pop();
            if low[v as usize] == index[v as usize] {
                let id = sizes.len() as u32;
                let mut size = 0u32;
                loop {
                    let w = stack.pop().unwrap();
                    on_stack[w as usize] = false;
                    comp[w as usize] = id;
                    size += 1;
                    if w == v {
                        break;
                    }
                }
                sizes.push(size);
            }
            if let Some(&(parent, _)) = call.last() {
                low[parent as usize] = low[parent as usize].min(low[v as usize]);
            }
        }
    }
    (comp, sizes)
}

// ---------------------------------------------------------------------------
// serialisation: a header plus flat arrays, so loading is read_exact into
// pre-sized vectors rather than deserialising a nested struct graph.
// ---------------------------------------------------------------------------

const MAGIC: &[u8; 8] = b"CHDGRAPH";
pub const FORMAT_VERSION: u32 = 3;

macro_rules! flat_io {
    ($w:ident, $r:ident, $t:ty, $size:expr) => {
        fn $w<W: Write>(out: &mut W, v: &[$t]) -> io::Result<()> {
            let mut buf = Vec::with_capacity(v.len() * $size);
            for x in v {
                buf.extend_from_slice(&x.to_le_bytes());
            }
            out.write_all(&buf)
        }
        fn $r<R: Read>(inp: &mut R, n: usize) -> io::Result<Vec<$t>> {
            let mut buf = vec![0u8; n * $size];
            inp.read_exact(&mut buf)?;
            Ok(buf
                .chunks_exact($size)
                .map(|c| <$t>::from_le_bytes(c.try_into().unwrap()))
                .collect())
        }
    };
}
flat_io!(w_u32, r_u32, u32, 4);
flat_io!(w_f32, r_f32, f32, 4);
flat_io!(w_f64, r_f64, f64, 8);
flat_io!(w_i64, r_i64, i64, 8);

impl Graph {
    /// `source_hash` identifies the extract this graph was built from.
    pub fn write_to<W: Write>(&self, out: &mut W, source_hash: u64) -> io::Result<()> {
        out.write_all(MAGIC)?;
        w_u32(out, &[FORMAT_VERSION])?;
        out.write_all(&source_hash.to_le_bytes())?;
        w_f64(out, &[self.max_speed_m_per_ms])?;
        w_u32(
            out,
            &[
                self.n_nodes() as u32,
                self.n_edges() as u32,
                self.geom.len() as u32,
                self.names.len() as u32,
            ],
        )?;
        w_f64(out, &self.lon)?;
        w_f64(out, &self.lat)?;
        w_i64(out, &self.osm_id)?;
        out.write_all(&self.node_flags)?;
        w_u32(out, &self.offsets)?;
        w_u32(out, &self.head)?;
        w_u32(out, &self.weight)?;
        w_f32(out, &self.length)?;
        w_u32(out, &self.geom_start)?;
        w_u32(out, &self.geom_len)?;
        out.write_all(&self.flags)?;
        w_u32(out, &self.name_id)?;
        w_u32(out, &self.r_offsets)?;
        w_u32(out, &self.r_head)?;
        w_u32(out, &self.r_edge)?;
        w_u32(out, &self.twin)?;
        let flat: Vec<f32> = self.geom.iter().flat_map(|p| [p[0], p[1]]).collect();
        w_f32(out, &flat)?;
        for s in &self.names {
            w_u32(out, &[s.len() as u32])?;
            out.write_all(s.as_bytes())?;
        }
        Ok(())
    }

    pub fn read_from<R: Read>(inp: &mut R) -> io::Result<(Graph, u64)> {
        let mut magic = [0u8; 8];
        inp.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "not a graph file",
            ));
        }
        let version = r_u32(inp, 1)?[0];
        if version != FORMAT_VERSION {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!("graph format v{version}, expected v{FORMAT_VERSION}"),
            ));
        }
        let mut hash = [0u8; 8];
        inp.read_exact(&mut hash)?;
        let source_hash = u64::from_le_bytes(hash);
        let max_speed_m_per_ms = r_f64(inp, 1)?[0];
        let counts = r_u32(inp, 4)?;
        let (n, m, ng, nn) = (
            counts[0] as usize,
            counts[1] as usize,
            counts[2] as usize,
            counts[3] as usize,
        );

        let lon = r_f64(inp, n)?;
        let lat = r_f64(inp, n)?;
        let osm_id = r_i64(inp, n)?;
        let mut node_flags = vec![0u8; n];
        inp.read_exact(&mut node_flags)?;
        let offsets = r_u32(inp, n + 1)?;
        let head = r_u32(inp, m)?;
        let weight = r_u32(inp, m)?;
        let length = r_f32(inp, m)?;
        let geom_start = r_u32(inp, m)?;
        let geom_len = r_u32(inp, m)?;
        let mut flags = vec![0u8; m];
        inp.read_exact(&mut flags)?;
        let name_id = r_u32(inp, m)?;
        let r_offsets = r_u32(inp, n + 1)?;
        let r_head = r_u32(inp, m)?;
        let r_edge = r_u32(inp, m)?;
        let twin = r_u32(inp, m)?;
        let flat = r_f32(inp, ng * 2)?;
        let geom = flat.as_chunks::<2>().0.to_vec();
        let mut names = Vec::with_capacity(nn);
        for _ in 0..nn {
            let len = r_u32(inp, 1)?[0] as usize;
            let mut b = vec![0u8; len];
            inp.read_exact(&mut b)?;
            names.push(
                String::from_utf8(b).map_err(|e| io::Error::new(io::ErrorKind::InvalidData, e))?,
            );
        }

        // The header value is what A* divides by. If it no longer bounds every
        // edge the heuristic is inadmissible and routes go quietly non-optimal.
        if let Some(e) = (0..m).find(|e| length[*e] as f64 / weight[*e] as f64 > max_speed_m_per_ms)
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                format!(
                    "edge {e} runs at {:.3} m/ms, above the header maximum {max_speed_m_per_ms:.3}",
                    length[e] as f64 / weight[e] as f64
                ),
            ));
        }

        Ok((
            Graph {
                lon,
                lat,
                osm_id,
                node_flags,
                twin,
                offsets,
                head,
                weight,
                length,
                geom_start,
                geom_len,
                flags,
                name_id,
                r_offsets,
                r_head,
                r_edge,
                geom,
                names,
                max_speed_m_per_ms,
            },
            source_hash,
        ))
    }

    pub fn save(&self, path: &Path, source_hash: u64) -> io::Result<()> {
        let mut w = BufWriter::new(File::create(path)?);
        self.write_to(&mut w, source_hash)?;
        w.flush()
    }

    pub fn load(path: &Path) -> io::Result<(Graph, u64)> {
        Graph::read_from(&mut BufReader::new(File::open(path)?))
    }

    pub fn to_bytes(&self, source_hash: u64) -> Vec<u8> {
        let mut v = Vec::new();
        self.write_to(&mut v, source_hash).expect("in-memory write");
        v
    }
}

/// FNV-1a over a file, so the graph header can name the extract it came from.
pub fn file_hash(path: &Path) -> io::Result<u64> {
    let mut f = BufReader::new(File::open(path)?);
    let mut buf = vec![0u8; 1 << 20];
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    loop {
        let n = f.read(&mut buf)?;
        if n == 0 {
            return Ok(h);
        }
        for b in &buf[..n] {
            h = (h ^ *b as u64).wrapping_mul(0x1000_0000_01b3);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn haversine_known_distance() {
        // Sector 17 Plaza to Sukhna Lake, about 3 km apart.
        let d = haversine((76.7794, 30.7410), (76.8106, 30.7423));
        assert!((d - 2990.0).abs() < 60.0, "{d}");
        assert_eq!(haversine((76.7, 30.7), (76.7, 30.7)), 0.0);
    }

    #[test]
    fn scc_splits_a_dangling_tail() {
        // 0<->1<->2 form a cycle back through 0; 3 is reachable but never returns.
        let edges: [(u32, u32); 5] = [(0, 1), (1, 0), (1, 2), (2, 1), (1, 3)];
        let n = 4;
        let mut e = edges.to_vec();
        e.sort();
        let offsets = csr_offsets(n, e.iter().map(|(s, _)| *s));
        let head: Vec<u32> = e.iter().map(|(_, d)| *d).collect();
        let (comp, sizes) = scc(n, &offsets, &head);
        assert_eq!(sizes.len(), 2);
        assert_eq!(comp[0], comp[1]);
        assert_eq!(comp[1], comp[2]);
        assert_ne!(comp[3], comp[0]);
        assert_eq!(*sizes.iter().max().unwrap(), 3);
    }

    fn seg(src: u32, dst: u32, geom: Vec<[f32; 2]>) -> RawEdge {
        RawEdge {
            src,
            dst,
            weight: 1200,
            length: 10.0,
            flags: 6,
            name_id: NO_NAME,
            geom,
        }
    }

    fn tiny_graph() -> Graph {
        let a = [76.7f32, 30.7];
        let b = [76.8f32, 30.8];
        assemble(
            vec![76.7, 76.8],
            vec![30.7, 30.8],
            vec![111, 222],
            vec![0, 0],
            vec![seg(0, 1, vec![a, b]), seg(1, 0, vec![b, a])],
            vec!["Madhya Marg".to_string()],
        )
    }

    #[test]
    fn csr_and_reverse_agree() {
        let g = tiny_graph();
        assert_eq!(g.n_nodes(), 2);
        assert_eq!(g.n_edges(), 2);
        assert_eq!(g.offsets, vec![0, 1, 2]);
        // the one edge arriving at node 1 comes from node 0
        let inc: Vec<u32> = g.in_edges(1).map(|j| g.r_head[j]).collect();
        assert_eq!(inc, vec![0]);
        assert_eq!(g.geometry(0).len(), 2);
        // one road, one span: the reverse direction reads it backwards
        assert_eq!(g.geom.len(), 2);
        assert_eq!(g.flags[1] & FLAG_GEOM_REVERSED, FLAG_GEOM_REVERSED);
        let f: Vec<[f32; 2]> = g.geometry(0).collect();
        let mut b: Vec<[f32; 2]> = g.geometry(1).collect();
        b.reverse();
        assert_eq!(f, b);
        assert_eq!(g.name(0), None);
        assert_eq!(g.max_speed_m_per_ms, 10.0 / 1200.0);
        // the two directions found each other
        assert_eq!(g.twin, vec![1, 0]);
        assert_eq!(g.road_length_m(), 10.0);
    }

    #[test]
    fn round_trip_is_byte_identical() {
        let g = tiny_graph();
        let bytes = g.to_bytes(0xdead_beef);
        let (back, hash) = Graph::read_from(&mut &bytes[..]).unwrap();
        assert_eq!(hash, 0xdead_beef);
        assert_eq!(back.to_bytes(hash), bytes);
        assert_eq!(back.lon, g.lon);
        assert_eq!(back.names, g.names);
        assert_eq!(back.twin, g.twin);
        assert_eq!(back.node_flags, g.node_flags);
    }

    /// 0 - 1 - 2 - 3 - 4, two-way, 10 m and 1200 ms per segment.
    fn chain_graph(node_flags: Vec<u8>) -> Graph {
        let lon: Vec<f64> = (0..5).map(|i| 76.70 + i as f64 * 0.001).collect();
        let lat = vec![30.70; 5];
        let pt = |i: usize| [lon[i] as f32, lat[i] as f32];
        let mut edges = Vec::new();
        for i in 0..4u32 {
            let (a, b) = (pt(i as usize), pt(i as usize + 1));
            edges.push(seg(i, i + 1, vec![a, b]));
            edges.push(seg(i + 1, i, vec![b, a]));
        }
        assemble(
            lon,
            lat,
            (0..5).map(|i| 1000 + i as i64).collect(),
            node_flags,
            edges,
            Vec::new(),
        )
    }

    #[test]
    fn chain_contracts_to_one_edge_each_way() {
        let g = chain_graph(vec![0; 5]);
        assert_eq!((g.n_nodes(), g.n_edges(), g.geom.len()), (5, 8, 8));
        let before = g.road_length_m();

        let (c, s) = contract(g);
        // Only the two tips survive; 1, 2 and 3 are two-way chain interiors.
        assert_eq!(c.n_nodes(), 2);
        assert_eq!(c.n_edges(), 2);
        assert_eq!(s.splices, 6); // three joints, both directions
                                  // Length and duration are summed, not recomputed.
        assert_eq!(c.weight, vec![4800, 4800]);
        assert_eq!(c.length, vec![40.0, 40.0]);
        assert!((c.road_length_m() - before).abs() < 1e-3);
        // One span for the road, read forwards and backwards.
        assert_eq!(c.geom.len(), 5);
        assert_eq!(c.geometry(0).len(), 5);

        // Geometry runs the way you travel it, and the reverse is the mirror.
        let fwd: Vec<[f32; 2]> = c.geometry(0).collect();
        let mut bwd: Vec<[f32; 2]> = c.geometry(1).collect();
        bwd.reverse();
        assert_eq!(fwd, bwd);
        assert_eq!(fwd[0], [c.lon[0] as f32, c.lat[0] as f32]);
        assert_eq!(fwd[4], [c.lon[1] as f32, c.lat[1] as f32]);
        assert_eq!(c.twin, vec![1, 0]);
        // The surviving nodes are the original tips.
        assert_eq!(c.osm_id, vec![1000, 1004]);
    }

    #[test]
    fn a_tagged_node_survives_contraction() {
        // Node 2 carries a barrier: it is a real routing feature, keep it.
        let mut flags = vec![0u8; 5];
        flags[2] = NODE_BARRIER;
        let (c, s) = contract(chain_graph(flags));
        assert_eq!(c.n_nodes(), 3);
        assert_eq!(c.osm_id, vec![1000, 1002, 1004]);
        assert_eq!(s.tagged_kept, 1);
        assert_eq!(c.weight, vec![2400, 2400, 2400, 2400]);
    }

    #[test]
    fn a_degree_two_ring_keeps_one_node() {
        // 0 - 1 - 2 - 0, every node two-way degree 2. Contracting all three
        // would collapse the ring to a self-loop.
        let lon = vec![76.70, 76.71, 76.705];
        let lat = vec![30.70, 30.70, 30.71];
        let pt = |i: usize| [lon[i] as f32, lat[i] as f32];
        let mut edges = Vec::new();
        for i in 0..3u32 {
            let j = (i + 1) % 3;
            let (a, b) = (pt(i as usize), pt(j as usize));
            edges.push(seg(i, j, vec![a, b]));
            edges.push(seg(j, i, vec![b, a]));
        }
        let g = assemble(lon, lat, vec![1, 2, 3], vec![0; 3], edges, Vec::new());
        let (c, s) = contract(g);
        assert_eq!(s.rings_kept, 1);
        assert_eq!(c.n_nodes(), 1);
        // One node left, so the ring is a self-loop pair rather than nothing.
        assert_eq!(c.n_edges(), 2);
        assert_eq!(c.head, vec![0, 0]);
        assert!((c.road_length_m() - 30.0).abs() < 1e-3);
    }

    #[test]
    fn an_asymmetric_node_is_not_a_chain() {
        // u -> v, v -> w, w -> v. No v -> u, so v encodes a real one-way turn.
        let p = [76.70f32, 30.70];
        let edges = vec![
            seg(0, 1, vec![p, p]),
            seg(1, 2, vec![p, p]),
            seg(2, 1, vec![p, p]),
        ];
        let g = assemble(
            vec![76.70, 76.71, 76.72],
            vec![30.70, 30.70, 30.70],
            vec![1, 2, 3],
            vec![0; 3],
            edges,
            Vec::new(),
        );
        assert_eq!(chain_at(&g, 1), None);
        assert_eq!(shape_at(&g, 1), Shape::Junction);
    }
}
