//! OSM ways -> CSR graph: junction detection, edge emission, largest SCC,
//! and packing an edge list into the arrays.

use crate::topology::scc;
use crate::{haversine, Graph, FLAG_GEOM_REVERSED, NODE_BARRIER, NODE_SIGNALS, NO_NAME, NO_TWIN};
use osm_parse::{Oneway, Way};
use std::collections::{HashMap, HashSet};
use std::path::Path;

/// One directed edge as emitted from a way, still keyed by OSM node id.
pub(crate) struct PendingEdge {
    a: i64,
    b: i64,
    weight: u32,
    length: f32,
    flags: u8,
    name_id: u32,
    geom: Vec<[f32; 2]>,
}

/// One directed edge before it is packed into CSR.
pub(crate) struct RawEdge {
    pub(crate) src: u32,
    pub(crate) dst: u32,
    pub(crate) weight: u32,
    pub(crate) length: f32,
    pub(crate) flags: u8,
    pub(crate) name_id: u32,
    pub(crate) geom: Vec<[f32; 2]>,
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

impl Graph {
    /// Build a graph straight from coordinates and `(src, dst, weight_ms)`
    /// triples, with straight-line geometry. For tests and small examples;
    /// the real thing comes from `build`.
    pub fn from_edges(coords: &[(f64, f64)], edges: &[(u32, u32, u32)]) -> Graph {
        let point = |v: u32| {
            let c = coords[v as usize];
            [c.0 as f32, c.1 as f32]
        };
        let raw = edges
            .iter()
            .map(|(s, d, w)| RawEdge {
                src: *s,
                dst: *d,
                weight: *w,
                length: haversine(coords[*s as usize], coords[*d as usize]) as f32,
                flags: 0,
                name_id: NO_NAME,
                geom: vec![point(*s), point(*d)],
            })
            .collect();
        assemble(
            coords.iter().map(|c| c.0).collect(),
            coords.iter().map(|c| c.1).collect(),
            (0..coords.len() as i64).collect(),
            vec![0; coords.len()],
            raw,
            Vec::new(),
        )
    }

pub(crate) fn node_point(g: &Graph, v: u32) -> [f32; 2] {
    [g.lon[v as usize] as f32, g.lat[v as usize] as f32]
}

/// Identity of a directed edge as a physical piece of road.
pub(crate) type EdgeShape = (u32, u32, Vec<(u32, u32)>);

pub(crate) fn shape_key<'a>(src: u32, dst: u32, points: impl Iterator<Item = &'a [f32; 2]>) -> EdgeShape {
    (
        src,
        dst,
        points.map(|p| (p[0].to_bits(), p[1].to_bits())).collect(),
    )
}

/// CSR offsets from an iterator of source ids. Length `n + 1`.
pub(crate) fn csr_offsets(n: usize, srcs: impl Iterator<Item = u32>) -> Vec<u32> {
    let mut off = vec![0u32; n + 1];
    for s in srcs {
        off[s as usize + 1] += 1;
    }
    for i in 1..=n {
        off[i] += off[i - 1];
    }
    off
}

pub(crate) fn assemble(
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
    // One pass over the edge list during a build that already takes 650 ms.
    assert!(
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
