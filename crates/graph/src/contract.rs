//! Splice out topologically degree-2 nodes left behind by the refcount rule.

use crate::construct::{assemble, node_point, RawEdge};
use crate::topology::{chain_at, Chain};
use crate::Graph;

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
                    assert_eq!(cur_edge, bwd.0, "chain walk entered {cur} on a stray edge");
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
            assert_ne!(
                remap[cur as usize],
                u32::MAX,
                "chain walk ended on a node that was itself contracted"
            );
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
