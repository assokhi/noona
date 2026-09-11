use crate::construct::{assemble, csr_offsets, RawEdge};
use crate::contract::contract;
use crate::topology::{chain_at, scc, shape_at, Shape};
use crate::*;


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

