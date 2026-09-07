//! Four hand-picked routes, dumped as GeoJSON and committed as golden files.
//!
//! The 1000-pair harness proves the three algorithms agree with each other. It
//! cannot tell you whether a table of invented speed defaults produces sensible
//! routes when `maxspeed` coverage is 0.4%. That needs eyes on a map, and a
//! golden file so a later phase changing these routes is a signal rather than a
//! surprise.
//!
//! Regenerate after a deliberate change:  UPDATE_GOLDEN=1 cargo test -p routing

use std::path::{Path, PathBuf};

use graph::Graph;
use routing::Search;

/// (name, from lon/lat, to lon/lat, what it is meant to exercise)
const ROUTES: [(&str, (f64, f64), (f64, f64), &str); 4] = [
    (
        "sector17-plaza-to-pgimer",
        (76.7794, 30.7410),
        (76.7648, 30.7649),
        "city centre to hospital, should follow the V3 arterials",
    ),
    (
        "sukhna-lake-to-airport",
        (76.8106, 30.7423),
        (76.7885, 30.6735),
        "long north-south crossing, exercises the fastest roads",
    ),
    (
        "sector17-to-mohali-phase7",
        (76.7794, 30.7410),
        (76.7150, 30.7050),
        "crosses into Mohali, leaning on the coarse-cut margin and back-filled nodes",
    ),
    (
        "inside-sector-40",
        (76.7480, 30.7180),
        (76.7560, 30.7245),
        "short hop inside one sector, exercises V5/V6 residential streets",
    ),
];

/// Nearest node by brute force.
///
/// ponytail: O(n) scan, fine for four fixed points in a test. Phase 3 builds a
/// real spatial index over *edges* and snaps to the perpendicular projection,
/// which is a different and better answer - do not promote this.
fn nearest_node(g: &Graph, at: (f64, f64)) -> u32 {
    (0..g.n_nodes() as u32)
        .min_by(|a, b| {
            graph::haversine(g.coord(*a), at).total_cmp(&graph::haversine(g.coord(*b), at))
        })
        .expect("graph has no nodes")
}

/// Share of the route spent on each road class, biggest first. This is the half
/// of the eyeball check that can be automated: a short hop that turns out to be
/// mostly `primary`, or any route with `service` in it, points at the speed
/// table rather than at the search.
fn class_mix(g: &Graph, r: &routing::Route) -> String {
    let mut by_class: Vec<(&str, f64)> = Vec::new();
    for e in &r.edges {
        let c = g.edge_class(*e as usize);
        let m = g.length[*e as usize] as f64;
        match by_class.iter_mut().find(|(k, _)| *k == c) {
            Some((_, acc)) => *acc += m,
            None => by_class.push((c, m)),
        }
    }
    by_class.sort_by(|a, b| b.1.total_cmp(&a.1));
    let total: f64 = by_class.iter().map(|(_, m)| m).sum();
    let parts: Vec<String> = by_class
        .iter()
        .map(|(c, m)| format!("{:?}: {:.0}", c, 100.0 * m / total))
        .collect();
    parts.join(", ")
}

/// Named streets in travel order, consecutive duplicates collapsed.
fn streets(g: &Graph, r: &routing::Route) -> String {
    let mut out: Vec<&str> = Vec::new();
    for e in &r.edges {
        if let Some(n) = g.name(*e as usize) {
            if out.last() != Some(&n) {
                out.push(n);
            }
        }
    }
    let quoted: Vec<String> = out.iter().map(|n| format!("{:?}", n)).collect();
    quoted.join(", ")
}

fn geojson(g: &Graph, name: &str, note: &str, r: &routing::Route) -> String {
    let coords: Vec<String> = r
        .geometry(g)
        .iter()
        .map(|p| format!("[{:.6}, {:.6}]", p[0], p[1]))
        .collect();
    let mut s = String::new();
    s.push_str("{\n  \"type\": \"Feature\",\n  \"properties\": {\n");
    s.push_str(&format!("    \"name\": {name:?},\n"));
    s.push_str(&format!("    \"note\": {note:?},\n"));
    s.push_str(&format!("    \"distance_m\": {:.1},\n", r.distance_m));
    s.push_str(&format!("    \"duration_s\": {:.1},\n", r.duration_s()));
    s.push_str(&format!("    \"cost_ms\": {},\n", r.cost_ms));
    s.push_str(&format!("    \"edges\": {},\n", r.edges.len()));
    s.push_str(&format!(
        "    \"class_mix_pct\": {{{}}},\n",
        class_mix(g, r)
    ));
    s.push_str(&format!("    \"streets\": [{}]\n", streets(g, r)));
    s.push_str("  },\n  \"geometry\": {\n    \"type\": \"LineString\",\n");
    s.push_str(&format!(
        "    \"coordinates\": [\n      {}\n    ]\n",
        coords.join(",\n      ")
    ));
    s.push_str("  }\n}\n");
    s
}

fn repo_root() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("..")
}

#[test]
fn four_routes_match_their_golden_files() {
    let graph_path = repo_root().join("data").join("build").join("graph.bin");
    if !graph_path.exists() {
        // data/build is gitignored, so CI has no graph. The committed fixture
        // is a few km of Sector 17 and contains neither PGIMER nor the airport,
        // so there is nothing honest to check against there.
        eprintln!(
            "skipping: {} not built, run `make graph`",
            graph_path.display()
        );
        return;
    }
    let (g, _) = Graph::load(&graph_path).expect("load graph");
    let mut search = Search::new(g.n_nodes());
    let update = std::env::var_os("UPDATE_GOLDEN").is_some();
    let dir = repo_root().join("tests").join("golden");
    std::fs::create_dir_all(&dir).expect("golden dir");

    for (name, from, to, note) in ROUTES {
        let (s, t) = (nearest_node(&g, from), nearest_node(&g, to));
        assert_ne!(s, t, "{name}: endpoints snapped to the same node");

        let d = search
            .dijkstra(&g, s, t)
            .unwrap_or_else(|| panic!("{name}: no route"));
        let a = search.astar(&g, s, t).expect("astar");
        let b = search.bidirectional(&g, s, t).expect("bidirectional");
        assert_eq!(d.cost_ms, a.cost_ms, "{name}: astar disagrees");
        assert_eq!(d.cost_ms, b.cost_ms, "{name}: bidirectional disagrees");

        // Sanity that the route is a real drive, not a teleport.
        let straight = graph::haversine(g.coord(s), g.coord(t));
        assert!(
            d.distance_m >= straight * 0.99,
            "{name}: {:.0} m route is shorter than the {straight:.0} m straight line",
            d.distance_m
        );
        assert!(
            d.distance_m < straight * 3.0,
            "{name}: {:.0} m route detours more than 3x the {straight:.0} m straight line",
            d.distance_m
        );

        // Where the slow classes sit matters more than how much of them there
        // is: service road at the very end is a campus or a driveway, service
        // road in the middle is the search cutting through a parking lot.
        let mut travelled = 0.0f64;
        let mut runs: Vec<(String, f64, f64)> = Vec::new();
        for e in &d.edges {
            let c = g.edge_class(*e as usize).to_string();
            let m = g.length[*e as usize] as f64;
            match runs.last_mut() {
                Some((k, _, end)) if *k == c => *end = travelled + m,
                _ => runs.push((c, travelled, travelled + m)),
            }
            travelled += m;
        }
        let slow: Vec<String> = runs
            .iter()
            .filter(|(c, a, b)| (c == "service" || c == "living_street") && b - a > 50.0)
            .map(|(c, a, b)| {
                format!(
                    "{c} {:.0}-{:.0}% of the way ({:.0} m)",
                    100.0 * a / travelled,
                    100.0 * b / travelled,
                    b - a
                )
            })
            .collect();
        eprintln!(
            "{name}: {:.2} km, slow-class runs: {}",
            d.distance_m / 1000.0,
            if slow.is_empty() {
                "none over 50 m".to_string()
            } else {
                slow.join("; ")
            }
        );

        let doc = geojson(&g, name, note, &d);
        let path = dir.join(format!("{name}.geojson"));
        if update || !path.exists() {
            std::fs::write(&path, &doc).expect("write golden");
            eprintln!("wrote {}", path.display());
            continue;
        }
        let want = std::fs::read_to_string(&path).expect("read golden");
        assert_eq!(
            want.replace("\r\n", "\n"),
            doc.replace("\r\n", "\n"),
            "{name} changed. If that was deliberate: UPDATE_GOLDEN=1 cargo test -p routing"
        );
    }
}
