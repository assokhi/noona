//! `graph build` - turn the clipped extract into `data/build/graph.bin`
//! and print the Phase 1 acceptance numbers.

use std::path::PathBuf;
use std::time::Instant;

const USAGE: &str = "usage: graph build [--pbf <in.osm.pbf>] [--out <graph.bin>]";

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut pbf = PathBuf::from("data/raw/chandigarh.osm.pbf");
    let mut out = PathBuf::from("data/build/graph.bin");
    let mut rest = args.iter();
    let Some(cmd) = rest.next() else {
        eprintln!("{USAGE}");
        std::process::exit(2);
    };
    while let Some(flag) = rest.next() {
        let value = rest.next().ok_or_else(|| format!("{flag} needs a value"))?;
        match flag.as_str() {
            "--pbf" => pbf = PathBuf::from(value),
            "--out" => out = PathBuf::from(value),
            other => return Err(format!("unknown flag {other}\n{USAGE}").into()),
        }
    }
    if cmd != "build" {
        eprintln!("{USAGE}");
        std::process::exit(2);
    }

    let t0 = Instant::now();
    let hash = graph::file_hash(&pbf)?;
    let (g, s) = graph::build(&pbf)?;
    let build_ms = t0.elapsed().as_millis();

    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let t1 = Instant::now();
    g.save(&out, hash)?;
    let save_ms = t1.elapsed().as_millis();

    // --- acceptance gate ---------------------------------------------------
    assert!(s.nodes_kept > 0 && s.edges > 0, "empty graph");
    for v in 0..g.n_nodes() as u32 {
        assert!(!g.out_edges(v).is_empty(), "node {v} has out-degree 0");
        assert!(!g.in_edges(v).is_empty(), "node {v} has in-degree 0");
    }
    assert_eq!(*g.offsets.last().unwrap() as usize, g.n_edges());
    assert_eq!(*g.geom_off.last().unwrap() as usize, g.geom.len());
    let t2 = Instant::now();
    let (back, back_hash) = graph::Graph::load(&out)?;
    let load_ms = t2.elapsed().as_millis();
    assert_eq!(
        back_hash, hash,
        "source hash did not survive the round trip"
    );
    assert_eq!(
        back.to_bytes(back_hash),
        std::fs::read(&out)?,
        "graph did not reload byte-identically"
    );

    // --- report ------------------------------------------------------------
    let p = &s.parse;
    println!("source            {} (fnv {:#018x})", pbf.display(), hash);
    println!("ways seen         {}", p.ways_seen);
    println!(
        "ways kept         {} ({:.1}%)  rejected: {} class, {} access, {} area",
        p.ways_kept,
        100.0 * p.ways_kept as f64 / p.ways_seen.max(1) as f64,
        p.rejected_class,
        p.rejected_access,
        p.rejected_area
    );
    println!(
        "maxspeed          {} tagged, {} parsed, {} unparseable",
        p.maxspeed_present,
        p.maxspeed_parsed,
        p.unparsed_total()
    );
    for (v, c) in p.top_unparsed(5) {
        println!("                    {c:>5} x {v:?}");
    }
    println!(
        "roundabouts       {} implied oneway",
        p.roundabout_implied_oneway
    );
    println!();
    println!("referenced nodes  {}", s.referenced_nodes);
    println!(
        "intersections     {} ({:.2}x compression from degree-2 contraction)",
        s.intersection_nodes,
        s.contraction_ratio()
    );
    println!("directed edges    {} before SCC", s.edges_before_scc);
    println!(
        "largest SCC       {} of {} components, {:.2}% of nodes",
        s.nodes_kept,
        s.scc_count,
        100.0 * s.scc_node_fraction
    );
    println!("nodes / edges     {} / {}", g.n_nodes(), g.n_edges());
    println!("geometry points   {}", s.geometry_points);
    println!("road length       {:.1} km", s.road_length_km);
    println!("missing coords    {}", s.missing_coords);
    println!(
        "max edge speed    {:.1} km/h",
        g.max_speed_m_per_ms() * 3600.0
    );
    println!();
    println!(
        "build {build_ms} ms, save {save_ms} ms, load {load_ms} ms, {} bytes -> {}",
        std::fs::metadata(&out)?.len(),
        out.display()
    );
    if s.missing_coords > 0 {
        eprintln!(
            "warning: {} way node refs had no coordinates - was the extract clipped without --complete-ways?",
            s.missing_coords
        );
    }
    Ok(())
}
