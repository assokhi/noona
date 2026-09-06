//! `graph build` - turn the clipped extract into `data/build/graph.bin` and
//! print the Phase 1 acceptance numbers.
//! `graph stats` - degree histogram and contractibility diagnosis.

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use graph::{Graph, Shape};

const USAGE: &str = "usage: graph build [--pbf <in.osm.pbf>] [--out <graph.bin>]\n       graph stats [--graph <graph.bin>]";

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
            "--out" | "--graph" => out = PathBuf::from(value),
            other => return Err(format!("unknown flag {other}\n{USAGE}").into()),
        }
    }
    match cmd.as_str() {
        "build" => build(&pbf, &out),
        "stats" => stats(&out),
        _ => {
            eprintln!("{USAGE}");
            std::process::exit(2);
        }
    }
}

fn build(pbf: &Path, out: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let t0 = Instant::now();
    let hash = graph::file_hash(pbf)?;
    let (g, s) = graph::build(pbf)?;
    let build_ms = t0.elapsed().as_millis();

    if let Some(parent) = out.parent() {
        std::fs::create_dir_all(parent)?;
    }
    let t1 = Instant::now();
    g.save(out, hash)?;
    let save_ms = t1.elapsed().as_millis();

    // --- acceptance gate ---------------------------------------------------
    assert!(s.nodes_kept > 0 && s.edges > 0, "empty graph");
    for v in 0..g.n_nodes() as u32 {
        assert!(!g.out_edges(v).is_empty(), "node {v} has out-degree 0");
        assert!(!g.in_edges(v).is_empty(), "node {v} has in-degree 0");
    }
    assert_eq!(*g.offsets.last().unwrap() as usize, g.n_edges());
    assert_eq!(*g.geom_off.last().unwrap() as usize, g.geom.len());

    // --- contraction gate --------------------------------------------------
    let c = s.contract;
    assert!(
        (c.road_m_after - c.road_m_before).abs() < 1.0,
        "contraction moved total road length by {:.3} m",
        c.road_m_after - c.road_m_before
    );
    // Each splice removes exactly one duplicated joint point and no distinct
    // vertex, so the arithmetic is exact rather than "roughly unchanged".
    assert_eq!(
        c.geom_after,
        c.geom_before - c.splices,
        "geometry points did not just move between edges"
    );
    // Connectivity is not a function of contraction: it was one SCC before.
    let (_, sizes) = graph::scc(g.n_nodes(), &g.offsets, &g.head);
    assert_eq!(
        (sizes.len(), sizes[0] as usize),
        (1, g.n_nodes()),
        "contraction broke strong connectivity"
    );
    let t2 = Instant::now();
    let (back, back_hash) = Graph::load(out)?;
    let load_ms = t2.elapsed().as_millis();
    assert_eq!(
        back_hash, hash,
        "source hash did not survive the round trip"
    );
    assert_eq!(
        back.to_bytes(back_hash),
        std::fs::read(out)?,
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
    println!();
    println!(
        "contraction       {} -> {} nodes ({} spliced), {} -> {} edges",
        c.nodes_before,
        c.nodes_after,
        c.nodes_before - c.nodes_after,
        c.edges_before,
        c.edges_after
    );
    println!(
        "                  mean out-degree {:.3} -> {:.3}, {} joints spliced",
        c.edges_before as f64 / c.nodes_before as f64,
        c.edges_after as f64 / c.nodes_after as f64,
        c.splices
    );
    println!(
        "                  geometry {} -> {} points, road {:.3} -> {:.3} km",
        c.geom_before,
        c.geom_after,
        c.road_m_before / 1000.0,
        c.road_m_after / 1000.0
    );
    println!(
        "                  kept back: {} tagged, {} mixed-twin, {} ring nodes",
        c.tagged_kept, c.mixed_kept, c.rings_kept
    );
    println!();
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
        std::fs::metadata(out)?.len(),
        out.display()
    );
    if s.missing_coords > 0 {
        eprintln!(
            "warning: {} way node refs had no coordinates - was the extract clipped without complete_ways?",
            s.missing_coords
        );
    }
    Ok(())
}

/// Is the refcount>=2 junction rule leaving degree-2 nodes behind? OSM splits
/// ways at every tagging change, so one continuous road arrives as several ways
/// end to end and each split node hits refcount 2 while being topologically
/// degree 2.
fn stats(path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let (g, hash) = Graph::load(path)?;
    let n = g.n_nodes();
    println!(
        "graph             {} (source fnv {:#018x})",
        path.display(),
        hash
    );
    println!(
        "nodes / edges     {} / {}   mean out-degree {:.3}",
        n,
        g.n_edges(),
        g.n_edges() as f64 / n as f64
    );
    println!();

    let mut hist: HashMap<(usize, usize), u32> = HashMap::new();
    let mut shapes: HashMap<&str, u32> = HashMap::new();
    for v in 0..n as u32 {
        let key = (g.in_edges(v).len(), g.out_edges(v).len());
        *hist.entry(key).or_insert(0) += 1;
        let name = match graph::shape_at(&g, v) {
            Shape::OneWayChain => "one-way chain interior",
            Shape::TwoWayChain => "two-way chain interior",
            Shape::CulDeSac => "cul-de-sac tip",
            Shape::Stub => "stub (no way in or out)",
            Shape::Junction => "junction",
        };
        *shapes.entry(name).or_insert(0) += 1;
    }

    let mut rows: Vec<((usize, usize), u32)> = hist.into_iter().collect();
    rows.sort_by_key(|(deg, count)| (std::cmp::Reverse(*count), *deg));
    println!(
        "(in, out) degree histogram, top 20 of {} distinct",
        rows.len()
    );
    for ((i, o), c) in rows.iter().take(20) {
        println!(
            "  ({i:>2},{o:>2})  {c:>7}  {:>5.2}%",
            100.0 * *c as f64 / n as f64
        );
    }
    println!();

    let mut sh: Vec<(&str, u32)> = shapes.into_iter().collect();
    sh.sort_by_key(|(_, count)| std::cmp::Reverse(*count));
    println!("topology");
    for (name, c) in &sh {
        println!(
            "  {name:<24} {c:>7}  {:>5.2}%",
            100.0 * *c as f64 / n as f64
        );
    }

    let chains: u32 = sh
        .iter()
        .filter(|(k, _)| k.ends_with("chain interior"))
        .map(|(_, c)| *c)
        .sum();
    // The pass declines a chain node that carries a tag, or whose two segments
    // disagree about being two-way, so those are not evidence of unfinished work.
    let ready = (0..n as u32)
        .filter(|v| graph::contractible_at(&g, *v))
        .count();
    println!();
    println!(
        "chain interiors   {chains} ({:.2}%), {} held back by a tag or mixed twins",
        100.0 * chains as f64 / n as f64,
        chains as usize - ready
    );
    println!(
        "still contractible {ready} ({:.2}%) - a contracted graph should be near zero,",
        100.0 * ready as f64 / n as f64
    );
    println!("                  the remainder being degree-2 rings that keep one node");
    Ok(())
}
