//! OD pair generation and the latency + correctness harness.
//!
//! Every algorithm after Dijkstra is validated here, so this exists before
//! there is anything to validate.
//!
//!   bench gen --n 1000 --seed 42 --out data/build/od.json
//!   bench run --alg dijkstra,astar,bidir --pairs data/build/od.json --reference dijkstra

use std::collections::HashMap;
use std::path::{Path, PathBuf};
use std::time::Instant;

use graph::Graph;
use routing::Search;
use serde::{Deserialize, Serialize};

const USAGE: &str = "usage: bench gen  [--n N] [--seed S] [--graph graph.bin] [--out od.json]\n\
                     \x20      bench run  [--alg a,b,c] [--pairs od.json] [--graph graph.bin] [--reference alg] [--json out.json]\n\
                     \x20      bench snap [--n N] [--seed S] [--graph graph.bin]\n\
                     \x20      bench coord [--pairs od.json] [--graph graph.bin]\n\
                     \x20      bench landmarks [--k 16] [--graph graph.bin] [--landmarks landmarks.bin]";

/// Pairs are stored by OSM node id, not by internal index. Internal ids are
/// dense positions that move whenever the graph is rebuilt; the point of a
/// frozen pair set is that it survives that.
#[derive(Serialize, Deserialize)]
struct Pair {
    from_osm: i64,
    to_osm: i64,
    from: [f64; 2],
    to: [f64; 2],
    straight_km: f64,
}

#[derive(Serialize, Deserialize)]
struct PairSet {
    seed: u64,
    generated_from_graph: String,
    graph_nodes: usize,
    graph_edges: usize,
    min_separation_m: f64,
    pairs: Vec<Pair>,
}

/// SplitMix64. Hand-rolled on purpose: the pair set has to be reproducible for
/// the life of the project, and no crate guarantees a stable stream across
/// versions.
struct Rng(u64);

impl Rng {
    fn next_u64(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }
    fn next_f64(&mut self) -> f64 {
        // 53 bits of mantissa, uniform in [0, 1).
        (self.next_u64() >> 11) as f64 / (1u64 << 53) as f64
    }
}

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut n = 1000usize;
    let mut seed = 42u64;
    let mut graph_path = PathBuf::from("data/build/graph.bin");
    let mut out = PathBuf::from("data/build/od.json");
    let mut pairs = PathBuf::from("data/build/od.json");
    let mut algs = "dijkstra".to_string();
    let mut reference = "dijkstra".to_string();
    let mut json_out: Option<PathBuf> = None;
    let mut by_coord = false;
    let mut landmarks_path = PathBuf::from("data/build/landmarks.bin");
    let mut k = routing::alt::DEFAULT_LANDMARKS;

    let mut rest = args.iter();
    let Some(cmd) = rest.next() else {
        eprintln!("{USAGE}");
        std::process::exit(2);
    };
    while let Some(flag) = rest.next() {
        let v = rest.next().ok_or_else(|| format!("{flag} needs a value"))?;
        match flag.as_str() {
            "--n" => n = v.parse()?,
            "--seed" => seed = v.parse()?,
            "--graph" => graph_path = PathBuf::from(v),
            "--out" => out = PathBuf::from(v),
            "--pairs" => pairs = PathBuf::from(v),
            "--alg" => algs = v.clone(),
            "--reference" => reference = v.clone(),
            "--json" => json_out = Some(PathBuf::from(v)),
            "--coord" => by_coord = v == "true" || v == "1",
            "--landmarks" => landmarks_path = PathBuf::from(v),
            "--k" => k = v.parse()?,
            other => return Err(format!("unknown flag {other}\n{USAGE}").into()),
        }
    }

    match cmd.as_str() {
        "gen" => gen(&graph_path, n, seed, &out),
        "run" => run(
            &graph_path,
            &pairs,
            &algs,
            &reference,
            json_out.as_deref(),
            by_coord,
            &landmarks_path,
        ),
        "snap" => snap(&graph_path, n, seed),
        "coord" => coord_gate(&graph_path, &pairs),
        "landmarks" => landmarks(&graph_path, k, &landmarks_path),
        _ => {
            eprintln!("{USAGE}");
            std::process::exit(2);
        }
    }
}

/// Minimum great-circle separation. Below this the pair says nothing about a
/// routing algorithm, it just measures heap setup.
const MIN_SEPARATION_M: f64 = 500.0;

fn gen(
    graph_path: &Path,
    n: usize,
    seed: u64,
    out: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let (g, _) = Graph::load(graph_path)?;

    // Sample nodes with probability proportional to the road length meeting
    // there. Uniform node sampling oversamples dense sector interiors and
    // yields a benchmark dominated by 400 m trips.
    let mut cumulative = Vec::with_capacity(g.n_nodes() + 1);
    let mut total = 0.0f64;
    cumulative.push(0.0);
    for v in 0..g.n_nodes() as u32 {
        let incident: f64 = g
            .out_edges(v)
            .chain(g.in_edges(v).map(|j| g.r_edge[j] as usize))
            .map(|e| g.length[e] as f64)
            .sum();
        total += incident;
        cumulative.push(total);
    }
    let pick = |r: f64| -> u32 {
        let target = r * total;
        (cumulative.partition_point(|c| *c <= target).max(1) - 1) as u32
    };

    let mut rng = Rng(seed);
    let mut search = Search::new(g.n_nodes());
    let mut pairs = Vec::with_capacity(n);
    let (mut tries, mut too_close, mut unreachable) = (0u64, 0u64, 0u64);

    while pairs.len() < n {
        tries += 1;
        if tries > 1_000_000 {
            return Err("gave up finding usable OD pairs".into());
        }
        let (s, t) = (pick(rng.next_f64()), pick(rng.next_f64()));
        if s == t {
            continue;
        }
        let (a, b) = (g.coord(s), g.coord(t));
        let straight = graph::haversine(a, b);
        if straight < MIN_SEPARATION_M {
            too_close += 1;
            continue;
        }
        // Mutually reachable, both ways. Inside one SCC this always holds, but
        // asserting it here means a later graph change cannot quietly poison
        // the frozen pair set.
        if search.dijkstra(&g, s, t).is_none() || search.dijkstra(&g, t, s).is_none() {
            unreachable += 1;
            continue;
        }
        pairs.push(Pair {
            from_osm: g.osm_id[s as usize],
            to_osm: g.osm_id[t as usize],
            from: [a.0, a.1],
            to: [b.0, b.1],
            straight_km: straight / 1000.0,
        });
    }

    let mut km: Vec<f64> = pairs.iter().map(|p| p.straight_km).collect();
    km.sort_by(f64::total_cmp);
    let set = PairSet {
        seed,
        generated_from_graph: graph_path.display().to_string(),
        graph_nodes: g.n_nodes(),
        graph_edges: g.n_edges(),
        min_separation_m: MIN_SEPARATION_M,
        pairs,
    };
    if let Some(p) = out.parent() {
        std::fs::create_dir_all(p)?;
    }
    std::fs::write(out, serde_json::to_string_pretty(&set)?)?;
    println!(
        "{} pairs, seed {seed}, {tries} draws ({too_close} too close, {unreachable} unreachable)",
        set.pairs.len()
    );
    println!(
        "straight-line km: min {:.2}, p50 {:.2}, p95 {:.2}, max {:.2}",
        km[0],
        km[km.len() / 2],
        km[km.len() * 95 / 100],
        km[km.len() - 1]
    );
    println!("wrote {}", out.display());
    Ok(())
}

#[derive(Serialize)]
struct AlgResult {
    alg: String,
    queries: usize,
    p50_ms: f64,
    p95_ms: f64,
    p99_ms: f64,
    max_ms: f64,
    nodes_settled_mean: f64,
    nodes_settled_max: u32,
    edges_relaxed_mean: f64,
    mismatches: usize,
    /// Wall-clock ratio. Noisy: on this laptop it moves by 2x run to run.
    speedup_vs_reference: f64,
    /// Deterministic work ratio - the same to the node on every run, which is
    /// why the table reports it next to latency rather than instead of it.
    settled_ratio_vs_reference: f64,
}

#[derive(Serialize)]
struct Report {
    machine: Machine,
    graph: GraphInfo,
    pair_set: PairInfo,
    reference: String,
    results: Vec<AlgResult>,
}

#[derive(Serialize)]
struct Machine {
    cpu: String,
    ram_gb: String,
    rustc: String,
    profile: String,
}

#[derive(Serialize)]
struct GraphInfo {
    path: String,
    nodes: usize,
    edges: usize,
    max_speed_kmh: f64,
}

#[derive(Serialize)]
struct PairInfo {
    path: String,
    seed: u64,
    count: usize,
}

fn percentile(sorted: &[f64], p: f64) -> f64 {
    if sorted.is_empty() {
        return 0.0;
    }
    let i = ((sorted.len() - 1) as f64 * p).round() as usize;
    sorted[i]
}

/// Build the landmark tables ALT routes on.
fn landmarks(graph_path: &Path, k: usize, out: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let (g, hash) = Graph::load(graph_path)?;
    let t0 = Instant::now();
    let lm = routing::alt::Landmarks::build(&g, k);
    let prep_ms = t0.elapsed().as_secs_f64() * 1000.0;
    if let Some(p) = out.parent() {
        std::fs::create_dir_all(p)?;
    }
    lm.save(out, hash)?;
    let bytes = std::fs::metadata(out)?.len();
    println!(
        "{k} landmarks over {} nodes in {prep_ms:.0} ms, {bytes} bytes -> {}",
        g.n_nodes(),
        out.display()
    );
    Ok(())
}

/// Either end of the comparison: node ids, or the snapped coordinates the API
/// actually serves. `--coord` makes the in-process baseline do exactly the work
/// the HTTP handler does, so the difference between them is purely transport.
enum Ends {
    Nodes(Vec<(u32, u32)>),
    Coords(
        Vec<(graph::grid::Snap, graph::grid::Snap)>,
        graph::grid::Metric,
    ),
}

fn route(
    search: &mut Search,
    g: &Graph,
    lm: Option<&routing::alt::Landmarks>,
    ends: &Ends,
    i: usize,
    alg: &str,
) -> Option<(u32, routing::SearchStats)> {
    let need_lm = || lm.expect("alt needs landmarks: run `bench landmarks` first");
    match ends {
        Ends::Nodes(v) => {
            let (s, t) = v[i];
            let r = match alg {
                "dijkstra" => search.dijkstra(g, s, t),
                "astar" => search.astar(g, s, t),
                "bidir" => search.bidirectional(g, s, t),
                "alt" => search.alt(g, need_lm(), s, t),
                other => panic!("unknown algorithm {other}"),
            };
            r.map(|r| (r.cost_ms, r.stats))
        }
        Ends::Coords(v, m) => {
            let (from, to) = v[i];
            let alg: routing::coord::Alg = alg.parse().expect("unknown algorithm");
            routing::coord::route_with(search, g, m, lm, from, to, alg)
                .map(|r| (r.cost_ms, r.stats))
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn run(
    graph_path: &Path,
    pairs_path: &Path,
    algs: &str,
    reference: &str,
    json_out: Option<&Path>,
    by_coord: bool,
    landmarks_path: &Path,
) -> Result<(), Box<dyn std::error::Error>> {
    let (g, graph_hash) = Graph::load(graph_path)?;
    let set: PairSet = serde_json::from_slice(&std::fs::read(pairs_path)?)?;

    let index: HashMap<i64, u32> = g
        .osm_id
        .iter()
        .enumerate()
        .map(|(i, id)| (*id, i as u32))
        .collect();
    let resolved: Vec<(u32, u32)> = set
        .pairs
        .iter()
        .map(|p| {
            let s = index.get(&p.from_osm);
            let t = index.get(&p.to_osm);
            match (s, t) {
                (Some(s), Some(t)) => Ok((*s, *t)),
                _ => Err(format!(
                    "OSM node {} or {} is not in {} - the pair set was generated against a different graph",
                    p.from_osm,
                    p.to_osm,
                    graph_path.display()
                )),
            }
        })
        .collect::<Result<_, _>>()?;

    let names: Vec<&str> = algs
        .split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .collect();
    let mut search = Search::new(g.n_nodes());

    // The reference costs every other algorithm must match exactly.
    // Only loaded when something asks for it, so the other algorithms do not
    // need preprocessing to exist.
    let lm = if algs.split(',').any(|a| a.trim() == "alt") || reference == "alt" {
        let (lm, lm_hash) = routing::alt::Landmarks::load(landmarks_path)?;
        if lm_hash != graph_hash {
            return Err(format!(
                "{} was built for a different graph - rerun `bench landmarks`",
                landmarks_path.display()
            )
            .into());
        }
        Some(lm)
    } else {
        None
    };

    let ends = if by_coord {
        let grid = graph::grid::Grid::build(&g);
        let m = grid.metric();
        let snaps = resolved
            .iter()
            .map(|(s, t)| {
                let a = grid.nearest(&g, g.coord(*s), 100.0).expect("origin snaps");
                let b = grid
                    .nearest(&g, g.coord(*t), 100.0)
                    .expect("destination snaps");
                (a, b)
            })
            .collect();
        Ends::Coords(snaps, m)
    } else {
        Ends::Nodes(resolved.clone())
    };

    let mut reference_cost: Vec<u32> = Vec::with_capacity(resolved.len());
    for i in 0..resolved.len() {
        let (cost, _) = route(&mut search, &g, lm.as_ref(), &ends, i, reference)
            .ok_or_else(|| format!("reference {reference} found no route for pair {i}"))?;
        reference_cost.push(cost);
    }

    // Warm up every algorithm first; these samples are thrown away.
    for alg in &names {
        for i in 0..resolved.len().min(50) {
            let _ = route(&mut search, &g, lm.as_ref(), &ends, i, alg);
        }
    }

    // Interleave the algorithms per pair, rotating which one goes first.
    // Timing them in sequence instead lets the last algorithm run on a hotter,
    // higher-clocked CPU than the first, which on this laptop was worth more
    // than the difference between the algorithms.
    let mut times: Vec<Vec<f64>> = vec![Vec::with_capacity(resolved.len()); names.len()];
    let mut settled: Vec<Vec<u32>> = vec![Vec::with_capacity(resolved.len()); names.len()];
    let mut relaxed: Vec<u64> = vec![0; names.len()];
    let mut mismatches: Vec<usize> = vec![0; names.len()];
    for (i, want) in reference_cost.iter().enumerate() {
        for k in 0..names.len() {
            let a = (i + k) % names.len();
            // One query per timing sample, single threaded, monotonic clock.
            let start = Instant::now();
            let r = route(&mut search, &g, lm.as_ref(), &ends, i, names[a]);
            times[a].push(start.elapsed().as_secs_f64() * 1000.0);
            match r {
                Some((cost_ms, stats)) => {
                    if cost_ms != *want {
                        if mismatches[a] < 5 {
                            eprintln!(
                                "MISMATCH {} pair {i}: {cost_ms} ms vs reference {want} ms",
                                names[a]
                            );
                        }
                        mismatches[a] += 1;
                    }
                    settled[a].push(stats.nodes_settled);
                    relaxed[a] += stats.edges_relaxed as u64;
                }
                None => {
                    eprintln!(
                        "MISMATCH {} pair {i}: no route, reference found one",
                        names[a]
                    );
                    mismatches[a] += 1;
                    settled[a].push(0);
                }
            }
        }
    }

    let mut results = Vec::new();
    let mut reference_p50 = 0.0f64;
    let mut reference_settled = 0.0f64;
    for (a, alg) in names.iter().enumerate() {
        times[a].sort_by(f64::total_cmp);
        let p50 = percentile(&times[a], 0.50);
        let settled_mean =
            settled[a].iter().map(|s| *s as f64).sum::<f64>() / settled[a].len() as f64;
        if *alg == reference {
            reference_p50 = p50;
            reference_settled = settled_mean;
        }
        results.push(AlgResult {
            alg: (*alg).to_string(),
            queries: resolved.len(),
            p50_ms: p50,
            p95_ms: percentile(&times[a], 0.95),
            p99_ms: percentile(&times[a], 0.99),
            max_ms: *times[a].last().unwrap_or(&0.0),
            nodes_settled_mean: settled_mean,
            nodes_settled_max: settled[a].iter().copied().max().unwrap_or(0),
            edges_relaxed_mean: relaxed[a] as f64 / resolved.len() as f64,
            mismatches: mismatches[a],
            speedup_vs_reference: 0.0,
            settled_ratio_vs_reference: 0.0,
        });
    }
    if reference_settled <= 0.0 {
        reference_settled = results.first().map(|r| r.nodes_settled_mean).unwrap_or(1.0);
    }
    for r in &mut results {
        r.settled_ratio_vs_reference = reference_settled / r.nodes_settled_mean;
    }
    if reference_p50 <= 0.0 {
        reference_p50 = results.first().map(|r| r.p50_ms).unwrap_or(1.0);
    }
    for r in &mut results {
        r.speedup_vs_reference = reference_p50 / r.p50_ms;
    }

    let report = Report {
        machine: Machine {
            cpu: cpu_model(),
            ram_gb: ram_gb(),
            rustc: rustc_version(),
            profile: "release opt-level=3, thin LTO, codegen-units=1".into(),
        },
        graph: GraphInfo {
            path: graph_path.display().to_string(),
            nodes: g.n_nodes(),
            edges: g.n_edges(),
            max_speed_kmh: g.max_speed_m_per_ms * 3600.0,
        },
        pair_set: PairInfo {
            path: pairs_path.display().to_string(),
            seed: set.seed,
            count: set.pairs.len(),
        },
        reference: reference.to_string(),
        results,
    };

    // A benchmark table without the machine on it is not comparable to anything.
    println!(
        "machine   {} | {} GB RAM | {}",
        report.machine.cpu, report.machine.ram_gb, report.machine.rustc
    );
    println!(
        "graph     {} nodes / {} edges, fastest edge {:.1} km/h",
        report.graph.nodes, report.graph.edges, report.graph.max_speed_kmh
    );
    println!(
        "pairs     {} from {} (seed {})",
        report.pair_set.count, report.pair_set.path, report.pair_set.seed
    );
    println!();
    println!("| Algorithm | p50 (ms) | p95 (ms) | p99 (ms) | max (ms) | Nodes settled (mean) | Nodes settled (max) | Edges relaxed (mean) | Latency vs {reference} | Work vs {reference} | Mismatches |");
    println!("|---|---|---|---|---|---|---|---|---|---|---|");
    for r in &report.results {
        println!(
            "| {} | {:.3} | {:.3} | {:.3} | {:.3} | {:.0} | {} | {:.0} | {:.2}x | {:.2}x | {} |",
            r.alg,
            r.p50_ms,
            r.p95_ms,
            r.p99_ms,
            r.max_ms,
            r.nodes_settled_mean,
            r.nodes_settled_max,
            r.edges_relaxed_mean,
            r.speedup_vs_reference,
            r.settled_ratio_vs_reference,
            r.mismatches
        );
    }

    if let Some(p) = json_out {
        if let Some(dir) = p.parent() {
            std::fs::create_dir_all(dir)?;
        }
        std::fs::write(p, serde_json::to_string_pretty(&report)?)?;
        println!("\nwrote {}", p.display());
    }

    let total: usize = report.results.iter().map(|r| r.mismatches).sum();
    if total > 0 {
        eprintln!("\n{total} cost mismatches against {reference} - this is a gate, not a report");
        std::process::exit(1);
    }
    println!(
        "\nall {} algorithms agree with {reference} on every pair",
        report.results.len()
    );
    Ok(())
}

/// Routing by coordinate must agree exactly with routing by node id when the
/// coordinates *are* node coordinates. Any drift is a seeding bug.
fn coord_gate(graph_path: &Path, pairs_path: &Path) -> Result<(), Box<dyn std::error::Error>> {
    let (g, _) = Graph::load(graph_path)?;
    let grid = graph::grid::Grid::build(&g);
    let m = grid.metric();
    let set: PairSet = serde_json::from_slice(&std::fs::read(pairs_path)?)?;
    let index: HashMap<i64, u32> = g
        .osm_id
        .iter()
        .enumerate()
        .map(|(i, id)| (*id, i as u32))
        .collect();

    let mut search = Search::new(g.n_nodes());
    let mut mismatches = 0usize;
    let mut same_edge = 0usize;
    let mut unsnapped = 0usize;
    for (i, p) in set.pairs.iter().enumerate() {
        let (Some(&s), Some(&t)) = (index.get(&p.from_osm), index.get(&p.to_osm)) else {
            return Err("pair set was generated against a different graph".into());
        };
        let want = search.dijkstra(&g, s, t).map(|r| r.cost_ms);
        let (Some(fs), Some(ts)) = (
            grid.nearest(&g, g.coord(s), 50.0),
            grid.nearest(&g, g.coord(t), 50.0),
        ) else {
            unsnapped += 1;
            continue;
        };
        for alg in routing::coord::Alg::ALL {
            let got = routing::coord::route(&mut search, &g, &m, fs, ts, alg);
            if let Some(r) = &got {
                if r.same_edge && alg == routing::coord::Alg::Dijkstra {
                    same_edge += 1;
                }
            }
            let got = got.map(|r| r.cost_ms);
            if got != want {
                if mismatches < 5 {
                    eprintln!(
                        "MISMATCH pair {i} {}: node route {want:?} ms, coordinate route {got:?} ms",
                        alg.name()
                    );
                }
                mismatches += 1;
            }
        }
    }

    println!(
        "coordinate routing: {} pairs x 3 algorithms, {same_edge} resolved on a single edge, {unsnapped} unsnappable",
        set.pairs.len()
    );
    if mismatches > 0 {
        eprintln!("\n{mismatches} coordinate/node mismatches - this is a gate");
        std::process::exit(1);
    }
    println!("every coordinate route matches its node route exactly");
    Ok(())
}

/// Five deliberately awkward cases plus a uniform sweep of the bbox, every one
/// checked against a linear scan of every edge.
fn snap(graph_path: &Path, n: usize, seed: u64) -> Result<(), Box<dyn std::error::Error>> {
    let (g, _) = Graph::load(graph_path)?;
    let t0 = Instant::now();
    let grid = graph::grid::Grid::build(&g);
    let build_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let metric = grid.metric();
    let (cols, rows) = grid.dimensions();
    println!(
        "grid      {cols} x {rows} cells at {} m, {} entries, built in {build_ms:.0} ms",
        graph::grid::CELL_M,
        grid.n_entries()
    );

    let (mut min_lon, mut min_lat) = (f64::MAX, f64::MAX);
    let (mut max_lon, mut max_lat) = (f64::MIN, f64::MIN);
    for p in &g.geom {
        min_lon = min_lon.min(p[0] as f64);
        max_lon = max_lon.max(p[0] as f64);
        min_lat = min_lat.min(p[1] as f64);
        max_lat = max_lat.max(p[1] as f64);
    }

    // Open water, an open space with no roads through it, a roundabout, a point
    // exactly on a node, and a point off the map entirely.
    let on_node = g.coord(g.n_nodes() as u32 / 2);
    let named: Vec<(&str, (f64, f64), f64)> = vec![
        ("middle of Sukhna Lake", (76.8150, 30.7440), 5000.0),
        ("Capitol Complex open space", (76.8050, 30.7590), 5000.0),
        ("Sector 17/22 roundabout", (76.7740, 30.7370), 5000.0),
        ("exactly on a node", on_node, 5000.0),
        ("outside the bbox", (77.6000, 31.4000), 1000.0),
    ];

    let mut rng = Rng(seed);
    let mut points: Vec<(&str, (f64, f64), f64)> = named.clone();
    while points.len() < n {
        points.push((
            "",
            (
                min_lon + rng.next_f64() * (max_lon - min_lon),
                min_lat + rng.next_f64() * (max_lat - min_lat),
            ),
            5000.0,
        ));
    }

    let mut times = Vec::with_capacity(points.len());
    let mut brute_times = Vec::with_capacity(points.len());
    let mut mismatches = 0usize;
    let mut misses = 0usize;
    for (label, at, radius) in &points {
        let t = Instant::now();
        let got = grid.nearest(&g, *at, *radius);
        times.push(t.elapsed().as_secs_f64() * 1000.0);

        let t = Instant::now();
        let want = graph::grid::nearest_brute(&g, *at, *radius, &metric);
        brute_times.push(t.elapsed().as_secs_f64() * 1000.0);

        match (got, want) {
            (Some(a), Some(b)) => {
                if a.edge != b.edge || (a.distance_m - b.distance_m).abs() > 1e-6 {
                    if mismatches < 5 {
                        eprintln!(
                            "MISMATCH at {at:?}: grid edge {} at {:.6} m, brute edge {} at {:.6} m",
                            a.edge, a.distance_m, b.edge, b.distance_m
                        );
                    }
                    mismatches += 1;
                }
            }
            (None, None) => misses += 1,
            (a, b) => {
                eprintln!("MISMATCH at {at:?}: grid {a:?} vs brute {b:?}");
                mismatches += 1;
            }
        }
        if !label.is_empty() {
            match got {
                Some(s) => println!(
                    "  {label:<28} edge {:>6} at {:>7.1} m  {}",
                    s.edge,
                    s.distance_m,
                    g.name(s.edge as usize).unwrap_or("(unnamed)")
                ),
                None => println!("  {label:<28} no road within {radius:.0} m"),
            }
        }
    }

    times.sort_by(f64::total_cmp);
    brute_times.sort_by(f64::total_cmp);
    let mean: f64 = times.iter().sum::<f64>() / times.len() as f64;
    let brute_mean: f64 = brute_times.iter().sum::<f64>() / brute_times.len() as f64;
    println!();
    println!(
        "snap      {} points, mean {:.4} ms, p50 {:.4} ms, p99 {:.4} ms, max {:.4} ms",
        points.len(),
        mean,
        percentile(&times, 0.50),
        percentile(&times, 0.99),
        times.last().copied().unwrap_or(0.0)
    );
    println!(
        "brute     mean {:.3} ms, p99 {:.3} ms  ({:.0}x slower than the grid)",
        brute_mean,
        percentile(&brute_times, 0.99),
        brute_mean / mean
    );
    println!("no road within radius: {misses} of {}", points.len());

    if mismatches > 0 {
        eprintln!("\n{mismatches} snap mismatches against brute force - this is a gate");
        std::process::exit(1);
    }
    println!("grid agrees with brute force on every point");
    Ok(())
}

fn shell(cmd: &str, args: &[&str]) -> Option<String> {
    let out = std::process::Command::new(cmd).args(args).output().ok()?;
    let s = String::from_utf8_lossy(&out.stdout).trim().to_string();
    (!s.is_empty()).then_some(s)
}

fn cpu_model() -> String {
    #[cfg(windows)]
    let probe = shell(
        "powershell",
        &[
            "-NoProfile",
            "-Command",
            "(Get-CimInstance Win32_Processor).Name",
        ],
    );
    #[cfg(not(windows))]
    let probe = shell(
        "sh",
        &["-c", "grep -m1 'model name' /proc/cpuinfo | cut -d: -f2"],
    );
    probe.unwrap_or_else(|| "unknown".into()).trim().to_string()
}

fn ram_gb() -> String {
    #[cfg(windows)]
    let probe = shell(
        "powershell",
        &[
            "-NoProfile",
            "-Command",
            "[math]::Round((Get-CimInstance Win32_ComputerSystem).TotalPhysicalMemory/1GB)",
        ],
    );
    #[cfg(not(windows))]
    let probe = shell(
        "sh",
        &[
            "-c",
            "awk '/MemTotal/ {printf \"%.0f\", $2/1048576}' /proc/meminfo",
        ],
    );
    probe.unwrap_or_else(|| "unknown".into())
}

fn rustc_version() -> String {
    shell("rustc", &["--version"]).unwrap_or_else(|| "unknown".into())
}
