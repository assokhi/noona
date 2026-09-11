//! `geocode build` - extract named features from the PBF into an index file.
//! `geocode query <text>` - search that index from the command line.

use std::path::PathBuf;

fn main() -> Result<(), Box<dyn std::error::Error>> {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let mut pbf = PathBuf::from("data/raw/chandigarh.osm.pbf");
    let mut out = PathBuf::from("data/build/places.json");
    let mut limit = 5usize;
    let Some(cmd) = args.first().cloned() else {
        eprintln!("usage: geocode build [--pbf in.osm.pbf] [--out places.json]");
        eprintln!("       geocode query <text> [--index places.json] [--limit 5]");
        std::process::exit(2);
    };
    let mut terms: Vec<String> = Vec::new();
    let mut i = 1;
    while i < args.len() {
        match args[i].as_str() {
            "--pbf" => {
                pbf = PathBuf::from(&args[i + 1]);
                i += 2;
            }
            "--out" | "--index" => {
                out = PathBuf::from(&args[i + 1]);
                i += 2;
            }
            "--limit" => {
                limit = args[i + 1].parse()?;
                i += 2;
            }
            other => {
                terms.push(other.to_string());
                i += 1;
            }
        }
    }

    match cmd.as_str() {
        "build" => {
            let t0 = std::time::Instant::now();
            let features = geocode::extract::from_pbf(&pbf)?;
            if let Some(p) = out.parent() {
                std::fs::create_dir_all(p)?;
            }
            std::fs::write(&out, serde_json::to_string(&features)?)?;
            let mut by_kind: std::collections::BTreeMap<&str, usize> = Default::default();
            for f in &features {
                *by_kind.entry(f.kind.as_str()).or_insert(0) += 1;
            }
            let mut top: Vec<_> = by_kind.into_iter().collect();
            top.sort_by_key(|(_, n)| std::cmp::Reverse(*n));
            println!(
                "{} named features in {:.1} s, {} with a sector -> {} ({} bytes)",
                features.len(),
                t0.elapsed().as_secs_f64(),
                features.iter().filter(|f| f.sector.is_some()).count(),
                out.display(),
                std::fs::metadata(&out)?.len()
            );
            println!(
                "most common kinds: {}",
                top.iter()
                    .take(8)
                    .map(|(k, n)| format!("{k} {n}"))
                    .collect::<Vec<_>>()
                    .join(", ")
            );
        }
        "query" => {
            let features: Vec<geocode::Feature> = serde_json::from_slice(&std::fs::read(&out)?)?;
            let metric = graph::grid::Metric {
                m_per_deg_lon: graph::haversine((0.0, 30.72), (1.0, 30.72)),
                m_per_deg_lat: graph::haversine((0.0, 30.22), (0.0, 31.22)),
            };
            let idx = geocode::Index::build(features, metric);
            let q = terms.join(" ");
            for h in idx.search(&q, limit, None) {
                println!(
                    "{:>6.3}  {:<40} {:<14} {}",
                    h.score,
                    h.feature.name,
                    h.feature.kind,
                    h.feature
                        .sector
                        .map(|s| s.to_string())
                        .unwrap_or_else(|| format!("{:.5},{:.5}", h.feature.lon, h.feature.lat))
                );
            }
        }
        _ => {
            eprintln!("unknown command {cmd}");
            std::process::exit(2);
        }
    }
    Ok(())
}
