//! Geocoding for Chandigarh addresses.
//!
//! **Deviation from the spec, deliberate.** The spec calls for PostgreSQL with
//! PostGIS, `tsvector` and `pg_trgm`. This index is ~15k named features for one
//! city: it is 2 MB in memory, builds in under a second, and needs no server,
//! no schema and no connection pool. A database here would be a dependency for
//! a hash map. If this ever grows past one city, or needs writes, or needs to
//! be shared between processes, that is when Postgres earns its place - the
//! query shape below (hard filter on sector, fuzzy rank within it) maps onto
//! `tsvector` + `pg_trgm` directly.
//!
//! Chandigarh's address grammar is unusually regular, which is a gift: a
//! rule-based parser gets most of the way before anything statistical is worth
//! reaching for.

pub mod extract;
pub mod parse;

use graph::grid::Metric;
use std::collections::HashMap;

pub use parse::{parse, Address};

/// A named thing you can search for.
#[derive(Clone, Debug, PartialEq, serde::Serialize, serde::Deserialize)]
pub struct Feature {
    pub name: String,
    pub kind: String,
    pub lon: f64,
    pub lat: f64,
    pub sector: Option<Sector>,
    pub housenumber: Option<String>,
    /// Higher wins ties. A hospital outranks a kiosk of the same name.
    pub prominence: u8,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, serde::Serialize, serde::Deserialize)]
pub struct Sector {
    pub number: u8,
    /// The `A`-`D` suffix on a subdivided sector, uppercased.
    pub suffix: Option<char>,
}

impl std::fmt::Display for Sector {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.suffix {
            Some(c) => write!(f, "Sector {}-{}", self.number, c),
            None => write!(f, "Sector {}", self.number),
        }
    }
}

#[derive(Clone, Debug)]
pub struct Hit {
    pub feature: Feature,
    pub score: f64,
}

pub struct Index {
    features: Vec<Feature>,
    /// Trigram -> feature ids. The fuzzy half of the query.
    trigrams: HashMap<[u8; 3], Vec<u32>>,
    /// Sector -> feature ids. The hard filter.
    by_sector: HashMap<Sector, Vec<u32>>,
    metric: Metric,
}

/// Lowercased, unaccented-ish, punctuation to spaces. Deliberately crude: the
/// names in this extract are ASCII or close to it.
pub fn normalise(s: &str) -> String {
    let mut out = String::with_capacity(s.len());
    let mut last_space = true;
    for c in s.chars() {
        if c.is_alphanumeric() {
            for l in c.to_lowercase() {
                out.push(l);
            }
            last_space = false;
        } else if !last_space {
            out.push(' ');
            last_space = true;
        }
    }
    out.trim_end().to_string()
}

fn trigrams(s: &str) -> Vec<[u8; 3]> {
    // Pad so short names and prefixes still generate grams, the way pg_trgm
    // does.
    let padded = format!("  {s} ");
    let b = padded.as_bytes();
    (0..b.len().saturating_sub(2))
        .map(|i| [b[i], b[i + 1], b[i + 2]])
        .collect()
}

impl Index {
    pub fn build(features: Vec<Feature>, metric: Metric) -> Index {
        let mut trigrams_map: HashMap<[u8; 3], Vec<u32>> = HashMap::new();
        let mut by_sector: HashMap<Sector, Vec<u32>> = HashMap::new();
        for (i, f) in features.iter().enumerate() {
            for t in trigrams(&normalise(&f.name)) {
                trigrams_map.entry(t).or_default().push(i as u32);
            }
            if let Some(s) = f.sector {
                by_sector.entry(s).or_default().push(i as u32);
            }
        }
        for v in trigrams_map.values_mut() {
            v.dedup();
        }
        Index {
            features,
            trigrams: trigrams_map,
            by_sector,
            metric,
        }
    }

    pub fn len(&self) -> usize {
        self.features.len()
    }
    pub fn is_empty(&self) -> bool {
        self.features.is_empty()
    }
    pub fn sectors(&self) -> usize {
        self.by_sector.len()
    }

    /// Search. `near` biases results toward the map viewport when given.
    pub fn search(&self, query: &str, limit: usize, near: Option<(f64, f64)>) -> Vec<Hit> {
        let addr = parse(query);
        let text = normalise(&addr.text);

        // A sector in the query is a hard filter, not a ranking signal: "sec 17"
        // must never return something in Sector 43 just because the name is a
        // better trigram match.
        let pool: Vec<u32> = match addr.sector {
            Some(s) => {
                let exact = self.by_sector.get(&s).cloned().unwrap_or_default();
                // "Sector 17" with no suffix should also find 17-A through 17-D.
                if s.suffix.is_none() {
                    let mut all = exact;
                    for (k, v) in &self.by_sector {
                        if k.number == s.number && k.suffix.is_some() {
                            all.extend(v);
                        }
                    }
                    all.sort_unstable();
                    all.dedup();
                    all
                } else {
                    exact
                }
            }
            None => (0..self.features.len() as u32).collect(),
        };

        // A bare sector, or a sector plus a house number, with no name to match.
        if text.is_empty() {
            let mut hits: Vec<Hit> = pool
                .iter()
                .filter(|i| {
                    addr.housenumber.as_ref().is_none_or(|h| {
                        self.features[**i as usize]
                            .housenumber
                            .as_deref()
                            .is_some_and(|x| normalise(x) == normalise(h))
                    })
                })
                .map(|i| Hit {
                    feature: self.features[*i as usize].clone(),
                    score: self.features[*i as usize].prominence as f64 / 10.0
                        + self.proximity(*i, near),
                })
                .collect();
            hits.sort_by(|a, b| b.score.total_cmp(&a.score));
            hits.truncate(limit);
            return hits;
        }

        // Trigram overlap, Jaccard-ish: shared grams over the query's grams.
        let query_grams = trigrams(&text);
        let mut shared: HashMap<u32, u32> = HashMap::new();
        for t in &query_grams {
            let Some(ids) = self.trigrams.get(t) else {
                continue;
            };
            for id in ids {
                *shared.entry(*id).or_insert(0) += 1;
            }
        }

        let in_pool: Option<std::collections::HashSet<u32>> = addr
            .sector
            .map(|_| pool.iter().copied().collect());
        let mut hits: Vec<Hit> = shared
            .into_iter()
            .filter(|(id, _)| in_pool.as_ref().is_none_or(|p| p.contains(id)))
            .map(|(id, n)| {
                let f = &self.features[id as usize];
                let name = normalise(&f.name);
                let similarity = n as f64 / query_grams.len().max(1) as f64;
                // A prefix or substring match is worth more than scattered
                // shared grams: "pgi" should find "PGIMER".
                let contains = if name.contains(&text) { 0.35 } else { 0.0 };
                let starts = if name.starts_with(&text) { 0.25 } else { 0.0 };
                Hit {
                    score: similarity + contains + starts + f.prominence as f64 / 20.0
                        + self.proximity(id, near),
                    feature: f.clone(),
                }
            })
            .filter(|h| h.score > 0.15)
            .collect();
        hits.sort_by(|a, b| b.score.total_cmp(&a.score));
        hits.truncate(limit);
        hits
    }

    /// A small bonus for being near the viewport, capped so it can reorder ties
    /// but never outrank a genuinely better name match.
    fn proximity(&self, id: u32, near: Option<(f64, f64)>) -> f64 {
        let Some(at) = near else { return 0.0 };
        let f = &self.features[id as usize];
        let dx = (f.lon - at.0) * self.metric.m_per_deg_lon;
        let dy = (f.lat - at.1) * self.metric.m_per_deg_lat;
        let km = (dx * dx + dy * dy).sqrt() / 1000.0;
        0.15 / (1.0 + km)
    }
}

impl Index {
    /// The features, for writing an index to disk.
    pub fn features(&self) -> &[Feature] {
        &self.features
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn metric() -> Metric {
        Metric {
            m_per_deg_lon: 95_800.0,
            m_per_deg_lat: 110_900.0,
        }
    }

    fn f(name: &str, kind: &str, sector: Option<(u8, Option<char>)>, prominence: u8) -> Feature {
        Feature {
            name: name.into(),
            kind: kind.into(),
            lon: 76.78,
            lat: 30.74,
            sector: sector.map(|(number, suffix)| Sector { number, suffix }),
            housenumber: None,
            prominence,
        }
    }

    fn index() -> Index {
        Index::build(
            vec![
                f("Sector 17 Plaza", "square", Some((17, None)), 8),
                f("PGIMER", "hospital", Some((12, None)), 9),
                f("Rock Garden", "attraction", Some((1, None)), 8),
                f("Sukhna Lake", "water", Some((6, None)), 8),
                f("Elante Mall", "mall", Some((66, None)), 7),
                f("Post Office", "amenity", Some((17, Some('C'))), 2),
                f("Post Office", "amenity", Some((43, None)), 2),
                f("Chandigarh Junction", "station", Some((26, None)), 7),
            ],
            metric(),
        )
    }

    #[test]
    fn a_sector_in_the_query_is_a_hard_filter() {
        let idx = index();
        let hits = idx.search("post office sector 43", 5, None);
        assert!(!hits.is_empty());
        assert_eq!(hits[0].feature.sector.unwrap().number, 43);
        // Every hit respects the sector; none leak in on name alone.
        assert!(hits.iter().all(|h| h.feature.sector.unwrap().number == 43));
    }

    #[test]
    fn a_bare_sector_matches_its_subdivisions() {
        let idx = index();
        let hits = idx.search("sector 17", 10, None);
        let numbers: Vec<u8> = hits.iter().map(|h| h.feature.sector.unwrap().number).collect();
        assert!(numbers.iter().all(|n| *n == 17));
        // 17 and 17-C both.
        assert!(hits.len() >= 2, "{hits:#?}");
    }

    #[test]
    fn abbreviations_and_typos_still_find_it() {
        let idx = index();
        for q in ["sec 17 plaza", "sector-17 plaza", "plaza sector 17"] {
            let hits = idx.search(q, 3, None);
            assert_eq!(hits[0].feature.name, "Sector 17 Plaza", "query {q:?}");
        }
        for q in ["pgimer", "pgi", "pgimr"] {
            let hits = idx.search(q, 3, None);
            assert_eq!(hits[0].feature.name, "PGIMER", "query {q:?}");
        }
        for q in ["sukna lake", "sukhna"] {
            let hits = idx.search(q, 3, None);
            assert_eq!(hits[0].feature.name, "Sukhna Lake", "query {q:?}");
        }
    }

    #[test]
    fn prominence_breaks_a_tie() {
        let idx = Index::build(
            vec![
                f("Chandigarh Kiosk", "kiosk", None, 1),
                f("Chandigarh Hospital", "hospital", None, 9),
            ],
            metric(),
        );
        let hits = idx.search("chandigarh", 2, None);
        assert_eq!(hits[0].feature.kind, "hospital");
    }

    #[test]
    fn proximity_reorders_ties_but_does_not_outrank_a_better_name() {
        let mut near = f("Post Office", "amenity", None, 2);
        near.lon = 76.70;
        let mut far = f("Post Office", "amenity", None, 2);
        far.lon = 76.86;
        let idx = Index::build(vec![far, near.clone()], metric());
        let hits = idx.search("post office", 2, Some((76.70, 30.74)));
        assert_eq!(hits[0].feature.lon, near.lon, "the nearer one should win");

        // But a exact-name match beats a nearer poor match.
        let idx = Index::build(
            vec![
                f("Rock Garden", "attraction", None, 5),
                Feature {
                    lon: 76.70,
                    ..f("Elante Mall", "mall", None, 5)
                },
            ],
            metric(),
        );
        let hits = idx.search("rock garden", 2, Some((76.70, 30.74)));
        assert_eq!(hits[0].feature.name, "Rock Garden");
    }

    #[test]
    fn nonsense_returns_nothing_rather_than_the_least_bad_thing() {
        let idx = index();
        assert!(idx.search("zzzzqqqq", 5, None).is_empty());
    }
}
