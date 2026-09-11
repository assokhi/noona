//! Pull named features out of an `.osm.pbf` into a searchable index.
//!
//! Nodes and ways both carry names. A way's position is the mean of the node
//! coordinates we kept for it, which is close enough to a centroid for a mall
//! or a hospital and much cheaper than a real one.

use std::collections::{HashMap, HashSet};
use std::path::Path;

use crate::{Feature, Sector};

/// How much a feature outranks another of the same name. Roughly "how likely is
/// this what someone typing that name meant".
fn prominence(kind: &str, value: &str) -> u8 {
    match (kind, value) {
        (_, "hospital") | (_, "university") | (_, "airport") | (_, "aerodrome") => 9,
        (_, "railway_station") | (_, "bus_station") | (_, "college") => 8,
        (_, "mall") | (_, "attraction") | (_, "museum") | (_, "stadium") => 7,
        (_, "school") | (_, "park") | (_, "government") | (_, "townhall") => 6,
        (_, "marketplace") | (_, "supermarket") | (_, "bank") | (_, "hotel") => 5,
        (_, "restaurant") | (_, "cafe") | (_, "fuel") | (_, "pharmacy") => 4,
        ("place", _) => 8,
        ("tourism", _) | ("leisure", _) => 5,
        ("amenity", _) | ("shop", _) => 3,
        ("office", _) => 2,
        _ => 1,
    }
}

/// `addr:suburb = Sector 17-C`, `addr:city`, or a name that says so.
fn sector_from(tags: &HashMap<&str, &str>) -> Option<Sector> {
    for key in ["addr:suburb", "addr:district", "addr:neighbourhood", "name"] {
        if let Some(v) = tags.get(key) {
            if let Some(s) = crate::parse(v).sector {
                return Some(s);
            }
        }
    }
    None
}

fn feature_from(
    tags: &HashMap<&str, &str>,
    lon: f64,
    lat: f64,
) -> Option<Feature> {
    let name = tags.get("name")?.trim();
    if name.is_empty() {
        return None;
    }
    // The kind is whichever classifying tag is present, most specific first.
    let (kind, value) = [
        "amenity", "shop", "tourism", "leisure", "office", "healthcare", "aeroway", "railway",
        "place", "building",
    ]
    .iter()
    .find_map(|k| tags.get(k).map(|v| (*k, *v)))?;

    Some(Feature {
        name: name.to_string(),
        kind: if value == "yes" {
            kind.to_string()
        } else {
            value.to_string()
        },
        lon,
        lat,
        sector: sector_from(tags),
        housenumber: tags.get("addr:housenumber").map(|s| s.to_string()),
        prominence: prominence(kind, value),
    })
}

/// Two passes: collect named ways and the nodes they need, then read the nodes
/// and emit both kinds of feature.
pub fn from_pbf(path: &Path) -> osmpbf::Result<Vec<Feature>> {
    use osmpbf::{Element, ElementReader};

    struct PendingWay {
        tags: HashMap<String, String>,
        refs: Vec<i64>,
    }

    let mut ways: Vec<PendingWay> = Vec::new();
    let mut needed: HashSet<i64> = HashSet::new();
    ElementReader::from_path(path)?.for_each(|el| {
        let Element::Way(w) = el else { return };
        let tags: HashMap<&str, &str> = w.tags().collect();
        if !tags.contains_key("name") {
            return;
        }
        // Cheap pre-filter: only keep ways that would produce a feature.
        let owned: HashMap<String, String> = tags
            .iter()
            .map(|(k, v)| ((*k).to_string(), (*v).to_string()))
            .collect();
        if feature_from(&tags, 0.0, 0.0).is_none() {
            return;
        }
        // A handful of nodes is enough for a centroid; a 1300-node way does not
        // need all of them to land in the right sector.
        let refs: Vec<i64> = w.refs().step_by(8).take(16).collect();
        if refs.is_empty() {
            return;
        }
        needed.extend(&refs);
        ways.push(PendingWay { tags: owned, refs });
    })?;

    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(needed.len());
    let mut features: Vec<Feature> = Vec::new();
    ElementReader::from_path(path)?.for_each(|el| {
        let (id, lon, lat, tags): (i64, f64, f64, HashMap<&str, &str>) = match &el {
            Element::Node(n) => (n.id(), n.lon(), n.lat(), n.tags().collect()),
            Element::DenseNode(n) => (n.id(), n.lon(), n.lat(), n.tags().collect()),
            _ => return,
        };
        if needed.contains(&id) {
            coords.insert(id, (lon, lat));
        }
        if let Some(f) = feature_from(&tags, lon, lat) {
            features.push(f);
        }
    })?;

    for w in &ways {
        let pts: Vec<(f64, f64)> = w.refs.iter().filter_map(|r| coords.get(r).copied()).collect();
        if pts.is_empty() {
            continue;
        }
        let lon = pts.iter().map(|p| p.0).sum::<f64>() / pts.len() as f64;
        let lat = pts.iter().map(|p| p.1).sum::<f64>() / pts.len() as f64;
        let borrowed: HashMap<&str, &str> = w
            .tags
            .iter()
            .map(|(k, v)| (k.as_str(), v.as_str()))
            .collect();
        if let Some(f) = feature_from(&borrowed, lon, lat) {
            features.push(f);
        }
    }

    // Same name, same kind, within a few metres: one thing mapped twice.
    features.sort_by(|a, b| {
        a.name
            .cmp(&b.name)
            .then(a.kind.cmp(&b.kind))
            .then(a.lon.total_cmp(&b.lon))
    });
    features.dedup_by(|a, b| {
        a.name == b.name
            && a.kind == b.kind
            && graph::haversine((a.lon, a.lat), (b.lon, b.lat)) < 50.0
    });
    Ok(features)
}
