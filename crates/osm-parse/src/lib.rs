//! `.osm.pbf` -> routable way records and the node coordinates they reference.
//!
//! Coordinates are `(lon, lat)` in that order, everywhere, matching GeoJSON.
//! This crate knows about OSM tags and nothing about graphs.

use osmpbf::{Element, ElementReader};
use std::collections::{HashMap, HashSet};
use std::path::Path;

/// Routable `highway` values and their default speed in km/h when `maxspeed`
/// is absent. Tuned for Indian urban conditions, not European defaults.
/// The index into this table is the class ordinal stored in the edge flags.
pub const CLASSES: [(&str, f64); 10] = [
    ("motorway", 80.0),      // 0 - rare inside the UT
    ("trunk", 60.0),         // 1 - roughly the V2 network
    ("primary", 50.0),       // 2 - V3 sector-dividing roads
    ("secondary", 45.0),     // 3
    ("tertiary", 35.0),      // 4 - V4 shopping streets
    ("unclassified", 30.0),  // 5
    ("residential", 25.0),   // 6 - V5/V6 inside sectors
    ("living_street", 12.0), // 7
    ("service", 15.0),       // 8
    ("road", 30.0),          // 9 - kept by the spec, no speed given; treat as unclassified
];

/// Only motorway..tertiary have `_link` variants in the keep list.
const MAX_LINK_CLASS: usize = 4;
/// `*_link` speed is the parent class speed times this.
pub const LINK_FACTOR: f64 = 0.7;
/// Travel-time multiplier for `service=parking_aisle`: routable, but routes
/// should not cut through a parking lot to save five seconds.
pub const PARKING_AISLE_PENALTY: f32 = 3.0;

/// Edge flag bits.
pub const FLAG_CLASS_MASK: u8 = 0x0f;
pub const FLAG_LINK: u8 = 0x10;
pub const FLAG_PARKING_AISLE: u8 = 0x20;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Oneway {
    Both,
    Forward,
    Reverse,
}

/// A retained way, reduced to what graph construction needs.
#[derive(Clone, Debug)]
pub struct Way {
    pub id: i64,
    pub nodes: Vec<i64>,
    pub speed_kmh: f64,
    pub oneway: Oneway,
    pub name: Option<String>,
    pub penalty: f32,
    pub flags: u8,
}

#[derive(Default, Debug)]
pub struct ParseStats {
    pub ways_seen: u64,
    pub ways_kept: u64,
    pub rejected_access: u64,
    pub rejected_area: u64,
    pub rejected_class: u64,
    pub maxspeed_present: u64,
    pub maxspeed_parsed: u64,
    /// Histogram of `maxspeed` values we could not read. The distribution tells
    /// you how good the local OSM coverage actually is.
    pub maxspeed_unparsed: HashMap<String, u32>,
    pub roundabout_implied_oneway: u64,
}

impl ParseStats {
    pub fn unparsed_total(&self) -> u32 {
        self.maxspeed_unparsed.values().sum()
    }
    /// Most common unparseable `maxspeed` values, worst first.
    pub fn top_unparsed(&self, n: usize) -> Vec<(&str, u32)> {
        let mut v: Vec<_> = self
            .maxspeed_unparsed
            .iter()
            .map(|(k, c)| (k.as_str(), *c))
            .collect();
        v.sort_by(|a, b| b.1.cmp(&a.1).then(a.0.cmp(b.0)));
        v.truncate(n);
        v
    }
}

/// What a `maxspeed` tag turned out to be.
#[derive(Clone, Copy, PartialEq, Debug)]
pub enum MaxSpeed {
    /// A usable limit in km/h.
    Kmh(f64),
    /// Recognised, but not a number (`none`, `signals`): use the class default.
    Unposted,
    /// Could not be read at all. Worth counting - the histogram of these is how
    /// you find out what the local mappers actually write.
    Unreadable,
}

pub fn parse_maxspeed(raw: &str) -> MaxSpeed {
    let s = raw.trim().to_ascii_lowercase();
    if s.is_empty() || s == "none" || s == "signals" || s == "variable" || s == "unposted" {
        return MaxSpeed::Unposted;
    }
    let split = s
        .find(|c: char| !c.is_ascii_digit() && c != '.')
        .unwrap_or(s.len());
    let (num, unit) = s.split_at(split);
    let Ok(v) = num.parse::<f64>() else {
        return MaxSpeed::Unreadable;
    };
    if !(v.is_finite() && v > 0.0) {
        return MaxSpeed::Unreadable;
    }
    MaxSpeed::Kmh(match unit.trim() {
        "" | "km/h" | "kmh" | "kph" | "kmph" => v,
        "mph" => v * 1.609_344,
        "knots" => v * 1.852,
        _ => return MaxSpeed::Unreadable,
    })
}

/// `Some((class ordinal, is_link))` for a routable `highway` value.
fn classify(hw: &str) -> Option<(usize, bool)> {
    let (base, is_link) = match hw.strip_suffix("_link") {
        Some(b) => (b, true),
        None => (hw, false),
    };
    let idx = CLASSES.iter().position(|(n, _)| *n == base)?;
    if is_link && idx > MAX_LINK_CLASS {
        return None;
    }
    Some((idx, is_link))
}

/// First pass: every routable way, with speed and directionality resolved.
pub fn read_ways(path: &Path) -> osmpbf::Result<(Vec<Way>, ParseStats)> {
    let mut ways = Vec::new();
    let mut stats = ParseStats::default();

    ElementReader::from_path(path)?.for_each(|element| {
        let Element::Way(w) = element else { return };
        stats.ways_seen += 1;

        let (mut hw, mut oneway_tag, mut junction) = (None, None, None);
        let (mut maxspeed, mut name, mut service) = (None, None, None);
        let (mut access, mut motor_vehicle) = (None, None);
        let mut area = false;
        for (k, v) in w.tags() {
            match k {
                "highway" => hw = Some(v),
                "oneway" => oneway_tag = Some(v),
                "junction" => junction = Some(v),
                "maxspeed" => maxspeed = Some(v),
                "name" => name = Some(v),
                "service" => service = Some(v),
                "access" => access = Some(v),
                "motor_vehicle" => motor_vehicle = Some(v),
                "area" => area = v == "yes",
                _ => {}
            }
        }

        // `motor_vehicle` overrides the generic `access` tag when both are present.
        let blocked = match (motor_vehicle, access) {
            (Some("no"), _) => true,
            (Some(_), _) => false,
            (None, Some("no" | "private")) => true,
            _ => false,
        };
        if blocked {
            stats.rejected_access += 1;
            return;
        }
        if area {
            stats.rejected_area += 1;
            return;
        }
        // `construction` and `proposed` are simply not in CLASSES.
        let Some(hw) = hw else { return };
        let Some((class, is_link)) = classify(hw) else {
            stats.rejected_class += 1;
            return;
        };

        let nodes: Vec<i64> = w.refs().collect();
        if nodes.len() < 2 {
            return;
        }

        // An explicit `oneway` tag wins over the roundabout implication.
        let oneway = match oneway_tag {
            Some("yes" | "true" | "1") => Oneway::Forward,
            Some("-1" | "reverse") => Oneway::Reverse,
            Some("no" | "false" | "0") => Oneway::Both,
            _ if matches!(junction, Some("roundabout" | "circular")) => {
                stats.roundabout_implied_oneway += 1;
                Oneway::Forward
            }
            _ => Oneway::Both,
        };

        let default_kmh = CLASSES[class].1 * if is_link { LINK_FACTOR } else { 1.0 };
        let speed_kmh = match maxspeed {
            None => default_kmh,
            Some(raw) => {
                stats.maxspeed_present += 1;
                match parse_maxspeed(raw) {
                    MaxSpeed::Kmh(v) => {
                        stats.maxspeed_parsed += 1;
                        v
                    }
                    MaxSpeed::Unposted => default_kmh,
                    MaxSpeed::Unreadable => {
                        *stats.maxspeed_unparsed.entry(raw.to_string()).or_insert(0) += 1;
                        default_kmh
                    }
                }
            }
        };

        let parking_aisle = service == Some("parking_aisle");
        let mut flags = class as u8;
        if is_link {
            flags |= FLAG_LINK;
        }
        if parking_aisle {
            flags |= FLAG_PARKING_AISLE;
        }

        stats.ways_kept += 1;
        ways.push(Way {
            id: w.id(),
            nodes,
            speed_kmh,
            oneway,
            name: name.map(str::to_string),
            penalty: if parking_aisle {
                PARKING_AISLE_PENALTY
            } else {
                1.0
            },
            flags,
        });
    })?;

    Ok((ways, stats))
}

/// OSM node id -> `(lon, lat)`. Dropped once dense ids are assigned.
pub type NodeCoords = HashMap<i64, (f64, f64)>;

/// Node ids carrying a routing-relevant tag, so graph construction can force
/// them to be intersection nodes even at refcount 1.
#[derive(Default, Debug)]
pub struct TaggedNodes {
    pub signals: HashSet<i64>,
    pub barriers: HashSet<i64>,
}

/// Second pass: coordinates as `(lon, lat)` for `needed` only, plus the
/// routing-relevant node tags encountered along the way.
pub fn read_nodes(path: &Path, needed: &HashSet<i64>) -> osmpbf::Result<(NodeCoords, TaggedNodes)> {
    let mut coords: HashMap<i64, (f64, f64)> = HashMap::with_capacity(needed.len());
    let mut tagged = TaggedNodes::default();

    ElementReader::from_path(path)?.for_each(|element| {
        let (id, lon, lat) = match &element {
            Element::Node(n) => (n.id(), n.lon(), n.lat()),
            Element::DenseNode(n) => (n.id(), n.lon(), n.lat()),
            _ => return,
        };
        if !needed.contains(&id) {
            return;
        }
        coords.insert(id, (lon, lat));
        let tags: Vec<(&str, &str)> = match &element {
            Element::Node(n) => n.tags().collect(),
            Element::DenseNode(n) => n.tags().collect(),
            _ => return,
        };
        for (k, v) in tags {
            match k {
                "highway" if v == "traffic_signals" => {
                    tagged.signals.insert(id);
                }
                "barrier" => {
                    tagged.barriers.insert(id);
                }
                _ => {}
            }
        }
    })?;

    Ok((coords, tagged))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn maxspeed_forms() {
        assert_eq!(parse_maxspeed("50"), MaxSpeed::Kmh(50.0));
        assert_eq!(parse_maxspeed("50 km/h"), MaxSpeed::Kmh(50.0));
        assert_eq!(parse_maxspeed(" 50kmh "), MaxSpeed::Kmh(50.0));
        assert_eq!(parse_maxspeed("none"), MaxSpeed::Unposted);
        let MaxSpeed::Kmh(mph) = parse_maxspeed("30 mph") else {
            panic!("30 mph should parse")
        };
        assert!((mph - 48.28).abs() < 0.01, "{mph}");
        assert_eq!(parse_maxspeed("IN:urban"), MaxSpeed::Unreadable);
        assert_eq!(parse_maxspeed("fast"), MaxSpeed::Unreadable);
        assert_eq!(parse_maxspeed("0"), MaxSpeed::Unreadable);
    }

    #[test]
    fn classes_and_links() {
        assert_eq!(classify("residential"), Some((6, false)));
        assert_eq!(classify("primary_link"), Some((2, true)));
        // residential_link is not a thing we route on
        assert_eq!(classify("residential_link"), None);
        assert_eq!(classify("construction"), None);
        assert_eq!(classify("footway"), None);
        // link speed is 70% of the parent
        assert_eq!(CLASSES[2].1 * LINK_FACTOR, 35.0);
    }
}
