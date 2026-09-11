//! The Chandigarh address grammar, as a rule-based parser.
//!
//! ```text
//! SECTOR  := ("sector" | "sec") WS? "-"? NUMBER ("-"? [A-D])?
//! HOUSE   := ("house" | "h.no" | "hno" | "#") WS? NUMBER
//! PHASE   := "phase" WS? (ROMAN | NUMBER)        # Mohali
//! ADDRESS := HOUSE? SECTOR | POI_NAME (","? SECTOR)?
//! ```
//!
//! The grid is regular enough that this gets most queries right before anything
//! statistical is worth reaching for. Whatever the parser does not claim is
//! left in `text` for the fuzzy name search.

use crate::Sector;

#[derive(Clone, Debug, Default, PartialEq)]
pub struct Address {
    pub sector: Option<Sector>,
    pub housenumber: Option<String>,
    /// Mohali phases, kept separate because they are a different grid.
    pub phase: Option<u8>,
    /// Everything the parser did not consume.
    pub text: String,
}

fn roman(s: &str) -> Option<u8> {
    Some(match s {
        "i" => 1,
        "ii" => 2,
        "iii" => 3,
        "iv" => 4,
        "v" => 5,
        "vi" => 6,
        "vii" => 7,
        "viii" => 8,
        "ix" => 9,
        "x" => 10,
        "xi" => 11,
        _ => return None,
    })
}

/// Pull the structured parts out of a free-text query.
pub fn parse(query: &str) -> Address {
    let mut out = Address::default();
    // Keep `#` and `-` and `.` as their own tokens-ish: they carry meaning here.
    let lower = query.to_lowercase().replace('#', " # ").replace(',', " ");
    let tokens: Vec<String> = lower
        .split_whitespace()
        .map(|t| t.trim_matches('.').to_string())
        .filter(|t| !t.is_empty())
        .collect();

    let mut rest: Vec<String> = Vec::new();
    let mut i = 0;
    while i < tokens.len() {
        let t = tokens[i].trim_matches('-');
        // "sector 17", "sec-17c", "sector17"
        if matches!(t, "sector" | "sec") || t.starts_with("sector") || t.starts_with("sec-") {
            let inline = t
                .trim_start_matches("sector")
                .trim_start_matches("sec")
                .trim_start_matches('-');
            let (raw, consumed) = if !inline.is_empty() {
                (inline.to_string(), 1)
            } else if i + 1 < tokens.len() {
                (tokens[i + 1].trim_matches('-').to_string(), 2)
            } else {
                (String::new(), 1)
            };
            if let Some(mut s) = sector_token(&raw) {
                i += consumed;
                // "sector 17 c" spells the suffix as its own token.
                if s.suffix.is_none() {
                    if let Some(next) = tokens.get(i) {
                        let t = next.trim_matches('-');
                        if t.len() == 1 {
                            if let Some(c) = t.chars().next().filter(|c| ('a'..='d').contains(c)) {
                                s.suffix = Some(c.to_ascii_uppercase());
                                i += 1;
                            }
                        }
                    }
                }
                out.sector = Some(s);
                continue;
            }
        }
        // "phase 7", "phase-vii"
        if t == "phase" || t.starts_with("phase") {
            let inline = t.trim_start_matches("phase").trim_start_matches('-');
            let (raw, consumed) = if !inline.is_empty() {
                (inline.to_string(), 1)
            } else if i + 1 < tokens.len() {
                (tokens[i + 1].trim_matches('-').to_string(), 2)
            } else {
                (String::new(), 1)
            };
            if let Some(n) = raw.parse::<u8>().ok().or_else(|| roman(&raw)) {
                out.phase = Some(n);
                i += consumed;
                continue;
            }
        }
        // "#234", "h.no 234", "house 234"
        if matches!(t, "#" | "house" | "hno" | "h" | "hno.") || t.starts_with("h.no") {
            if let Some(next) = tokens.get(i + 1) {
                let n = next.trim_matches('-');
                if n.chars().next().is_some_and(|c| c.is_ascii_digit()) {
                    out.housenumber = Some(n.to_string());
                    i += 2;
                    continue;
                }
            }
            i += 1;
            continue;
        }
        rest.push(tokens[i].clone());
        i += 1;
    }

    // A leading bare number with a sector elsewhere is a house number: "234
    // sector 40" is unambiguous once the sector is claimed.
    if out.housenumber.is_none() && out.sector.is_some() {
        if let Some(first) = rest.first() {
            if first.chars().all(|c| c.is_ascii_digit()) {
                out.housenumber = Some(first.clone());
                rest.remove(0);
            }
        }
    }
    out.text = rest.join(" ");
    out
}

/// `17`, `17c`, `17-c`, `17 c`.
fn sector_token(raw: &str) -> Option<Sector> {
    let raw = raw.trim_matches('-');
    let digits: String = raw.chars().take_while(|c| c.is_ascii_digit()).collect();
    if digits.is_empty() {
        return None;
    }
    let number: u8 = digits.parse().ok()?;
    if !(1..=70).contains(&number) {
        return None;
    }
    let tail: String = raw[digits.len()..]
        .chars()
        .filter(|c| c.is_ascii_alphabetic())
        .collect();
    let suffix = tail
        .chars()
        .next()
        .filter(|c| ('a'..='d').contains(c))
        .map(|c| c.to_ascii_uppercase());
    Some(Sector { number, suffix })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sec(number: u8, suffix: Option<char>) -> Option<Sector> {
        Some(Sector { number, suffix })
    }

    #[test]
    fn sector_spellings() {
        for q in [
            "sector 17",
            "sec 17",
            "sec-17",
            "sector-17",
            "sector17",
            "Sector 17",
        ] {
            assert_eq!(parse(q).sector, sec(17, None), "query {q:?}");
        }
        for q in ["sector 17-c", "sec 17c", "sec-17c", "sector 17 c"] {
            assert_eq!(parse(q).sector, sec(17, Some('C')), "query {q:?}");
        }
    }

    #[test]
    fn house_numbers() {
        assert_eq!(parse("#234 sector 40").housenumber.as_deref(), Some("234"));
        assert_eq!(parse("h.no 234 sec 40").housenumber.as_deref(), Some("234"));
        assert_eq!(
            parse("house 234 sector 40").housenumber.as_deref(),
            Some("234")
        );
        // Bare leading number only counts once a sector is present.
        assert_eq!(parse("234 sector 40").housenumber.as_deref(), Some("234"));
        assert_eq!(parse("234").housenumber, None);
        assert_eq!(parse("234").text, "234");
    }

    #[test]
    fn mohali_phases() {
        assert_eq!(parse("phase 7").phase, Some(7));
        assert_eq!(parse("phase-vii").phase, Some(7));
        assert_eq!(parse("Phase XI mohali").phase, Some(11));
        assert_eq!(parse("Phase XI mohali").text, "mohali");
    }

    #[test]
    fn the_name_survives_the_parse() {
        let a = parse("Sector 17 Plaza");
        assert_eq!(a.sector, sec(17, None));
        assert_eq!(a.text, "plaza");

        let a = parse("elante mall");
        assert_eq!(a.sector, None);
        assert_eq!(a.text, "elante mall");
    }

    #[test]
    fn an_out_of_range_sector_is_just_text() {
        // Chandigarh stops well short of 900; this is a name, not a sector.
        assert_eq!(parse("sector 900").sector, None);
    }
}
