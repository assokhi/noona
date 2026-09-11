//! Map matching: snap a *sequence* of noisy GPS fixes to a coherent path.
//!
//! Newson & Krumm (2009), as an HMM decoded with Viterbi.
//!
//! - **States** are candidate edges near each fix.
//! - **Emission** is a zero-mean Gaussian over the perpendicular distance from
//!   the fix to the candidate.
//! - **Transition** is exponential over `|great_circle(p, q) - route_distance|`.
//!   Two consecutive fixes are a fixed distance apart as the crow flies; if the
//!   road distance between two candidates is wildly different from that, those
//!   candidates are probably not both on the true path.
//!
//! The route distance is what makes this phase depend on CH. A trace with 200
//! fixes and 8 candidates each needs up to 200 * 64 shortest paths, and on the
//! plain graph that is minutes rather than milliseconds.
//!
//! Everything is done in log space. Multiplying a few hundred probabilities
//! each around 1e-3 underflows f64 long before the trace ends.

use graph::grid::{Grid, Snap};
use graph::Graph;

use crate::ch::Ch;
use crate::Search;

#[derive(Clone, Copy, Debug)]
pub struct Fix {
    pub at: (f64, f64),
    pub timestamp_ms: i64,
    /// Reported horizontal accuracy, if the device gave one.
    pub accuracy_m: Option<f64>,
}

#[derive(Clone, Copy, Debug)]
pub struct Params {
    /// Emission sigma. Newson & Krumm derive 4.07 m from their data; 10 m is
    /// closer to a phone in a dense grid. Tune against a hand-labelled trace.
    pub sigma_m: f64,
    /// Transition beta, in metres of route-vs-crow discrepancy.
    pub beta_m: f64,
    /// How far from a fix to look for candidates.
    pub radius_m: f64,
    pub max_candidates: usize,
    /// Fixes closer together than this are treated as the vehicle standing
    /// still and collapsed.
    pub stationary_m: f64,
    /// A gap longer than this breaks the trace: a tunnel, or the app was
    /// backgrounded. Guessing across it invents a route nobody drove.
    pub max_gap_ms: i64,
}

impl Default for Params {
    fn default() -> Self {
        Params {
            sigma_m: 10.0,
            beta_m: 30.0,
            radius_m: 60.0,
            max_candidates: 8,
            stationary_m: 5.0,
            max_gap_ms: 60_000,
        }
    }
}

/// One matched fix.
#[derive(Clone, Copy, Debug)]
pub struct Matched {
    /// Index into the input trace.
    pub fix: usize,
    /// `None` when no candidate was within the radius: an honest gap rather
    /// than a forced match.
    pub snap: Option<Snap>,
    /// Which contiguous segment of the trace this belongs to. A new segment
    /// starts after a time gap or an unmatchable fix.
    pub segment: usize,
}

#[derive(Debug, Default)]
pub struct MatchResult {
    pub points: Vec<Matched>,
    /// Original edge ids of the inferred path, in travel order.
    pub edges: Vec<u32>,
    /// Mean emission distance over matched fixes, in metres. Low is good.
    pub mean_offset_m: f64,
    pub matched: usize,
    pub unmatched: usize,
    pub segments: usize,
    pub collapsed: usize,
}

/// Natural log of an unnormalised Gaussian density.
fn log_emission(distance_m: f64, sigma_m: f64) -> f64 {
    let z = distance_m / sigma_m;
    -0.5 * z * z
}

/// Natural log of an unnormalised exponential density over the discrepancy
/// between crow-flies and road distance.
fn log_transition(crow_m: f64, route_m: f64, beta_m: f64) -> f64 {
    -(route_m - crow_m).abs() / beta_m
}

const NEG_INF: f64 = f64::NEG_INFINITY;

/// Viterbi over the candidate lattice.
pub fn match_trace(
    search: &mut Search,
    g: &Graph,
    grid: &Grid,
    ch: &Ch,
    trace: &[Fix],
    p: Params,
) -> MatchResult {
    let mut out = MatchResult::default();
    if trace.is_empty() {
        return out;
    }

    // A stationary vehicle produces a cloud of fixes in one place. Keeping them
    // all makes the lattice quadratically larger and tells you nothing.
    let mut kept: Vec<usize> = vec![0];
    for i in 1..trace.len() {
        let last = trace[*kept.last().expect("kept starts non-empty")];
        if graph::haversine(last.at, trace[i].at) < p.stationary_m
            && trace[i].timestamp_ms - last.timestamp_ms < p.max_gap_ms
        {
            out.collapsed += 1;
            continue;
        }
        kept.push(i);
    }

    // Candidates per kept fix.
    let candidates: Vec<Vec<Snap>> = kept
        .iter()
        .map(|i| nearby(g, grid, trace[*i].at, p))
        .collect();

    // Viterbi, restarted at every break. `prev` holds the best log-probability
    // of reaching each candidate of the previous fix.
    let mut prev_scores: Vec<f64> = Vec::new();
    let mut back: Vec<Vec<usize>> = vec![Vec::new(); kept.len()];
    let mut segment_of: Vec<usize> = vec![0; kept.len()];
    let mut segment = 0usize;
    let mut started = false;

    for (step, &fix_idx) in kept.iter().enumerate() {
        let cands = &candidates[step];
        back[step] = vec![usize::MAX; cands.len()];
        segment_of[step] = segment;

        if cands.is_empty() {
            // No road within the radius. Emit a gap and restart.
            prev_scores.clear();
            started = false;
            segment += 1;
            continue;
        }

        let sigma = trace[fix_idx]
            .accuracy_m
            .map_or(p.sigma_m, |a| a.clamp(p.sigma_m * 0.5, p.sigma_m * 4.0));
        let emission: Vec<f64> = cands
            .iter()
            .map(|c| log_emission(c.distance_m, sigma))
            .collect();

        let gap = step > 0
            && trace[fix_idx].timestamp_ms - trace[kept[step - 1]].timestamp_ms > p.max_gap_ms;
        if !started || gap {
            if gap {
                segment += 1;
                segment_of[step] = segment;
            }
            prev_scores = emission;
            started = true;
            continue;
        }

        let prev_cands = &candidates[step - 1];
        let crow = graph::haversine(trace[kept[step - 1]].at, trace[fix_idx].at);
        let mut scores = vec![NEG_INF; cands.len()];
        for (j, c) in cands.iter().enumerate() {
            for (i, pc) in prev_cands.iter().enumerate() {
                if prev_scores[i] == NEG_INF {
                    continue;
                }
                let Some(route_m) = route_distance(search, g, ch, pc, c) else {
                    continue;
                };
                let s = prev_scores[i] + log_transition(crow, route_m, p.beta_m) + emission[j];
                if s > scores[j] {
                    scores[j] = s;
                    back[step][j] = i;
                }
            }
        }
        // Every transition was impossible: treat it as a break rather than
        // letting the whole trace collapse to -inf.
        if scores.iter().all(|s| *s == NEG_INF) {
            segment += 1;
            segment_of[step] = segment;
            scores = emission;
            for b in back[step].iter_mut() {
                *b = usize::MAX;
            }
        }
        prev_scores = scores;
    }

    // Walk the backpointers from the best final state.
    let mut chosen: Vec<Option<usize>> = vec![None; kept.len()];
    let mut step = kept.len();
    let mut cursor: Option<usize> = None;
    while step > 0 {
        step -= 1;
        if candidates[step].is_empty() {
            cursor = None;
            continue;
        }
        let pick = match cursor {
            Some(c) => c,
            None => {
                // Start of a run: take the best-scoring candidate here. Only
                // the final step still has its scores, so for earlier runs the
                // emission alone decides, which is what a restart means.
                let scores: Vec<f64> = if step == kept.len() - 1 && !prev_scores.is_empty() {
                    prev_scores.clone()
                } else {
                    candidates[step]
                        .iter()
                        .map(|c| log_emission(c.distance_m, p.sigma_m))
                        .collect()
                };
                scores
                    .iter()
                    .enumerate()
                    .filter(|(_, s)| **s > NEG_INF)
                    .max_by(|a, b| a.1.total_cmp(b.1))
                    .map(|(i, _)| i)
                    .unwrap_or(0)
            }
        };
        chosen[step] = Some(pick);
        cursor = match back[step].get(pick) {
            Some(&b) if b != usize::MAX => Some(b),
            _ => None,
        };
    }

    let mut offsets = 0.0;
    for (step, &fix_idx) in kept.iter().enumerate() {
        let snap = chosen[step].and_then(|i| candidates[step].get(i).copied());
        match snap {
            Some(s) => {
                out.matched += 1;
                offsets += s.distance_m;
            }
            None => out.unmatched += 1,
        }
        out.points.push(Matched {
            fix: fix_idx,
            snap,
            segment: segment_of[step],
        });
    }
    out.mean_offset_m = if out.matched > 0 {
        offsets / out.matched as f64
    } else {
        0.0
    };
    out.segments = segment + 1;

    // Stitch the inferred path: the route between consecutive matched points.
    for w in out.points.windows(2) {
        let (Some(a), Some(b)) = (w[0].snap, w[1].snap) else {
            continue;
        };
        if w[0].segment != w[1].segment {
            continue;
        }
        if a.edge == b.edge {
            if out.edges.last() != Some(&a.edge) {
                out.edges.push(a.edge);
            }
            continue;
        }
        let sources = crate::coord::source_seeds_for(g, &a);
        let targets = crate::coord::target_seeds_for(g, &b);
        if let Some((_, edges, _)) = search.ch_multi(ch, &sources, &targets) {
            for e in edges {
                if out.edges.last() != Some(&e) {
                    out.edges.push(e);
                }
            }
        }
    }
    out
}

/// Candidate edges near a fix, nearest first, capped.
fn nearby(g: &Graph, grid: &Grid, at: (f64, f64), p: Params) -> Vec<Snap> {
    let mut seen: Vec<Snap> = Vec::new();
    // The grid returns one nearest edge; widening the search by re-querying
    // around the point picks up parallel carriageways, which is exactly the
    // ambiguity map matching exists to resolve.
    let step = p.radius_m / 2.0;
    let m = grid.metric();
    for dx in [-1.0, 0.0, 1.0] {
        for dy in [-1.0, 0.0, 1.0] {
            let probe = (
                at.0 + dx * step / m.m_per_deg_lon,
                at.1 + dy * step / m.m_per_deg_lat,
            );
            if let Some(s) = grid.nearest(g, probe, p.radius_m) {
                let real = graph::haversine(at, s.point);
                if real <= p.radius_m && !seen.iter().any(|x| x.edge == s.edge) {
                    seen.push(Snap {
                        distance_m: real,
                        ..s
                    });
                }
            }
        }
    }
    seen.sort_by(|a, b| a.distance_m.total_cmp(&b.distance_m));
    seen.truncate(p.max_candidates);
    seen
}

/// Road distance between two snapped points, in metres.
fn route_distance(search: &mut Search, g: &Graph, ch: &Ch, a: &Snap, b: &Snap) -> Option<f64> {
    if a.edge == b.edge {
        return Some((b.offset_m - a.offset_m).abs());
    }
    let sources = crate::coord::source_seeds_for(g, a);
    let targets = crate::coord::target_seeds_for(g, b);
    let (_, edges, _) = search.ch_multi(ch, &sources, &targets)?;
    let mid: f64 = edges.iter().map(|e| g.length[*e as usize] as f64).sum();
    // The partial legs at each end, same as coordinate routing.
    Some(mid + (g.length[a.edge as usize] as f64 - a.offset_m) + b.offset_m)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_space_ordering_is_sane() {
        // Closer is better, and a route that matches the crow line is better
        // than one that does not.
        assert!(log_emission(1.0, 10.0) > log_emission(20.0, 10.0));
        assert!(log_transition(100.0, 105.0, 30.0) > log_transition(100.0, 400.0, 30.0));
        // And the scale does not underflow over a long trace.
        let total: f64 = (0..500).map(|_| log_emission(15.0, 10.0)).sum();
        assert!(total.is_finite());
    }

    #[test]
    fn a_noisy_trace_along_a_known_route_recovers_it() {
        let g = crate::tests::grid_graph();
        let grid = Grid::build(&g);
        let (ch, _) = crate::ch::build(&g, crate::ch::Limits::default());
        let mut s = Search::new(g.n_nodes());

        // Take a real route, sample points along its geometry, and jitter them
        // with a deterministic pseudo-random wobble.
        let truth = s.dijkstra(&g, 0, 35).expect("grid is connected");
        let pts = truth.geometry(&g);
        let mut seed = 12345u64;
        let mut rnd = move || {
            seed = seed.wrapping_mul(6364136223846793005).wrapping_add(1);
            ((seed >> 33) as f64 / (1u64 << 31) as f64) - 1.0
        };
        let m = grid.metric();
        let trace: Vec<Fix> = pts
            .iter()
            .enumerate()
            .map(|(i, p)| Fix {
                at: (
                    p[0] as f64 + rnd() * 12.0 / m.m_per_deg_lon,
                    p[1] as f64 + rnd() * 12.0 / m.m_per_deg_lat,
                ),
                timestamp_ms: i as i64 * 5_000,
                accuracy_m: Some(12.0),
            })
            .collect();

        let r = match_trace(&mut s, &g, &grid, &ch, &trace, Params::default());
        assert_eq!(r.points.len(), trace.len() - r.collapsed);
        assert!(r.matched > 0, "nothing matched");
        assert!(
            r.mean_offset_m < 30.0,
            "mean offset {} m is too high",
            r.mean_offset_m
        );
        // The inferred path should be edges of the original graph that chain.
        for w in r.edges.windows(2) {
            let (a, b) = (w[0] as usize, w[1] as usize);
            assert_eq!(
                g.head[a],
                g.edge_source(b),
                "inferred path does not chain at {a} -> {b}"
            );
        }
    }

    #[test]
    fn a_long_time_gap_breaks_the_trace_instead_of_guessing() {
        let g = crate::tests::grid_graph();
        let grid = Grid::build(&g);
        let (ch, _) = crate::ch::build(&g, crate::ch::Limits::default());
        let mut s = Search::new(g.n_nodes());
        let trace = vec![
            Fix {
                at: g.coord(0),
                timestamp_ms: 0,
                accuracy_m: None,
            },
            Fix {
                at: g.coord(1),
                timestamp_ms: 5_000,
                accuracy_m: None,
            },
            // Ten minutes later, on the far side of the grid.
            Fix {
                at: g.coord(35),
                timestamp_ms: 605_000,
                accuracy_m: None,
            },
        ];
        let r = match_trace(&mut s, &g, &grid, &ch, &trace, Params::default());
        assert!(
            r.segments >= 2,
            "expected a break, got {} segment",
            r.segments
        );
        assert_ne!(
            r.points[1].segment, r.points[2].segment,
            "the gap should start a new segment"
        );
    }

    #[test]
    fn a_stationary_cloud_collapses() {
        let g = crate::tests::grid_graph();
        let grid = Grid::build(&g);
        let (ch, _) = crate::ch::build(&g, crate::ch::Limits::default());
        let mut s = Search::new(g.n_nodes());
        let at = g.coord(0);
        let trace: Vec<Fix> = (0..20)
            .map(|i| Fix {
                at,
                timestamp_ms: i * 1000,
                accuracy_m: Some(8.0),
            })
            .collect();
        let r = match_trace(&mut s, &g, &grid, &ch, &trace, Params::default());
        assert_eq!(r.collapsed, 19, "all but one fix should collapse");
        assert_eq!(r.points.len(), 1);
    }
}
