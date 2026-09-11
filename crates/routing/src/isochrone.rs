//! Isochrones: how far can you get in N minutes.
//!
//! Deliberately on the plain graph. CH does not help here and cannot: it throws
//! away the search space, and the search space is exactly the answer. A CH query
//! visits a few hundred nodes near the hierarchy's top; an isochrone needs every
//! node within the budget. That contrast is worth understanding rather than
//! working around.
//!
//! The boundary is an alpha shape (a concave hull) over the reached nodes. A
//! convex hull would claim the whole gap between two arterials as reachable,
//! which is exactly the thing an isochrone is supposed to show you is not.

use crate::{Search, Seed, UNREACHED};
use graph::grid::Metric;
use graph::Graph;

/// A reached node and how long it took.
#[derive(Clone, Copy, Debug)]
pub struct Reached {
    pub node: u32,
    pub cost_ms: u32,
}

/// Multi-source Dijkstra truncated at a time budget.
pub fn reachable(search: &mut Search, g: &Graph, sources: &[Seed], budget_ms: u32) -> Vec<Reached> {
    search.within_budget(g, sources, budget_ms)
}

impl Search {
    /// Everything reachable within `budget_ms`, with its cost.
    pub fn within_budget(&mut self, g: &Graph, sources: &[Seed], budget_ms: u32) -> Vec<Reached> {
        self.reset();
        let mut out = Vec::new();
        for (v, c) in sources {
            if *c <= budget_ms && *c < self.dist[*v as usize] {
                if self.dist[*v as usize] == UNREACHED {
                    self.touched.push(*v);
                }
                self.dist[*v as usize] = *c;
                self.heap.push(std::cmp::Reverse((*c, *v)));
            }
        }
        while let Some(std::cmp::Reverse((d, u))) = self.heap.pop() {
            if d > self.dist[u as usize] {
                continue;
            }
            out.push(Reached {
                node: u,
                cost_ms: d,
            });
            for e in g.out_edges(u) {
                let v = g.head[e];
                let nd = d + g.weight[e];
                // Truncating here rather than after the fact is the whole
                // saving: the search never leaves the budget.
                if nd <= budget_ms && nd < self.dist[v as usize] {
                    if self.dist[v as usize] == UNREACHED {
                        self.touched.push(v);
                    }
                    self.dist[v as usize] = nd;
                    self.parent[v as usize] = e as u32;
                    self.heap.push(std::cmp::Reverse((nd, v)));
                }
            }
        }
        out
    }
}

/// Alpha shape of a point set, as a ring of `(lon, lat)`.
///
/// Built by taking the convex hull and then repeatedly pushing any hull edge
/// longer than `alpha_m` inward onto the nearest unused point. That is a
/// digging algorithm rather than a true Delaunay alpha shape - a full
/// triangulation would be more principled, and for a few thousand points this
/// gets the shape an isochrone needs without one.
///
/// ponytail: O(n) scan per dig step, so O(n * hull edges). Fine for the few
/// thousand nodes an isochrone reaches; a Delaunay-based alpha shape is the
/// upgrade if isochrones ever need to cover a region rather than a city.
pub fn concave_hull(points: &[(f64, f64)], alpha_m: f64, m: &Metric) -> Vec<(f64, f64)> {
    if points.len() < 3 {
        return points.to_vec();
    }
    let xy: Vec<(f64, f64)> = points
        .iter()
        .map(|p| (p.0 * m.m_per_deg_lon, p.1 * m.m_per_deg_lat))
        .collect();

    let mut hull = convex_hull(&xy);
    if hull.len() < 3 {
        return hull.iter().map(|i| points[*i]).collect();
    }

    let mut used = vec![false; xy.len()];
    for i in &hull {
        used[*i] = true;
    }
    let dist = |a: usize, b: usize| -> f64 {
        let (dx, dy) = (xy[a].0 - xy[b].0, xy[a].1 - xy[b].1);
        (dx * dx + dy * dy).sqrt()
    };

    // Bounded so a pathological point set cannot spin here.
    for _ in 0..points.len() * 2 {
        let mut longest = (0usize, 0.0f64);
        for i in 0..hull.len() {
            let d = dist(hull[i], hull[(i + 1) % hull.len()]);
            if d > longest.1 {
                longest = (i, d);
            }
        }
        if longest.1 <= alpha_m {
            break;
        }
        let (a, b) = (hull[longest.0], hull[(longest.0 + 1) % hull.len()]);
        // The best point to dig to keeps both new edges shorter than the one it
        // replaces, or the boundary would grow instead of tightening.
        let pick = (0..xy.len())
            .filter(|c| !used[*c])
            .map(|c| (c, dist(a, c) + dist(c, b)))
            .filter(|(c, sum)| {
                *sum < longest.1 * 2.0 && dist(a, *c) < longest.1 && dist(*c, b) < longest.1
            })
            .min_by(|x, y| x.1.total_cmp(&y.1));
        let Some((c, _)) = pick else { break };
        used[c] = true;
        hull.insert(longest.0 + 1, c);
    }
    hull.iter().map(|i| points[*i]).collect()
}

/// Andrew's monotone chain, returning indices counter-clockwise.
fn convex_hull(xy: &[(f64, f64)]) -> Vec<usize> {
    let mut order: Vec<usize> = (0..xy.len()).collect();
    order.sort_by(|a, b| {
        xy[*a]
            .0
            .total_cmp(&xy[*b].0)
            .then(xy[*a].1.total_cmp(&xy[*b].1))
    });
    let cross = |o: usize, a: usize, b: usize| -> f64 {
        (xy[a].0 - xy[o].0) * (xy[b].1 - xy[o].1) - (xy[a].1 - xy[o].1) * (xy[b].0 - xy[o].0)
    };
    let mut lower: Vec<usize> = Vec::new();
    for &i in &order {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], i) <= 0.0 {
            lower.pop();
        }
        lower.push(i);
    }
    let mut upper: Vec<usize> = Vec::new();
    for &i in order.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], i) <= 0.0 {
            upper.pop();
        }
        upper.push(i);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn budget_truncates_the_search() {
        let g = crate::tests::grid_graph();
        let mut s = Search::new(g.n_nodes());
        let all = s.within_budget(&g, &[(0, 0)], u32::MAX / 2);
        assert_eq!(
            all.len(),
            g.n_nodes(),
            "an infinite budget reaches the graph"
        );

        let near = s.within_budget(&g, &[(0, 0)], 2000);
        assert!(near.len() < all.len());
        // Every reported cost is inside the budget, and matches Dijkstra.
        for r in &near {
            assert!(r.cost_ms <= 2000);
            let want = s.dijkstra(&g, 0, r.node).map(|x| x.cost_ms);
            assert_eq!(want, Some(r.cost_ms), "cost differs at node {}", r.node);
        }
        // And nothing inside the budget is missing.
        let reached: Vec<u32> = near.iter().map(|r| r.node).collect();
        for v in 0..g.n_nodes() as u32 {
            if let Some(r) = s.dijkstra(&g, 0, v) {
                if r.cost_ms <= 2000 {
                    assert!(
                        reached.contains(&v),
                        "node {v} within budget but not reached"
                    );
                }
            }
        }
    }

    #[test]
    fn concave_hull_is_a_ring_inside_the_point_set() {
        let g = crate::tests::grid_graph();
        let m = Metric::for_graph(&g);
        let pts: Vec<(f64, f64)> = (0..g.n_nodes() as u32).map(|v| g.coord(v)).collect();
        let ring = concave_hull(&pts, 600.0, &m);
        assert!(ring.len() >= 4, "got {} points", ring.len());
        // Every ring vertex is one of the inputs - a hull invents no geometry.
        for p in &ring {
            assert!(pts.iter().any(|q| q == p));
        }
        // A generous alpha degenerates to the convex hull, which for a square
        // grid is its four corners.
        let convex = concave_hull(&pts, 1e9, &m);
        assert_eq!(convex.len(), 4, "{convex:?}");
    }
}
