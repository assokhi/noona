//! ALT: A* with landmarks and the triangle inequality.
//!
//! A* with haversine is bounded by the fastest edge in the graph, and on this
//! graph that bound is about 3x loose because the typical edge is 25-35 km/h
//! against a 70 km/h maximum. Landmarks replace the speed assumption with real
//! network distances: for a landmark L, `d(v, t) >= d(v, L) - d(t, L)` and
//! `d(v, t) >= d(L, t) - d(L, v)`, both by the triangle inequality, so the
//! larger of them over every landmark is a lower bound that already knows the
//! road network is not a plane.
//!
//! Preprocessing is 2k full Dijkstras for k landmarks; the tables are
//! `2 * k * n_nodes` u32s, 5 MB here for k = 16.

use crate::{Search, UNREACHED};
use graph::Graph;
use std::fs::File;
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::Path;

const MAGIC: &[u8; 8] = b"CHDLANDM";
pub const DEFAULT_LANDMARKS: usize = 16;

pub struct Landmarks {
    pub nodes: Vec<u32>,
    n: usize,
    /// `from[l * n + v]` = cost from landmark l to v.
    from: Vec<u32>,
    /// `to[l * n + v]` = cost from v to landmark l.
    to: Vec<u32>,
}

impl Landmarks {
    pub fn count(&self) -> usize {
        self.nodes.len()
    }

    /// Farthest-point sampling on network distance: each new landmark is the
    /// node furthest from every landmark chosen so far. Spreads them to the
    /// edges of the graph, which is where they give the tightest bounds.
    pub fn build(g: &Graph, k: usize) -> Landmarks {
        let n = g.n_nodes();
        let mut search = Search::new(n);
        let mut nodes: Vec<u32> = Vec::with_capacity(k);
        let mut from: Vec<u32> = Vec::with_capacity(k * n);
        let mut to: Vec<u32> = Vec::with_capacity(k * n);
        let mut nearest = vec![u32::MAX; n];

        // Start from the node furthest from an arbitrary one, so the first
        // landmark is already on the periphery rather than wherever node 0 is.
        let mut next = argmax(&search.distances_from(g, 0, false));
        for _ in 0..k {
            let l = next;
            nodes.push(l);
            let fwd = search.distances_from(g, l, false);
            let bwd = search.distances_from(g, l, true);
            for v in 0..n {
                nearest[v] = nearest[v].min(fwd[v]);
            }
            from.extend_from_slice(&fwd);
            to.extend_from_slice(&bwd);
            next = argmax(&nearest);
        }
        Landmarks { nodes, n, from, to }
    }

    /// Lower bound on the cost from `v` to `t`, in milliseconds.
    #[inline]
    pub fn bound(&self, v: u32, t: u32) -> u32 {
        let (v, t) = (v as usize, t as usize);
        let mut best = 0u32;
        for l in 0..self.nodes.len() {
            let base = l * self.n;
            let (vl, tl) = (self.to[base + v], self.to[base + t]);
            let (lv, lt) = (self.from[base + v], self.from[base + t]);
            if vl != UNREACHED && tl != UNREACHED {
                best = best.max(vl.saturating_sub(tl));
            }
            if lv != UNREACHED && lt != UNREACHED {
                best = best.max(lt.saturating_sub(lv));
            }
        }
        best
    }

    pub fn save(&self, path: &Path, source_hash: u64) -> io::Result<()> {
        let mut w = BufWriter::new(File::create(path)?);
        w.write_all(MAGIC)?;
        w.write_all(&source_hash.to_le_bytes())?;
        w.write_all(&(self.nodes.len() as u32).to_le_bytes())?;
        w.write_all(&(self.n as u32).to_le_bytes())?;
        for v in self.nodes.iter().chain(&self.from).chain(&self.to) {
            w.write_all(&v.to_le_bytes())?;
        }
        w.flush()
    }

    pub fn load(path: &Path) -> io::Result<(Landmarks, u64)> {
        let mut r = BufReader::new(File::open(path)?);
        let mut magic = [0u8; 8];
        r.read_exact(&mut magic)?;
        if &magic != MAGIC {
            return Err(io::Error::new(
                io::ErrorKind::InvalidData,
                "not a landmarks file",
            ));
        }
        let mut b8 = [0u8; 8];
        r.read_exact(&mut b8)?;
        let source_hash = u64::from_le_bytes(b8);
        let mut b4 = [0u8; 4];
        r.read_exact(&mut b4)?;
        let k = u32::from_le_bytes(b4) as usize;
        r.read_exact(&mut b4)?;
        let n = u32::from_le_bytes(b4) as usize;
        let mut read = |count: usize| -> io::Result<Vec<u32>> {
            let mut buf = vec![0u8; count * 4];
            r.read_exact(&mut buf)?;
            Ok(buf
                .as_chunks::<4>()
                .0
                .iter()
                .map(|c| u32::from_le_bytes(*c))
                .collect())
        };
        let nodes = read(k)?;
        let from = read(k * n)?;
        let to = read(k * n)?;
        Ok((Landmarks { nodes, n, from, to }, source_hash))
    }
}

fn argmax(v: &[u32]) -> u32 {
    let mut best = 0usize;
    for (i, x) in v.iter().enumerate() {
        // Unreached nodes are not "far", they are absent.
        if *x != UNREACHED && (v[best] == UNREACHED || *x > v[best]) {
            best = i;
        }
    }
    best as u32
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bound_never_exceeds_the_true_cost() {
        let g = crate::tests::grid_graph();
        let lm = Landmarks::build(&g, 4);
        assert_eq!(lm.count(), 4);
        let mut s = Search::new(g.n_nodes());
        for a in 0..36u32 {
            for b in 0..36u32 {
                let Some(r) = s.dijkstra(&g, a, b) else {
                    continue;
                };
                assert!(
                    lm.bound(a, b) <= r.cost_ms,
                    "bound {} exceeds true cost {} for {a} -> {b}",
                    lm.bound(a, b),
                    r.cost_ms
                );
            }
        }
        // And it is exact when either end is a landmark.
        let l = lm.nodes[0];
        for b in 0..36u32 {
            let want = s.dijkstra(&g, l, b).map(|r| r.cost_ms).unwrap_or(0);
            assert_eq!(lm.bound(l, b), want, "{l} -> {b}");
        }
    }

    #[test]
    fn landmarks_round_trip() {
        let g = crate::tests::grid_graph();
        let lm = Landmarks::build(&g, 3);
        let dir = std::env::temp_dir().join(format!("chd-landmarks-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("lm.bin");
        lm.save(&path, 42).unwrap();
        let (back, hash) = Landmarks::load(&path).unwrap();
        assert_eq!(hash, 42);
        assert_eq!(back.nodes, lm.nodes);
        assert_eq!(back.from, lm.from);
        assert_eq!(back.to, lm.to);
        std::fs::remove_dir_all(&dir).ok();
    }
}
