//! Local shape of a node, and strongly connected components.

use crate::Graph;

/// A degree-2 node and the edges that would be spliced if it were contracted.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Chain {
    pub u: u32,
    pub w: u32,
    /// `(edge u->v, edge v->w)`, in travel order.
    pub fwd: (u32, u32),
    /// `(edge w->v, edge v->u)`, present only when the chain is two-way.
    pub bwd: Option<(u32, u32)>,
}

/// A node's local topology.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Shape {
    /// `u -> v -> w` and nothing else.
    OneWayChain,
    /// `u <-> v <-> w` and nothing else.
    TwoWayChain,
    /// Exactly one undirected neighbour: the tip of a cul-de-sac. Legitimate.
    CulDeSac,
    /// No way in, or no way out. Cannot occur inside the largest SCC; counted
    /// separately so it is never confused with a chain interior.
    Stub,
    /// A real junction, or an asymmetry that encodes a directional constraint.
    Junction,
}

/// The chain through `v`, if `v` is a topologically degree-2 node that can be
/// spliced away without changing any shortest path.
///
/// Deliberately strict. `u->v`, `v->w`, `w->v` with no `v->u` is *not* a chain:
/// that asymmetry is a real one-way constraint and splicing it would invent a
/// turn that does not exist.
pub fn chain_at(g: &Graph, v: u32) -> Option<Chain> {
    let outs: Vec<(u32, u32)> = g.out_edges(v).map(|e| (e as u32, g.head[e])).collect();
    let ins: Vec<(u32, u32)> = g.in_edges(v).map(|j| (g.r_edge[j], g.r_head[j])).collect();
    // A self-loop is never contractible.
    if outs.iter().any(|(_, h)| *h == v) || ins.iter().any(|(_, t)| *t == v) {
        return None;
    }
    match (ins.len(), outs.len()) {
        (1, 1) => {
            let ((e_in, u), (e_out, w)) = (ins[0], outs[0]);
            // u == w is a cul-de-sac tip, not a chain: splicing makes a self-loop.
            (u != w).then_some(Chain {
                u,
                w,
                fwd: (e_in, e_out),
                bwd: None,
            })
        }
        (2, 2) => {
            let mut nb: Vec<u32> = ins.iter().map(|(_, t)| *t).collect();
            nb.sort_unstable();
            nb.dedup();
            if nb.len() != 2 {
                return None;
            }
            let (u, w) = (nb[0], nb[1]);
            // Every edge must pair with a reverse, or the node is a constraint.
            let e_uv = ins.iter().find(|(_, t)| *t == u)?.0;
            let e_wv = ins.iter().find(|(_, t)| *t == w)?.0;
            let e_vu = outs.iter().find(|(_, h)| *h == u)?.0;
            let e_vw = outs.iter().find(|(_, h)| *h == w)?.0;
            Some(Chain {
                u,
                w,
                fwd: (e_uv, e_vw),
                bwd: Some((e_wv, e_vu)),
            })
        }
        _ => None,
    }
}

pub fn shape_at(g: &Graph, v: u32) -> Shape {
    if g.in_edges(v).is_empty() || g.out_edges(v).is_empty() {
        return Shape::Stub;
    }
    if let Some(c) = chain_at(g, v) {
        return if c.bwd.is_some() {
            Shape::TwoWayChain
        } else {
            Shape::OneWayChain
        };
    }
    let mut nb: Vec<u32> = g
        .out_edges(v)
        .map(|e| g.head[e])
        .chain(g.in_edges(v).map(|j| g.r_head[j]))
        .collect();
    nb.sort_unstable();
    nb.dedup();
    if nb.len() == 1 && nb[0] != v {
        return Shape::CulDeSac;
    }
    Shape::Junction
}

/// Iterative Tarjan. Returns `(component per node, size per component)`.
/// Iterative because 10^5 nodes will overflow the stack in the recursive form.
pub fn scc(n: usize, offsets: &[u32], head: &[u32]) -> (Vec<u32>, Vec<u32>) {
    const UNVISITED: u32 = u32::MAX;
    let mut index = vec![UNVISITED; n];
    let mut low = vec![0u32; n];
    let mut on_stack = vec![false; n];
    let mut comp = vec![UNVISITED; n];
    let mut sizes: Vec<u32> = Vec::new();
    let mut stack: Vec<u32> = Vec::new();
    let mut call: Vec<(u32, u32)> = Vec::new(); // (node, next edge cursor)
    let mut next_index = 0u32;

    for s in 0..n as u32 {
        if index[s as usize] != UNVISITED {
            continue;
        }
        index[s as usize] = next_index;
        low[s as usize] = next_index;
        next_index += 1;
        stack.push(s);
        on_stack[s as usize] = true;
        call.push((s, offsets[s as usize]));

        while let Some(&(v, ei)) = call.last() {
            if ei < offsets[v as usize + 1] {
                call.last_mut().unwrap().1 = ei + 1;
                let w = head[ei as usize];
                if index[w as usize] == UNVISITED {
                    index[w as usize] = next_index;
                    low[w as usize] = next_index;
                    next_index += 1;
                    stack.push(w);
                    on_stack[w as usize] = true;
                    call.push((w, offsets[w as usize]));
                } else if on_stack[w as usize] {
                    low[v as usize] = low[v as usize].min(index[w as usize]);
                }
                continue;
            }

            call.pop();
            if low[v as usize] == index[v as usize] {
                let id = sizes.len() as u32;
                let mut size = 0u32;
                loop {
                    let w = stack.pop().unwrap();
                    on_stack[w as usize] = false;
                    comp[w as usize] = id;
                    size += 1;
                    if w == v {
                        break;
                    }
                }
                sizes.push(size);
            }
            if let Some(&(parent, _)) = call.last() {
                low[parent as usize] = low[parent as usize].min(low[v as usize]);
            }
        }
    }
    (comp, sizes)
}
