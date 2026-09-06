# Benchmarks

Same machine, same extract, every row. Machine: Windows 11, `x86_64-pc-windows-gnullvm`,
`--release` (`opt-level=3`, thin LTO, `codegen-units=1`).

Extract: `data/raw/chandigarh.osm.pbf`, bbox `76.65,30.65,76.87,30.80` with
complete_ways, FNV-1a `0x9e01f16debb933bd`.

## Phase 1 - graph construction

| Metric | Value |
|---|---|
| Ways seen | 72,702 |
| Ways kept | 25,457 (35.0%) |
| Ways rejected | 1,938 class, 3,053 access, 36 area |
| `maxspeed` tagged / parsed / unparseable | 106 / 106 / 0 |
| Roundabouts given an implied oneway | 415 |
| Referenced OSM nodes | 110,836 |
| Intersection nodes (refcount rule) | 43,422 |
| Directed edges before SCC | 108,562 |
| Strongly connected components | 180 |
| Largest SCC | 43,097 nodes, 99.25% |
| Total road length | 4,557.5 km |
| Way node refs with no coordinate | 0 |
| Build time | 649 ms |
| Save / load time | 10 ms / 7 ms |

## Phase 1b - degree-2 contraction

The refcount>=2 junction rule was leaving degree-2 nodes behind, exactly as
suspected. The diagnosis is more interesting than the raw mean degree suggested,
so both halves are recorded.

**Diagnosis** (`graph stats` on the pre-contraction graph, 43,097 nodes):

| (in, out) | Nodes | Share |
|---|---|---|
| (3,3) | 25,218 | 58.51% |
| (1,1) | 7,335 | 17.02% |
| (2,2) | 5,460 | 12.67% |
| (4,4) | 2,460 | 5.71% |
| (2,1) | 1,106 | 2.57% |
| (1,2) | 1,098 | 2.55% |
| (2,3) / (3,2) | 200 / 189 | 0.46% / 0.44% |
| everything else | 31 | 0.07% |

| Topology | Nodes | Share |
|---|---|---|
| Junction | 33,445 | 77.60% |
| Cul-de-sac tip | 6,693 | 15.53% |
| Two-way chain interior | 2,290 | 5.31% |
| One-way chain interior | 669 | 1.55% |
| Stub (no way in or out) | 0 | 0.00% |

Two findings:

- **Degree is not the same question as contractibility.** `(1,1)` is 17% of nodes
  but only 669 of those 7,335 are chain interiors. The other 6,666 are cul-de-sac
  tips: `u -> v` and `v -> u` to the *same* neighbour. Splicing one would make a
  self-loop, so the raw `(1,1)` count would have overstated the opportunity by 10x.
- **The low mean degree is mostly real, not an artefact.** 2.51 against the
  2.8-3.2 expectation is dominated by the 15.5% cul-de-sac tips, which is what
  Chandigarh actually looks like. Contraction moves the mean to 2.56, not to 2.9 -
  the refcount artefact was worth fixing but was never the main term.

**Result** (one round; a second round finds nothing, verified by `graph stats`
reporting 0 still-contractible nodes afterwards):

| | Before | After | Change |
|---|---|---|---|
| Nodes | 43,097 | 40,572 | -2,525 (-5.86%) |
| Directed edges | 108,140 | 103,658 | -4,482 (-4.14%) |
| Mean out-degree | 2.509 | 2.555 | +0.046 |
| Geometry points | 183,092 | 180,567 | -2,525 |
| Polyline segments | 124,668 | 124,668 | 0 |
| Total road length | 4,557.524 km | 4,557.524 km | 0 |
| Fastest edge | 70.1 km/h | 70.0 km/h | - |

Gates, all asserted inside `graph build`:

- Total road length unchanged (tolerance 1 m, actual drift below f32 noise).
- **Polyline segment count unchanged.** *Deviation from the stated gate*, which
  asked for an unchanged geometry *point* count. Points legitimately fall: each
  splice removes a joint that was stored twice, once as the end of the incoming
  polyline and once as the start of the outgoing one. The invariant that does hold
  exactly is the number of polyline segments - splicing joins two lines without
  drawing or erasing one - so that is what the gate asserts. Unchanged road length
  confirms it independently.
- The contracted graph is still a single SCC covering all its nodes.
- Round-trips byte-identically; every node keeps in- and out-degree >= 1.

Nodes deliberately left uncontracted: 411 carrying `highway=traffic_signals` or
`barrier=*`, 16 whose two segments disagree about being two-way, 7 holding a
degree-2 ring open.

## Phase 1c - two audits

**Geometry sharing.** An edge and its reverse each stored their own copy of the
polyline. Measured before fixing: 47,759 two-way pairs holding 152,041 duplicated
points, 46% of a 332,608-point arena. Every pair was an exact mirror, which is a
useful confirmation that the structural twin rule is right. Each road now stores
one forward-oriented span and the reverse direction reads it backwards via a
`FLAG_GEOM_REVERSED` bit. Phase 5 roughly doubles the edge count with shortcuts;
the arena no longer follows.

**`max_speed` in the header.** Was recomputed from the edge arrays at load. Now
fixed at build time and serialised, so the value A\* divides by is provably the
one the weights were built with. `Graph::read_from` rejects a file whose header
maximum does not bound every edge - that check is at load rather than in the build
loop, where it would be tautological, and it is what would fire the first time
anyone adds a speed boost and makes A\* inadmissible.

| `graph.bin` | Bytes | vs v1 |
|---|---|---|
| v1 (pre-contraction, duplicated geometry) | 7,213,681 | - |
| v2 (contracted, `node_flags` + `twin`) | 7,422,251 | +2.9% |
| v3 (shared geometry, header `max_speed`) | 6,620,559 | -8.2% |

### A second bug this surfaced

Contraction copied each source edge's flag byte onto the spliced edge, including
`FLAG_GEOM_REVERSED` - a storage detail of one particular arena, not a property of
the road. Spliced edges inheriting it had their polylines read backwards. The
existing orientation guard did not catch it because it was a `debug_assert!` and
the build that matters runs `--release`. It is now a plain `assert!`: one
comparison per edge, and the class of bug it catches draws the wrong line on the
map while costing exactly the right amount.

### A bug this surfaced

The first implementation derived the two-directions-of-one-road relation from OSM
way provenance: both directions emitted from one way shared an id. That silently
fails on a street mapped as two separate one-way ways - a real case in this
extract - and made total road length depend on how a mapper chose to split
geometry. It is now decided structurally: two directed edges are the same road iff
they run between the same nodes over the identical polyline, reversed. Dual
carriageways keep distinct polylines and correctly stay unpaired.

## Phase 2 - routing

Not built yet. Same 1000 OD pairs and a committed seed, measured on the
**post-contraction** graph (40,572 nodes / 103,658 edges):

| Algorithm | Prep time | Prep memory | p50 (ms) | p95 (ms) | p99 (ms) | Nodes settled (mean) | Speedup vs Dijkstra |
|---|---|---|---|---|---|---|---|
| Dijkstra | - | - | | | | | 1.0x |
| A\* | - | - | | | | | |
| Bidirectional Dijkstra | - | - | | | | | |
| ALT (16 landmarks) | | | | | | | |
| CH | | | | | | | |

## Finding: `maxspeed` coverage is 0.4%

106 of 25,457 kept ways carry a `maxspeed` tag, and 0 of those were unparseable.
Effectively every edge weight comes from the class default table, so route
*plausibility* rests entirely on a table of invented constants.

Route *optimality* is unaffected, and so is every algorithm from Phase 2 through
Phase 5: CH does not care whether weights are realistic, only that they are fixed
and non-negative. The payoff comes in Phase 6, where map-matched GPS traces with
timestamps allow per-class speeds to be derived from real driving rather than
tuned against intuition.

The fastest edge is 70.0 km/h, above every class default because a handful of
ways carry an explicit `maxspeed`. It is carried in the `graph.bin` header, and
Phase 2's A\* heuristic divides by that stored value.
