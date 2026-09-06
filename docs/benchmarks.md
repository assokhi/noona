# Benchmarks

Same machine, same extract, every row. Machine: Windows 11, `x86_64-pc-windows-gnullvm`,
`--release` (`opt-level=3`, thin LTO, `codegen-units=1`).

## Phase 1 - graph construction

Extract: `data/raw/chandigarh.osm.pbf`, bbox `76.65,30.65,76.87,30.80` with
complete_ways, FNV-1a `0x9e01f16debb933bd`.

| Metric | Value |
|---|---|
| Ways seen | 72,702 |
| Ways kept | 25,457 (35.0%) |
| Ways rejected | 1,938 class, 3,053 access, 36 area |
| `maxspeed` tagged / parsed / unparseable | 106 / 106 / 0 |
| Roundabouts given an implied oneway | 415 |
| Referenced OSM nodes | 110,836 |
| Intersection nodes | 43,422 |
| Degree-2 contraction ratio | 2.55x |
| Directed edges before SCC | 108,562 |
| Strongly connected components | 180 |
| Largest SCC | 43,097 nodes, 99.25% |
| **Graph** | **43,097 nodes / 108,140 directed edges** |
| Geometry points | 337,090 |
| Total road length | 4,557.5 km |
| Way node refs with no coordinate | 0 |
| Fastest edge | 70.1 km/h |
| Build time | 285 ms |
| Save / load time | 6 ms / 4 ms |
| `graph.bin` size | 7,213,681 bytes |

Notes:

- **`maxspeed` coverage is 0.4%** (106 of 25,457 kept ways). Effectively every
  edge weight comes from the class default table, so those defaults, not OSM,
  decide what the router thinks is fast. Worth revisiting if routes look wrong.
- **0 missing coordinates** is the check that complete_ways actually worked. A
  non-zero count means ways were truncated at the bbox edge.
- **Fastest edge is 70.1 km/h**, above every class default because a handful of
  ways carry an explicit `maxspeed`. Phase 2's A\* heuristic must divide by this
  measured maximum (`Graph::max_speed_m_per_ms`), not by an assumed one and
  never by the average - an inadmissible heuristic fails silently.
- 180 components for 43,422 intersection nodes: the discarded 0.75% is the usual
  OSM debris - service roads with a mistagged oneway, fragments whose connecting
  node sat outside the coarse cut.

## Phase 2 - routing

Not built yet. Table format, to be filled with the same 1000 OD pairs and a
committed seed:

| Algorithm | Prep time | Prep memory | p50 (ms) | p95 (ms) | p99 (ms) | Nodes settled (mean) | Speedup vs Dijkstra |
|---|---|---|---|---|---|---|---|
| Dijkstra | - | - | | | | | 1.0x |
| A\* | - | - | | | | | |
| Bidirectional Dijkstra | - | - | | | | | |
| ALT (16 landmarks) | | | | | | | |
| CH | | | | | | | |
