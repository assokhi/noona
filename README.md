# Chandigarh routing engine

A self-hosted map and routing service for Chandigarh UT, built from
OpenStreetMap data with the routing core written from scratch in Rust. No
routing libraries: graph construction, Dijkstra, A\*, bidirectional search,
ALT, contraction hierarchies, snapping, map matching, geocoding and the HTTP
API are all in this repo. Rendering uses MapLibre with self-hosted PMTiles.

The point is to learn how routing engines work, so every optimisation is gated
by exact-cost equality against plain Dijkstra on 1000 fixed OD pairs, and every
measurement lives in [docs/benchmarks.md](docs/benchmarks.md) with the machine
it ran on.

| Algorithm | Nodes settled (mean) | Work vs Dijkstra | Mismatches |
|---|---|---|---|
| Dijkstra | 21,103 | 1.00x | 0 |
| A\* | 11,017 | 1.92x | 0 |
| Bidirectional | 12,026 | 1.75x | 0 |
| ALT, 16 landmarks | 1,273 | 16.57x | 0 |
| **Contraction hierarchies** | **147** | **143.81x** | **0** |

40,572 nodes, 103,658 edges. CH preprocesses in 0.3 s and adds 98,119
shortcuts; 85.5% of the shortcuts its witness search considered turned out to be
unnecessary and were skipped.

## Run it

Three things, in three shells. Windows or Linux; nothing here needs `make`.

```sh
# 1. build the graph, then the two preprocessed structures (once each)
cargo run -p graph --release -- build          # data/build/graph.bin
cargo run -p bench  --release -- landmarks     # data/build/landmarks.bin, for ALT
cargo run -p bench  --release -- ch            # data/build/ch.bin, for CH
cargo run -p geocode --release -- build        # data/build/places.json, for search

# 2. the API on :8080
cargo run -p api --release -- --addr 127.0.0.1:8080 --pool 16

# 3. the map on http://localhost:5173
cd web && npm install && npm run dev
```

The API runs without `landmarks.bin` or `ch.bin`; it refuses `alg=alt` or
`alg=ch` rather than quietly answering with a different algorithm.

If you cannot run `cargo` locally, the `binaries` workflow builds
`graph.exe`, `bench.exe` and `api.exe` on every push and uploads them as a
workflow artifact. Download, unzip, and use them in place of the
`cargo run -p X --release --` prefixes above.

Open <http://localhost:5173>, click an origin, click a destination. The panel
shows nodes settled, edges relaxed and timings for the algorithm you pick;
switching algorithm re-runs the same query.

`make <target>` works on Linux and in CI; the [Makefile](Makefile) is one line
per target and doubles as the list of things you can run.

## Data

`data/raw/` and `data/build/` are gitignored. To regenerate from scratch:

```sh
bash tools/data.sh    # 1.6 GB Geofabrik download, clipped to the bbox with complete_ways
bash tools/tiles.sh   # basemap from the Protomaps global build, ~4.5 MB via range requests
```

[data/raw/PROVENANCE](data/raw/PROVENANCE) records the source hash, bbox and
extract statistics. `data/build/od.json` - the frozen 1000 OD pairs every
benchmark uses - *is* committed.

## Layout

```
crates/osm-parse    .osm.pbf -> way records; knows about OSM tags, not graphs
crates/graph        CSR graph: construct, contract, topology, io, grid (snapping)
crates/routing      Dijkstra, A*, bidirectional, ALT, CH; isochrones;
                    seeded search; coordinate routing
crates/geocode      Chandigarh address grammar and an in-memory place index
crates/api          axum server: /v1/route, /v1/nearest, /v1/isochrone,
                    /v1/match, /v1/geocode, /healthz, /metrics
tools/bench         OD generation and the correctness + latency gates
tools/*.sh, *.py    data pipeline, fixture, CI helpers
web/                MapLibre + Vite, no framework
tests/fixtures      99 KB Sector 17 extract that CI builds against
tests/golden        four routes as GeoJSON; a change here is a signal
docs/benchmarks.md  every number, with the deviations from the spec
```

## Gates

Each exits non-zero on failure. They are what "verified" means in the docs.

```sh
cargo test --workspace
cargo run -p bench --release -- run --alg dijkstra,astar,bidir --reference dijkstra
cargo run -p bench --release -- snap     # grid vs brute-force nearest edge
cargo run -p bench --release -- coord    # coordinate routing vs node routing
cargo run -p bench --release -- run --alg dijkstra,astar,bidir,alt,ch --reference dijkstra
bash tools/assert.sh                     # debug-only assertions, forced on in release
python tools/http_bench.py               # same pairs over HTTP; needs the API running
```

## Conventions

Coordinates are `(lon, lat)`, matching GeoJSON, everywhere. Distances are
metres, durations seconds, edge weights milliseconds as `u32` so the priority
queue stays integral and correctness comparisons are exact. Node ids are dense
indices, never OSM ids.
