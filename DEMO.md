# Demo

Fifteen minutes, no Rust toolchain needed.

## 1. Get the binaries

The local toolchain is blocked by Smart App Control, so CI builds them. Open
<https://github.com/assokhi/noona/actions/workflows/binaries.yml>, click the
newest green run, and download the **chd-maps-windows** artifact at the bottom.
Unzip it into `target/release/` in this repo.

You should have `graph.exe`, `bench.exe`, `api.exe` and `geocode.exe`. They are
MSVC builds, so they need no extra DLLs.

> If you ever turn Smart App Control off (Windows Security → App & browser
> control → Smart App Control → Off, one-way), every `cargo` command in
> [README.md](README.md) works directly and you can skip this step.

## 2. Build the data

`data/raw/chandigarh.osm.pbf` is committed, so nothing gets downloaded here.

```powershell
target\release\graph.exe build          # ~0.3 s  -> data/build/graph.bin
target\release\bench.exe landmarks      # ~0.1 s  -> data/build/landmarks.bin
target\release\bench.exe ch             # ~0.3 s  -> data/build/ch.bin
```

```powershell
targetelease\geocode.exe build        # ~0.1 s  -> data/build/places.json
```

## 3. Show the numbers

This is the part worth showing first, because it is the whole point of the
project. Same 1000 origin/destination pairs, five algorithms, all gated on
returning *exactly* the same cost as plain Dijkstra:

```powershell
target\release\bench.exe run --alg dijkstra,astar,bidir,alt,ch --reference dijkstra
```

```
| Algorithm | p50 (ms) | Nodes settled | Latency | Work  | Mismatches |
| dijkstra  |    1.565 |        21,103 |  1.00x  | 1.00x |          0 |
| astar     |    1.110 |        11,017 |  1.41x  | 1.92x |          0 |
| bidir     |    0.950 |        12,026 |  1.65x  | 1.75x |          0 |
| alt       |    0.181 |         1,273 |  8.66x  |16.57x |          0 |
| ch        |    0.047 |           147 | 32.98x  |143.81x|          0 |
```

The mismatch column is the claim. Every one of those speedups returns the
identical route cost on all 1000 pairs, and the harness exits non-zero if not.

Two more, each also a gate:

```powershell
target\release\bench.exe snap    # grid snapping vs a linear scan of every edge
target\release\bench.exe coord   # coordinate routing vs node routing
```

## 4. Show the map

Two shells:

```powershell
target\release\api.exe --addr 127.0.0.1:8080 --pool 16
```

```powershell
cd web
npm install
npm run dev
```

Open **<http://localhost:5173>** — that exact host. Vite binds IPv6 `localhost`,
and the Geolocation API needs `localhost` or HTTPS anyway.

What to do, roughly in this order:

1. **Click two points.** A route draws: wide dark casing under a bright line.
   Orange dashed lines show how far each click was from the road it snapped to.
2. **Switch algorithm** in the panel. Same query, re-issued. Watch **nodes
   settled** in the debug panel go from ~21,000 for dijkstra to ~150 for ch
   while the route and the duration stay identical. That is the demo.
3. **Press isochrone.** Three bands from the origin: 5, 10 and 15 minutes. Note
   that it deliberately does *not* use CH — CH throws away the search space, and
   for an isochrone the search space is the answer.
4. **Search**, if you built the place index. Try `pgimer`, `sec 43`,
   `sukhna lake`, `rock garden`. The parser output is in the response, so a bad
   parse is visible rather than mysterious.
5. **Track me**, on a phone. Needs HTTPS or localhost, so a LAN IP will not
   work — use a tunnel. Toggle **raw fix** to see the unfiltered GPS jumping
   between parallel roads next to the Kalman-filtered dot. That difference is
   the argument for the map matching in `/v1/match`.

## 5. Poke the API directly

```powershell
curl "http://127.0.0.1:8080/healthz"
curl "http://127.0.0.1:8080/v1/route?from=76.7794,30.7410&to=76.7648,30.7649&alg=ch"
curl "http://127.0.0.1:8080/v1/nearest?lon=76.8150&lat=30.7440"
curl "http://127.0.0.1:8080/metrics"
```

The second one carries a `debug` block with nodes settled, edges relaxed,
search time, snap time and both snap distances. The third snaps a point in the
middle of Sukhna Lake and honestly reports that the nearest road is 421 m away.

Error handling is typed, not stringly:

```powershell
curl "http://127.0.0.1:8080/v1/route?from=76.7794,30.7410&to=200,30.7"   # MALFORMED_COORDINATE, 400
curl "http://127.0.0.1:8080/v1/nearest?lon=76.90&lat=30.60"             # NO_ROAD_WITHIN_RADIUS, 404
```

## What is worth pointing at

- `docs/benchmarks.md` is the artefact. Every number, the machine it came from,
  and every place the implementation deviates from the spec with the reason.
- The bugs it caught are in there too: a twin relation derived from OSM way
  provenance that made total road length depend on how a mapper split geometry;
  a storage flag copied onto spliced edges that drew polylines backwards; a
  benchmark that was measuring CPU thermal state rather than code.
- `tests/golden/` holds four routes as GeoJSON. If a later change moves them,
  the test fails and you see it.
