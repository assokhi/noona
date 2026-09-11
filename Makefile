.PHONY: data graph stats landmarks ch places bench snap coord serve apibench tiles web test fmt lint fixture assert ci
CARGO ?= cargo

## Regenerate the Chandigarh extract. Local only - downloads 1.6 GB.
data: ; @bash tools/data.sh
## Build data/build/graph.bin from the extract.
graph: ; @$(CARGO) run -p graph --release -- build
## Degree histogram and contractibility diagnosis for the built graph.
stats: ; @$(CARGO) run -p graph --release -- stats
## Build the ALT landmark tables.
landmarks: ; @$(CARGO) run -p bench --release -- landmarks --k 16
## Build the contraction hierarchy.
ch: ; @$(CARGO) run -p bench --release -- ch
## Extract named features into the place index.
places: ; @$(CARGO) run -p geocode --release -- build
## The correctness gate over the frozen OD set.
bench: ; @$(CARGO) run -p bench --release -- run --alg dijkstra,astar,bidir,alt,ch --pairs data/build/od.json --reference dijkstra --json docs/bench-phase2.json
## Grid snapping against a brute-force scan of every edge.
snap: ; @$(CARGO) run -p bench --release -- snap --n 1000 --seed 42
## Coordinate routing against node routing on the frozen OD set.
coord: ; @$(CARGO) run -p bench --release -- coord --pairs data/build/od.json
## Run the HTTP API against the built graph.
serve: ; @$(CARGO) run -p api --release -- --addr 127.0.0.1:8080 --pool 16
## Same OD pairs over HTTP, at concurrency 1 and 16. Needs `make serve` running.
apibench: ; @python tools/http_bench.py
## Extract the basemap from the Protomaps global build. Local only.
tiles: ; @bash tools/tiles.sh
## Run the map UI. Needs `make serve` in another shell.
web: ; @cd web && npm install --silent && npm run dev
test: ; @$(CARGO) test --workspace
fmt: ; @$(CARGO) fmt --all --check
lint: ; @$(CARGO) clippy --workspace --all-targets -- -D warnings
## Full clip -> build -> assert path against the committed fixture, no download.
fixture: ; @bash tools/fixture.sh
## Same paths under a profile that forces debug assertions on in release.
assert: ; @bash tools/assert.sh
ci: fmt lint test fixture assert
