.PHONY: data graph stats bench snap test fmt lint fixture assert ci
CARGO ?= cargo

## Regenerate the Chandigarh extract. Local only - downloads 1.6 GB.
data: ; @bash tools/data.sh
## Build data/build/graph.bin from the extract.
graph: ; @$(CARGO) run -p graph --release -- build
## Degree histogram and contractibility diagnosis for the built graph.
stats: ; @$(CARGO) run -p graph --release -- stats
## The correctness gate over the frozen OD set.
bench: ; @$(CARGO) run -p bench --release -- run --alg dijkstra,astar,bidir --pairs data/build/od.json --reference dijkstra --json docs/bench-phase2.json
## Grid snapping against a brute-force scan of every edge.
snap: ; @$(CARGO) run -p bench --release -- snap --n 1000 --seed 42
test: ; @$(CARGO) test --workspace
fmt: ; @$(CARGO) fmt --all --check
lint: ; @$(CARGO) clippy --workspace --all-targets -- -D warnings
## Full clip -> build -> assert path against the committed fixture, no download.
fixture: ; @bash tools/fixture.sh
## Same paths under a profile that forces debug assertions on in release.
assert: ; @bash tools/assert.sh
ci: fmt lint test fixture assert
