.PHONY: data graph test fmt lint fixture ci
CARGO ?= cargo

## Regenerate the Chandigarh extract. Local only - downloads 1.6 GB.
data: ; @bash tools/data.sh
## Build data/build/graph.bin from the extract.
graph: ; @$(CARGO) run -p graph --release -- build
test: ; @$(CARGO) test --workspace
fmt: ; @$(CARGO) fmt --all --check
lint: ; @$(CARGO) clippy --workspace --all-targets -- -D warnings
## Full clip -> build -> assert path against the committed fixture, no download.
fixture: ; @bash tools/fixture.sh
ci: fmt lint test fixture
