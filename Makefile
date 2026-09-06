.PHONY: data graph test
CARGO ?= cargo

## Regenerate the Chandigarh extract. Local only - downloads 1.6 GB.
data: ; @bash tools/data.sh
## Build data/build/graph.bin from the extract.
graph: ; @$(CARGO) run -p graph --release -- build
test: ; @$(CARGO) test --workspace
