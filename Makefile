.PHONY: data graph test
data: ; @bash tools/data.sh
graph: ; @cargo run -p graph --release -- build
test: ; @cargo test --workspace
