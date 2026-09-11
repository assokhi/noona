#!/usr/bin/env bash
# Assertions that guard an expensive algorithmic invariant stay debug-only, so
# --release never runs them. That is exactly how the FLAG_GEOM_REVERSED splice
# bug survived. This runs the same paths under a profile with assertions forced
# on, against the committed fixture, so the debug-only bucket is not dead code.
set -euo pipefail
cd "$(dirname "$0")/.."

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

python -c "import osmium" 2>/dev/null || python -m pip install --quiet osmium
python tools/clip.py tests/fixtures/sector17.osm.pbf "$WORK/clip.osm.pbf" 76.772,30.732,76.793,30.748

P=(cargo run --quiet --profile release-assert)
"${P[@]}" -p graph -- build --pbf "$WORK/clip.osm.pbf" --out "$WORK/graph.bin"
"${P[@]}" -p bench -- gen --n 100 --seed 42 --graph "$WORK/graph.bin" --out "$WORK/od.json"
"${P[@]}" -p bench -- landmarks --k 8 --graph "$WORK/graph.bin" --landmarks "$WORK/lm.bin"
"${P[@]}" -p bench -- run --alg dijkstra,astar,bidir,alt \
  --graph "$WORK/graph.bin" --pairs "$WORK/od.json" --landmarks "$WORK/lm.bin" \
  --reference dijkstra

echo "release-assert: fixture build and bench ran with assertions on"
