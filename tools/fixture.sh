#!/usr/bin/env bash
# The full clip -> build -> assert path against the committed fixture, so CI
# exercises it without downloading the 1.6 GB India extract. tools/data.sh
# stays a local manual step.
set -euo pipefail
cd "$(dirname "$0")/.."

WORK=$(mktemp -d)
trap 'rm -rf "$WORK"' EXIT

python -c "import osmium" 2>/dev/null || python -m pip install --quiet osmium

# Inset from the fixture bounds, so the clip actually has something to cut and
# the complete_ways back-fill has nodes to reach for.
python tools/clip.py tests/fixtures/sector17.osm.pbf "$WORK/clip.osm.pbf" 76.772,30.732,76.793,30.748
cargo run -p graph --release -- build --pbf "$WORK/clip.osm.pbf" --out "$WORK/graph.bin"
echo "fixture clip -> build -> assert: ok"
