#!/usr/bin/env bash
# Phase 0: regenerate the Chandigarh extract from scratch, reproducibly.
#
# Chandigarh UT plus a margin into Mohali and Panchkula, so routes near the
# border do not dead-end at a clipped graph edge. Do not clip tighter.
#
# Two stages, because neither tool alone does the job on Windows:
#   1. osmconvert cuts a wide coarse box out of the 1.6 GB India extract fast.
#      Its --complete-ways is broken in the Windows build (silently writes an
#      empty file), so this stage only does the cheap cut.
#   2. tools/clip.py cuts the exact bbox with complete_ways semantics, which is
#      the part that actually matters. See that file.
set -euo pipefail
cd "$(dirname "$0")/.."

BBOX="76.65,30.65,76.87,30.80"     # west,south,east,north - the real extract
WIDE="75.5,30.0,78.0,31.5"         # coarse cut, about 140 km of slack, so the
                                   # complete_ways back-fill never runs out of
                                   # nodes at the edge of stage 1
URL="https://download.geofabrik.de/asia/india-latest.osm.pbf"
SRC="data/raw/india-latest.osm.pbf"
TMP="data/raw/_wide.osm.pbf"
OUT="data/raw/chandigarh.osm.pbf"
OSMCONVERT="tools/bin/osmconvert.exe"
[ -x "$OSMCONVERT" ] || OSMCONVERT="osmconvert"   # non-Windows

mkdir -p data/raw data/build tools/bin

if [ ! -x "$OSMCONVERT" ] && ! command -v osmconvert >/dev/null; then
  echo "fetching osmconvert"
  curl -sSL --retry 5 -o tools/bin/osmconvert.exe http://m.m.i24.cc/osmconvert64.exe
  chmod +x tools/bin/osmconvert.exe
  OSMCONVERT="tools/bin/osmconvert.exe"
fi
python -c "import osmium" 2>/dev/null || python -m pip install --quiet osmium

curl -sSL --retry 5 --retry-all-errors -o "$SRC.md5" "$URL.md5"
if [ ! -f "$SRC" ]; then
  echo "downloading $URL (about 1.6 GB)"
  curl -L --retry 5 --retry-all-errors -C - -o "$SRC" "$URL"
fi
( cd data/raw && md5sum -c "$(basename "$SRC").md5" )

echo "stage 1: coarse cut to $WIDE"
"$OSMCONVERT" "$SRC" -b="$WIDE" --out-pbf -o="$TMP"

echo "stage 2: exact cut to $BBOX with complete_ways"
python tools/clip.py "$TMP" "$OUT" "$BBOX"
rm -f "$TMP"

# Fail loudly rather than leave a useless extract lying around.
STATS=$("$OSMCONVERT" "$OUT" --out-statistics)
NODES=$(echo "$STATS" | awk -F': ' '/^nodes:/ {print $2}')
[ "${NODES:-0}" -gt 100000 ] || { echo "clip produced only ${NODES:-0} nodes - refusing" >&2; exit 1; }

{
  echo "# PROVENANCE - regenerate with: make data"
  echo "generated_utc:   $(date -u +%Y-%m-%dT%H:%M:%SZ)"
  echo "source_url:      $URL"
  echo "source_md5:      $(cut -d' ' -f1 < "$SRC.md5")"
  echo "source_bytes:    $(wc -c < "$SRC" | tr -d ' ')"
  echo "coarse_bbox:     $WIDE   (osmconvert -b)"
  echo "clip_bbox:       $BBOX   (tools/clip.py, complete_ways)"
  echo "relations:       dropped - Phase 8 turn restrictions will need a re-clip"
  echo "extract_bytes:   $(wc -c < "$OUT" | tr -d ' ')"
  echo "extract_md5:     $(md5sum "$OUT" | cut -d' ' -f1)"
  echo
  echo "# osmconvert --out-statistics $OUT"
  echo "$STATS"
} > data/raw/PROVENANCE

cat data/raw/PROVENANCE
SIZE_MB=$(( $(wc -c < "$OUT") / 1000000 ))
echo
echo "extract is ${SIZE_MB} MB"
[ "$SIZE_MB" -lt 80 ] || echo "warning: extract is larger than expected - check the bbox"
