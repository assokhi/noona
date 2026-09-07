#!/usr/bin/env bash
# Extract the Chandigarh basemap from the Protomaps global daily build.
#
# The source archive is ~138 GB and is never downloaded: pmtiles extract pulls
# only the tiles inside the bbox over HTTP range requests. This one took 41
# requests and 4.7 MB of transfer.
set -euo pipefail
cd "$(dirname "$0")/.."

BBOX="76.65,30.65,76.87,30.80"
OUT="web/public/chandigarh.pmtiles"
PM="tools/bin/pmtiles.exe"
[ -x "$PM" ] || PM="pmtiles"

mkdir -p web/public tools/bin
if [ ! -x "tools/bin/pmtiles.exe" ] && ! command -v pmtiles >/dev/null; then
  echo "install the pmtiles CLI from https://github.com/protomaps/go-pmtiles/releases" >&2
  exit 1
fi

# Daily builds expire, so walk back from today until one answers.
BUILD=""
for i in $(seq 0 10); do
  D=$(date -u -d "-$i day" +%Y%m%d 2>/dev/null || date -u -v-"${i}"d +%Y%m%d)
  if curl -sS -I --max-time 30 "https://build.protomaps.com/$D.pmtiles" | grep -q "200 OK"; then
    BUILD="$D"
    break
  fi
done
[ -n "$BUILD" ] || { echo "no recent protomaps build found" >&2; exit 1; }

echo "extracting $BBOX from build $BUILD"
"$PM" extract "https://build.protomaps.com/$BUILD.pmtiles" "$OUT" \
  --bbox="$BBOX" --maxzoom=15 --download-threads=8
ls -l "$OUT"
