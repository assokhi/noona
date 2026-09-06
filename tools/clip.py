#!/usr/bin/env python3
"""Bbox extract with complete_ways semantics.

osmium-tool has no Windows build and osmconvert's --complete-ways silently
emits an empty file here (its two-pass tempfile path is broken in that build),
so the clip runs through pyosmium instead. BackReferenceWriter does the work:
we write every node inside the box and every way with at least one node inside,
and it back-fills the referenced nodes that fell outside. Without that, ways get
truncated at the bbox edge and every arterial leaving the city dead-ends.

usage: clip.py <in.osm.pbf> <out.osm.pbf> <west,south,east,north>
"""
import sys

import osmium

WEST, SOUTH, EAST, NORTH = 0, 1, 2, 3


def main(src: str, out: str, bbox: str) -> int:
    b = [float(x) for x in bbox.split(",")]
    if len(b) != 4 or b[WEST] >= b[EAST] or b[SOUTH] >= b[NORTH]:
        sys.exit(f"bad bbox {bbox!r}, want west,south,east,north")

    def inside(loc) -> bool:
        # loc is (lon, lat); Chandigarh is (76.7, 30.7) and both numbers look
        # plausible either way round, so a swap would not crash - it would just
        # silently produce an empty extract.
        return (
            loc.valid()
            and b[WEST] <= loc.lon <= b[EAST]
            and b[SOUTH] <= loc.lat <= b[NORTH]
        )

    nodes = ways = 0
    # remove_tags=False: back-filled nodes keep highway=traffic_signals and
    # barrier=*, which graph construction needs to force intersection nodes.
    # ponytail: relations are dropped (relation_depth=0). Phase 8 turn
    # restrictions will need them - raise the depth and re-clip then.
    with osmium.BackReferenceWriter(out, ref_src=src, overwrite=True, remove_tags=False) as w:
        for obj in osmium.FileProcessor(src).with_locations():
            if obj.is_node():
                if inside(obj.location):
                    w.add_node(obj)
                    nodes += 1
            elif obj.is_way():
                if any(inside(n.location) for n in obj.nodes):
                    w.add_way(obj)
                    ways += 1

    print(f"wrote {nodes} nodes and {ways} ways inside {bbox}")
    if nodes == 0:
        sys.exit("clip selected nothing - check the bbox order (west,south,east,north)")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 4:
        sys.exit(__doc__)
    sys.exit(main(*sys.argv[1:]))
