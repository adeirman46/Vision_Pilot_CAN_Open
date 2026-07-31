#!/usr/bin/env bash
#
# Build the offline OSM road map the navigation panel draws — greater Bandung.
#
# Usage:
#   ./build_osm_map.sh                       # greater Bandung (default)
#   ./build_osm_map.sh --bbox 107.5,-7.0,107.7,-6.85
#   ./build_osm_map.sh --output ~/maps/bandung.vpmap
#
# Options:
#   --bbox <l,b,r,t>   lon,lat,lon,lat clip box. Default greater Bandung:
#                      Cimahi in the west through the city to Bandung regency.
#   --region <name>    source extract on download.openstreetmap.fr, under
#                      asia/indonesia. Default west_java (~344 MB, cached).
#   --output <file>    where the .vpmap goes. Default VisionPilot/maps/bandung-roads.vpmap
#   --src <file>       use this .osm.pbf instead of downloading one
#   --keep-intermediate  leave the clipped/filtered .pbf files behind
#   --force            rebuild even when the .vpmap is newer than the source
#
# Downloads once into $VP_OSM_CACHE (default ~/openpilot_maps/src), so a machine
# that already built openpilot's mapd extract does not fetch West Java twice.
#
# Requires: osmium-tool (sudo apt-get install osmium-tool), curl, python3.
#
# Env overrides: VP_ROOT, VP_OSM_CACHE
set -euo pipefail

VP_ROOT="${VP_ROOT:-$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)}"
CACHE="${VP_OSM_CACHE:-$HOME/openpilot_maps/src}"

# left,bottom,right,top (lon,lat,lon,lat) — same box openpilot's mapd uses.
BBOX="107.40,-7.10,107.80,-6.75"
REGION="west_java"
OUTPUT="$VP_ROOT/maps/bandung-roads.vpmap"
SRC=""
KEEP="false"
FORCE="false"

usage() { sed -n '2,26p' "$0"; exit "${1:-1}"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --bbox)    [ $# -ge 2 ] || { echo "error: --bbox needs a value" >&2; exit 1; }
                   BBOX="$2"; shift 2 ;;
        --region)  [ $# -ge 2 ] || { echo "error: --region needs a value" >&2; exit 1; }
                   REGION="$2"; shift 2 ;;
        --output)  [ $# -ge 2 ] || { echo "error: --output needs a value" >&2; exit 1; }
                   OUTPUT="$2"; shift 2 ;;
        --src)     [ $# -ge 2 ] || { echo "error: --src needs a value" >&2; exit 1; }
                   SRC="$(readlink -f "$2")"; shift 2 ;;
        --keep-intermediate) KEEP="true"; shift ;;
        --force)   FORCE="true"; shift ;;
        -h|--help) usage 0 ;;
        *)         echo "error: unknown option '$1'" >&2; usage ;;
    esac
done

command -v osmium >/dev/null 2>&1 || {
    echo "error: osmium-tool is required" >&2
    echo "       sudo apt-get install -y osmium-tool" >&2
    exit 1; }

# The converter is stdlib-only; use the system interpreter rather than whatever
# conda env happens to be first on PATH.
PYTHON="/usr/bin/python3"
command -v "$PYTHON" >/dev/null 2>&1 || PYTHON="python3"

case "$BBOX" in
    *,*,*,*) ;;
    *) echo "error: --bbox must be lon,lat,lon,lat" >&2; exit 1 ;;
esac

WORK="$(dirname "$OUTPUT")/work"
mkdir -p "$CACHE" "$WORK" "$(dirname "$OUTPUT")"

# ── Source extract ───────────────────────────────────────────────────────────
if [ -z "$SRC" ]; then
    SRC="$CACHE/${REGION}-latest.osm.pbf"
    if [ ! -f "$SRC" ]; then
        URL="https://download.openstreetmap.fr/extracts/asia/indonesia/${REGION}-latest.osm.pbf"
        echo "downloading $REGION extract (~344 MB, cached in $CACHE)"
        # -C - resumes a half-finished download instead of starting over.
        curl -L --retry 5 --retry-delay 5 -C - -o "$SRC.part" "$URL"
        mv "$SRC.part" "$SRC"
    else
        echo "using cached source $SRC"
    fi
fi
[ -f "$SRC" ] || { echo "error: source extract not found: $SRC" >&2; exit 1; }

if [ "$FORCE" = "false" ] && [ -f "$OUTPUT" ] && [ "$OUTPUT" -nt "$SRC" ]; then
    echo "$OUTPUT is newer than $SRC — nothing to do (use --force to rebuild)"
    ls -lh "$OUTPUT"
    exit 0
fi

# ── Clip to Bandung ──────────────────────────────────────────────────────────
# complete_ways keeps a road whole when it merely crosses the box, so the
# geometry does not stop dead at the border.
echo "clipping bbox $BBOX ..."
osmium extract -b "$BBOX" --strategy=complete_ways "$SRC" \
    -o "$WORK/area.osm.pbf" --overwrite

# ── Roads only ───────────────────────────────────────────────────────────────
# Same highway set openpilot's mapd queries: everything a car can drive on, and
# nothing else. This is what takes greater Bandung from ~57 MB to ~12 MB.
echo "keeping only highways ..."
osmium tags-filter "$WORK/area.osm.pbf" w/highway \
    -o "$WORK/area-highways.osm.pbf" --overwrite

# `service` is deliberately KEPT, unlike openpilot's mapd filter. Its yard and
# complex access roads are the only roads inside a factory site or a housing
# estate, and dropping them leaves the map blank exactly where the car is.
echo "dropping non-road highway types ..."
osmium tags-filter -i "$WORK/area-highways.osm.pbf" \
    w/highway=footway,path,corridor,bridleway,steps,cycleway,construction,bus_guideway,escape,track \
    -o "$WORK/roads.osm.pbf" --overwrite

# ── Convert ──────────────────────────────────────────────────────────────────
# Places come from the *unfiltered* clip: a mall is not a highway, so it is not
# in roads.osm.pbf at all, and "drive me to Trans Studio Mall" needs it.
echo "converting to $OUTPUT ..."
"$PYTHON" "$VP_ROOT/scripts/osm_to_vpmap.py" "$WORK/roads.osm.pbf" \
    --places "$WORK/area.osm.pbf" -o "$OUTPUT"

if [ "$KEEP" = "false" ]; then
    rm -f "$WORK/area.osm.pbf" "$WORK/area-highways.osm.pbf" "$WORK/roads.osm.pbf"
    rmdir "$WORK" 2>/dev/null || true
fi

echo ""
echo "=================================================="
echo " map ready"
ls -lh "$OUTPUT"
echo ""
echo " enable it in VisionPilot/config/vision_pilot.conf:"
echo "   nav.enabled  = true"
echo "   nav.map_file = $OUTPUT"
echo " or run:  ./scripts/inference_cam.sh --nav"
echo "=================================================="
