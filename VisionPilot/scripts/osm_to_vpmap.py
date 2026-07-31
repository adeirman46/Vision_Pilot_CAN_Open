#!/usr/bin/env python3
"""Convert an OSM road extract into VisionPilot's .vpmap binary.

VisionPilot has no libosmium and no Overpass client: the navigation module reads
one flat file with the road geometry, the topology and the speed limits already
resolved. This is what builds it, offline, once per area.

Geometry comes out of osmium-tool as GeoJSON text sequences, so the only
dependency is `osmium` on PATH — pyosmium is not needed. Coordinates are printed
with 7 decimals, which is exactly the resolution the file stores, so nodes shared
between ways still land on identical integers and the routing graph reconnects.

  ./osm_to_vpmap.py bandung-roads.osm.pbf -o ../maps/bandung-roads.vpmap

File layout (little-endian):

  char   magic[8]     "VPMAP01\\n"
  int32  min_lat_e7, min_lon_e7, max_lat_e7, max_lon_e7
  uint32 node_count, way_count, name_count
  nodes  node_count x (int32 lat_e7, int32 lon_e7)
  names  name_count x (uint16 len, bytes[len])        # index 0 is ""
  ways   way_count  x (uint8 road_class, uint8 flags, uint16 speed_kph,
                       uint32 name_idx, uint32 point_count,
                       uint32 node_index[point_count])
  uint32 place_count
  places place_count x (int32 lat_e7, int32 lon_e7,
                        uint16 name_len, bytes name, uint8 kind_len, bytes kind)

Places are the named destinations a driver actually asks for — "Trans Studio
Mall", not "Jalan Gatot Subroto". Roads alone cannot answer that.
"""
from __future__ import annotations

import argparse
import json
import struct
import subprocess
import sys
from pathlib import Path

MAGIC = b"VPMAP02\n"
MAGIC_V1 = b"VPMAP01\n"   # roads only, no places table

# Must match navigation::RoadClass in modules/navigation/include/navigation/osm_map.hpp
ROAD_CLASS = {
    "motorway": 1,
    "trunk": 2,
    "primary": 3,
    "secondary": 4,
    "tertiary": 5,
    "residential": 6,
    "unclassified": 7,
    "living_street": 8,
    "service": 9,
}

FLAG_ONEWAY_FWD = 1 << 0
FLAG_ONEWAY_REV = 1 << 1
FLAG_SPEED_INFERRED = 1 << 2
FLAG_LINK = 1 << 3

# Fallback limits, km/h. Indonesian urban defaults (UU 22/2009 and the usual
# Bandung signage) rather than OSM's global guesses, since most ways in the
# extract carry no maxspeed at all.
DEFAULT_SPEED = {
    1: 80,   # motorway  — jalan tol
    2: 60,   # trunk
    3: 50,   # primary
    4: 40,   # secondary
    5: 40,   # tertiary
    6: 30,   # residential
    7: 40,   # unclassified
    8: 20,   # living_street
    9: 15,   # service - yard/complex access roads
    0: 40,
}

MPH_TO_KPH = 1.609344

# Tags worth keeping as a destination. Deliberately narrow: every named bench and
# waste basket in OSM would bury the useful answers in the search results.
PLACE_TAGS = {
    "shop": {"mall", "department_store", "supermarket", "wholesale"},
    "amenity": {"hospital", "clinic", "university", "college", "school",
                "bus_station", "fuel", "marketplace", "place_of_worship",
                "townhall", "police", "courthouse", "library", "cinema",
                "parking", "restaurant", "cafe"},
    "tourism": {"hotel", "attraction", "theme_park", "museum", "zoo",
                "guest_house", "hostel", "motel", "viewpoint"},
    "leisure": {"park", "stadium", "sports_centre", "golf_course"},
    "aeroway": {"terminal", "aerodrome"},
    "railway": {"station", "halt"},
    "office": {"government", "company"},
    "building": {"retail", "commercial", "train_station"},
    "landuse": {"retail", "industrial"},
    "place": {"suburb", "neighbourhood", "village", "town", "city", "quarter"},
}


def place_kind(props: dict) -> str | None:
    """The most specific whitelisted tag on this feature, or None to skip it."""
    for key, values in PLACE_TAGS.items():
        v = props.get(key)
        if isinstance(v, str) and v in values:
            return v
    return None


def centroid(geom: dict):
    """Representative point for a POI: nodes as-is, areas/ways averaged."""
    t = geom.get("type")
    c = geom.get("coordinates")
    if not c:
        return None
    if t == "Point":
        return c[0], c[1]
    pts = []

    def walk(x):
        if (isinstance(x, list) and len(x) >= 2
                and all(isinstance(v, (int, float)) for v in x[:2])):
            pts.append((x[0], x[1]))
        elif isinstance(x, list):
            for y in x:
                walk(y)

    walk(c)
    if not pts:
        return None
    return sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts)


def stream_places(pbf: Path):
    """Yield (name, kind, lat, lon) for every named, whitelisted feature."""
    cmd = ["osmium", "export", str(pbf), "-f", "geojsonseq",
           "-x", "print_record_separator=false"]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.stdout is not None
    seen: set[tuple[str, int, int]] = set()
    for line in proc.stdout:
        line = line.strip().lstrip("\x1e")
        if not line:
            continue
        try:
            feat = json.loads(line)
        except json.JSONDecodeError:
            continue
        props = feat.get("properties") or {}
        name = props.get("name")
        if not isinstance(name, str) or not name.strip():
            continue
        kind = place_kind(props)
        if kind is None:
            continue
        pt = centroid(feat.get("geometry") or {})
        if pt is None:
            continue
        lon, lat = pt
        # OSM maps a mall as a node AND a building AND a landuse; one entry each
        # would fill the search results with the same place three times.
        key = (name.lower(), int(round(lat * 1e4)), int(round(lon * 1e4)))
        if key in seen:
            continue
        seen.add(key)
        yield name[:120], kind[:32], lat, lon
    proc.stdout.close()
    proc.wait()


def parse_maxspeed(raw: str | None) -> int:
    """OSM maxspeed → km/h, or 0 when it says nothing usable."""
    if not raw:
        return 0
    s = raw.strip().lower()
    if s in ("none", "signals", "variable", "walk"):
        return 20 if s == "walk" else 0
    mult = 1.0
    if "mph" in s:
        mult = MPH_TO_KPH
        s = s.replace("mph", "")
    s = s.replace("km/h", "").replace("kph", "").strip()
    # "ID:urban", "RU:rural" and friends carry no number
    try:
        return int(round(float(s.split(";")[0].split("-")[0]) * mult))
    except ValueError:
        return 0


def classify(highway: str) -> tuple[int, bool]:
    """highway tag → (road class, is_link)."""
    link = highway.endswith("_link")
    base = highway[: -len("_link")] if link else highway
    return ROAD_CLASS.get(base, 0), link


def oneway_flags(tags: dict) -> int:
    val = str(tags.get("oneway", "")).strip().lower()
    if val in ("yes", "true", "1"):
        return FLAG_ONEWAY_FWD
    if val in ("-1", "reverse"):
        return FLAG_ONEWAY_REV
    # Motorway carriageways and slip roads are one-way by definition unless
    # tagged otherwise, and OSM leaves that implicit.
    if val in ("no", "false", "0"):
        return 0
    if tags.get("highway") in ("motorway", "motorway_link"):
        return FLAG_ONEWAY_FWD
    if tags.get("junction") in ("roundabout", "circular"):
        return FLAG_ONEWAY_FWD
    return 0


def stream_features(pbf: Path):
    """Yield (properties, coordinates) for every road linestring in the extract."""
    cmd = [
        "osmium", "export", str(pbf),
        "-f", "geojsonseq",
        "--geometry-types=linestring",
        "-x", "print_record_separator=false",
    ]
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    assert proc.stdout is not None
    for line in proc.stdout:
        line = line.strip().lstrip("\x1e")
        if not line:
            continue
        try:
            feat = json.loads(line)
        except json.JSONDecodeError:
            continue
        geom = feat.get("geometry") or {}
        if geom.get("type") != "LineString":
            continue
        yield feat.get("properties") or {}, geom.get("coordinates") or []
    proc.stdout.close()
    rc = proc.wait()
    if rc != 0:
        err = proc.stderr.read() if proc.stderr else ""
        raise SystemExit(f"osmium export failed ({rc}): {err.strip()}")


def build(pbf: Path, out: Path, keep_unclassified: bool,
          places_pbf: Path | None = None) -> None:
    nodes: dict[tuple[int, int], int] = {}
    node_list: list[tuple[int, int]] = []
    names: dict[str, int] = {"": 0}
    name_list: list[str] = [""]
    ways: list[tuple[int, int, int, int, list[int]]] = []

    kept = skipped = 0
    for props, coords in stream_features(pbf):
        highway = props.get("highway")
        if not highway or len(coords) < 2:
            skipped += 1
            continue

        road_class, is_link = classify(highway)
        if road_class == 0 and not keep_unclassified:
            skipped += 1
            continue

        speed = parse_maxspeed(props.get("maxspeed"))
        flags = oneway_flags(props)
        if is_link:
            flags |= FLAG_LINK
            # A slip road inherits the direction of the road it feeds.
            flags |= FLAG_ONEWAY_FWD
        if speed == 0:
            speed = DEFAULT_SPEED.get(road_class, 40)
            flags |= FLAG_SPEED_INFERRED

        name = props.get("name") or props.get("ref") or ""
        if isinstance(name, str) and name:
            name = name[:120]
        else:
            name = ""
        name_idx = names.get(name)
        if name_idx is None:
            name_idx = len(name_list)
            names[name] = name_idx
            name_list.append(name)

        idx_list: list[int] = []
        for lon, lat, *_ in coords:
            key = (int(round(lat * 1e7)), int(round(lon * 1e7)))
            ni = nodes.get(key)
            if ni is None:
                ni = len(node_list)
                nodes[key] = ni
                node_list.append(key)
            # Collapse repeated coordinates: they would be zero-length edges.
            if idx_list and idx_list[-1] == ni:
                continue
            idx_list.append(ni)

        if len(idx_list) < 2:
            skipped += 1
            continue

        ways.append((road_class, flags, min(speed, 65535), name_idx, idx_list))
        kept += 1

    if not ways:
        raise SystemExit(f"{pbf}: no drivable ways found — is this a road extract?")

    places: list[tuple[str, str, float, float]] = []
    if places_pbf is not None:
        places = list(stream_places(places_pbf))

    lats = [n[0] for n in node_list]
    lons = [n[1] for n in node_list]

    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open("wb") as f:
        f.write(MAGIC)
        f.write(struct.pack("<4i", min(lats), min(lons), max(lats), max(lons)))
        f.write(struct.pack("<3I", len(node_list), len(ways), len(name_list)))

        node_buf = bytearray()
        for lat_e7, lon_e7 in node_list:
            node_buf += struct.pack("<2i", lat_e7, lon_e7)
        f.write(node_buf)

        for name in name_list:
            raw = name.encode("utf-8")[:65535]
            f.write(struct.pack("<H", len(raw)))
            f.write(raw)

        for road_class, flags, speed, name_idx, idx_list in ways:
            f.write(struct.pack("<BBHII", road_class, flags, speed, name_idx, len(idx_list)))
            f.write(struct.pack(f"<{len(idx_list)}I", *idx_list))

        f.write(struct.pack("<I", len(places)))
        for name, kind, lat, lon in places:
            raw_name = name.encode("utf-8")[:65535]
            raw_kind = kind.encode("utf-8")[:255]
            f.write(struct.pack("<2i", int(round(lat * 1e7)), int(round(lon * 1e7))))
            f.write(struct.pack("<H", len(raw_name)))
            f.write(raw_name)
            f.write(struct.pack("<B", len(raw_kind)))
            f.write(raw_kind)

    size_mb = out.stat().st_size / 1e6
    print(f"wrote {out}")
    print(f"  {kept} ways, {len(node_list)} nodes, {len(name_list)} names, "
          f"{len(places)} places, {skipped} skipped, {size_mb:.1f} MB")
    print(f"  bbox {min(lats)/1e7:.5f},{min(lons)/1e7:.5f} .. "
          f"{max(lats)/1e7:.5f},{max(lons)/1e7:.5f}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("pbf", type=Path, help="road-filtered .osm.pbf")
    ap.add_argument("-o", "--output", type=Path, required=True, help="destination .vpmap")
    ap.add_argument("--keep-unclassified", action="store_true",
                    help="also keep highway types VisionPilot has no class for")
    ap.add_argument("--places", type=Path, default=None,
                    help="extract to mine named destinations (malls, hospitals, "
                         "stations) from; usually the unfiltered area .osm.pbf")
    args = ap.parse_args()

    if not args.pbf.is_file():
        raise SystemExit(f"not found: {args.pbf}")
    if args.places is not None and not args.places.is_file():
        raise SystemExit(f"not found: {args.places}")
    build(args.pbf, args.output, args.keep_unclassified, args.places)
    return 0


if __name__ == "__main__":
    sys.exit(main())
