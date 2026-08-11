# Navigation

An offline OSM map panel drawn to the right of the inference view: the road
network around the car, the route to a destination, the next manoeuvre, and the
speed limit of the road under the wheels.

```
┌──────────────────────────┬──────────────┐
│                          │  120 m       │
│   inference HUD          │  Turn left   │
│   1024 x 512             ├──────────────┤
│                          │   map        │
│   path, CIPO, alerts     │   route      │
│                          │   ego        │
│                          ├──────────────┤
│                          │ Jl. Braga 40 │
└──────────────────────────┴──────────────┘
```

Everything is local. There is no Overpass client, no tile server and no network
at drive time — the whole of greater Bandung is one 10 MB file, read once into
memory at startup.

## Quick start

```bash
./scripts/build/build_osm_map.sh                 # once: builds maps/bandung-roads.vpmap
./scripts/run/inference_cam.sh --nav --gps-url http://172.27.74.24:8080
./scripts/run/inference_cam.sh --nav --dest -6.9034,107.6181 --dest-name "Gedung Sate"
```

Check the map on its own, with no phone, camera or GPU:

```bash
cd build
./nav_preview --map ../maps/bandung-roads.vpmap --at -6.9034,107.6181 \
              --dest -6.8890,107.6100 -o /tmp/nav.png
```

## Where position comes from

Set `nav.gps_source` in `config/vision_pilot.conf`:

| source | what it reads | needs |
| --- | --- | --- |
| `phyphox` | phyphox "Location (GPS)" experiment, `GET /get?lat&lon&z&v&dir&accuracy` | `nav.gps_url` |
| `ipwebcam` | IP Webcam sensor log, `GET /sensors.json?sense=gps` | `nav.gps_url`, or the camera stream URL |
| `file` | recorded track, one `lat,lon[,speed_ms[,bearing_deg]]` per line | `nav.gps_file` |
| `nucleo` | Nucleo-G474RE on USB: `$VPFUS` / `$VPRTK` lines from the DFRobot GNSS-RTK rover, GPS/IMU fused on the board | `nav.gps_port` (empty = first `/dev/ttyACM*`) |
| `fixed` | one static coordinate — exercises the map with no receiver | `nav.fixed_position` |
| `none` | nothing; the panel says so | |

**`nucleo` is the one that does not need a phone at all.** A Nucleo-G474RE reads
the DFRobot GNSS-RTK rover over I2C and a Yahboom IMU over UART, fuses them on
the board, and publishes 20 Hz positions over its ST-LINK port — so the position
arrives already smoothed, with a heading that survives a stop. The firmware and
its wiring are in
[modules/sensing/gps_rtk_dfrobot](../sensing/gps_rtk_dfrobot/); the run script is
`scripts/run/inference_cam_nav_gps.sh`. Leave `nav.imu_enabled` off with it: the
board has already fused these two, and filtering a filtered position lags rather
than averaging out.

**phyphox is the one that works on the phones tested here.** On the phone: open
the *Location (GPS)* experiment, three-dot menu → *Allow remote access*, press
play. It serves on port 8080, so if it shares a phone with IP Webcam, move one of
them (IP Webcam: Connection settings → Local broadcasting → Port).

**IP Webcam is only viable on builds that list GPS under Data logging → Active
data logging sources.** Several builds do not — the list ends at *Rotation
vector* — and on those `/sensors.json?sense=gps` answers `{}` for ever, no matter
what location permissions are granted. Two further traps on the builds that do
support it: the sensor is off until `GET /settings/gps_active?set=on` (VisionPilot
issues this itself at startup), and *Sensor data retention* defaults to 5 seconds,
which is shorter than the interval between GPS fixes indoors — so raise it.

A source that cannot work degrades the panel, never the drive: VisionPilot logs
the reason, draws it on the panel, and keeps driving.

## The map file

`scripts/build/build_osm_map.sh` clips a Geofabrik-style regional extract to greater
Bandung, drops everything a car cannot drive on, and converts the result to
`.vpmap` — a flat binary with the geometry, the topology and the speed limits
already resolved:

```
char   magic[8]   "VPMAP01\n"
int32  min_lat_e7, min_lon_e7, max_lat_e7, max_lon_e7
uint32 node_count, way_count, name_count
nodes  node_count x (int32 lat_e7, int32 lon_e7)
names  name_count x (uint16 len, bytes[len])
ways   way_count  x (uint8 class, uint8 flags, uint16 speed_kph,
                     uint32 name_idx, uint32 npts, uint32 nodes[npts])
```

Coordinates are integer 1e-7 degrees, which is also the precision `osmium export`
prints — so nodes shared between ways land on identical integers and the routing
graph reconnects without needing OSM node IDs. Greater Bandung comes to 102,883
ways / 693,023 nodes / 9.9 MB.

The conversion needs only `osmium-tool` and a stdlib Python; pyosmium is not
required.

## Inside

| file | what it does |
| --- | --- |
| `osm_map.*` | loads `.vpmap`, 500 m grid index, map matching, routing graph |
| `router.*` | Dijkstra on travel time, turn-by-turn steps, progress tracking |
| `map_renderer.*` | draws the panel: roads, route, ego, banner, footer |
| `gps_sources.*` | phyphox / IP Webcam / Nucleo serial / file / fixed, each on its own thread |
| `http_get.*` | ~150-line HTTP GET over a raw socket — no libcurl dependency |
| `navigation.*` | `Navigator`: owns the above, `update()` per frame, `compose()` |

Costs on the Bandung extract: load + graph build ~0.3 s once, map match well
under a millisecond, a cross-city Dijkstra a few tens of milliseconds. Routing
runs on a worker thread so it never lands in the inference loop; `update()` and
`compose()` are the only things the loop calls.

Map matching widens with the fix's own accuracy (60–200 m): a phone reporting
±92 m indoors would otherwise match no road at all, and "not sure which road" is
a different statement from "no road".

## Notes

- OpenCV's Hershey fonts are ASCII only, so road names are folded before drawing
  (`Jalan Ir. H. Djuanda` survives; accents degrade rather than turn into `?`).
- The route is recomputed only when the car has genuinely left it — four
  consecutive fixes more than 50 m off the line, with an 8 s cooldown.
- Speed limits are shown dimmed when they are a per-class default rather than an
  OSM `maxspeed` tag. The planner does not read them; they are display only.
