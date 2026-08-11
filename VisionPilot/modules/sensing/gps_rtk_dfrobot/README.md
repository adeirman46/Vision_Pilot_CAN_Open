# DFRobot GNSS-RTK on STM32 G474RE

Centimetre-level position from the DFRobot GNSS-RTK kit (KIT0198), read by a
Nucleo-G474RE and published to the host PC as checksummed lines over the
ST-LINK serial port.

This replaces the phone as a position source. phyphox on a handset is metre-
level at best and drifts under trees; this kit is centimetre-level once it
reaches RTK Fixed, which is the difference between knowing which road the car
is on and knowing which lane.

```bash
../../../../scripts/sensors/flash_nucleo.sh fused    # GPS + IMU, fused on the board
../../../../scripts/sensors/gps_imu_check.sh         # is it publishing? is the IMU in?
```

or, straight from here:

```bash
pio run -e gps_imu_ekf -t upload   # fused. See "Which firmware" below
pio device monitor                 # watch it publish, 115200 baud
```

## Which firmware

Each one is a folder beside an empty `src/`, and a PlatformIO env that reaches
into it — so no two can drift into each other's builds.

| flash it with | env / folder | what it does |
|---|---|---|
| `flash_nucleo.sh fused` | `gps_imu_ekf/` | **GPS + IMU fused on the board.** Rover on I2C at 1 Hz, Yahboom MRA02 IMU on USART1 at 25 Hz, through the same 6-state EKF the host runs. Publishes `$VPFUS` at 20 Hz and the raw `$VPRTK` at 1 Hz. Needs no RTK base. |
| `flash_nucleo.sh gps` | `gps_1hz/` | GPS only, 1 Hz, RTK optional. The baseline to compare the fused track against, and the fallback when the IMU will not talk. |
| `flash_nucleo.sh gps10` | `10hz_i2c/` | GPS only, 10 Hz. The original. Worth flashing once an RTK base is up — at centimetre accuracy there is something to gain from ten fixes a second. |
| `flash_nucleo.sh sniff` | `diag_imu_uart/` | Where is the IMU? Sweeps four UART pin pairs × six baud rates. |
| `flash_nucleo.sh probe` | `10hz_i2c/diag_baud_scan.cpp` | Where is the GPS? For `no-device`. |
| `flash_nucleo.sh rate` | `10hz_i2c/diag_rate.cpp` | Why the rover ignores `$PAIR050`. |
| `flash_nucleo.sh rtklink` | `10hz_i2c/diag_rtk_link.cpp` | Why no corrections arrive. |
| `flash_nucleo.sh legacy` | `legacy_1hz/` | The original UART-wired build. |

**Why 1 Hz in the fused build.** The 10 Hz build asks for a 100 ms fix interval
and then measures `rate=1.0hz` anyway: this kit does not sustain 10 Hz without
RTK. Asking for a rate the hardware will not deliver buys nothing and makes the
measured rate line useless as a health check — at 1 Hz it reads 1.0, and any
departure is real. Position between fixes is the IMU's job now, which is the
whole point of fusing them.

## GPS + IMU on the board

Both sensors are wired to the Nucleo, so that is where they are fused. The host
has the same filter (`navigation::FusedImuGps`), and it works — but only when
the IMU is plugged into the *PC*. Here the two streams are already together, and
the timestamps are honest: `dt` between IMU samples is measured by the MCU that
received them, not inferred after a USB round trip and a scheduler delay.

What each sensor contributes, and why neither is enough:

| | GPS | IMU |
|---|---|---|
| rate | 1 Hz | 25 Hz |
| absolute position | yes | no |
| error | a couple of metres, but bounded | integrates without limit |
| heading when stopped | meaningless | held |

The state is position north/east, velocity north/east, heading, and the gyro's
yaw-rate bias. GPS anchors the absolute position and bounds the drift; the IMU
fills in the second between fixes and carries the heading through a stop, a
tunnel, or a row of tall buildings. The filter is a port of
[gps_imu_ekf.cpp](../../navigation/src/gps_imu_ekf.cpp), kept line-for-line
identical on purpose: the host copy has a test that drives it along a known
route and measures the error, and that test is the only evidence either copy
works.

### IMU wiring

The GPS is on I2C, which frees USART1 — the pins the old UART GPS build used.

| Yahboom MRA02 | Wire | Nucleo-G474RE | Arduino pin |
|---|---|---|---|
| GND | black | GND | — |
| VCC (3.3–5 V) | red | 5V | — |
| TX (module out) | | **PA10** | D2 |
| RX (module in) | | PA9 | D8 |

**This pair crosses over** — the module's TX to the Nucleo's RX — unlike the GPS
wiring beside it. Only PA10 is required: nothing ever commands the IMU, it just
streams. The ground must be common with the Nucleo's; a module powered from
somewhere else with no shared ground transmits into nothing.

If the firmware reports `imu-silent`, the sniffer answers the question before
the baud question — *do any bytes reach this board at all, on any pin?*

```
../../../../scripts/sensors/flash_nucleo.sh sniff -m
```

`bytes=0` is power, ground, or the pair not crossed. Bytes but no frames is a
baud or protocol problem, and the sniffer hex-dumps what arrived so you can tell
which.

### The two constants to check on the first drive

Everything about how the module is bolted in lives in two named constants in
[gps_imu_ekf/main.cpp](gps_imu_ekf/main.cpp), because both are wrong *quietly*:

* **`kYawRateSign`** (default −1). The MRA02's gyro is right-handed about a z
  axis that points up, so a right turn reads negative; the filter's heading is
  clockwise from north, so it needs the opposite sign. **Verify in ten seconds,
  parked:** watch the heading field of `$VPFUS` and rotate the car clockwise. It
  must count *up* — 350 → 0 → 10. If it counts down, set this to +1.
* **`kBodyYIsLeft`** (default true). A module whose silkscreen has y pointing
  right needs `false`. Getting it wrong sends cornering acceleration the wrong
  way — visible as the fused track cutting inside bends the raw track goes round
  the outside of. It matters less than the yaw sign, because GPS speed corrects
  the velocity every second.

### What the fused build publishes

```
$VPFUS,<uptime_ms>,<lat>,<lon>,<speed_ms>,<heading_deg>,<sigma_m>,<quality>,<imu_hz>,<yaw_bias_dps>*<xx>
$VPFUS,187950,-6.9349896,107.6505035,0.00,190.6,0.32,1,25.0,0.004*55
```

`<sigma_m>` is the filter's own estimate of its position error, and the host
uses it as the fix accuracy — so it grows visibly when the receiver drops out
and the position becomes pure dead reckoning. `<imu_hz>` is measured, not
assumed: **0.0 means the filter is running on GPS alone**, which still gives a
correct position but one that jumps once a second and loses heading at a stop.
`<quality>` is the GPS quality digit behind the fix, so an RTK-anchored track is
still distinguishable from a standalone one.

The raw `$VPRTK` line continues alongside it at 1 Hz. Keeping both is what lets
the host draw the receiver beside the filter (`--compare`) and judge the fusion
against the thing it replaced — on the road there is no ground truth, so that
comparison is the only honest measure there is.

Every 10 s the board also reports its own health:

```
$VPSTA,208959,imu=25.0hz,baud=115200,bytes=41230,frames=4102,bad=0,gps-quality=1,filter=running,sigma=0.61m,yaw-bias=0.004dps
```

A yaw bias that settles within a degree per second of a constant and stays there
is a healthy gyro. One that keeps climbing means the heading is being corrected
against something the gyro disagrees with — check `kYawRateSign` first.

## What is in the kit

| Part | Module | Job |
|---|---|---|
| Base station | Quectel LC29HBS | Sits still on a known spot, measures its own error, broadcasts the correction as RTCM 3.x over LoRa |
| Rover / mobile station | Quectel LC29HDA on a DFRobot TEL0171-LoRa carrier | Receives that correction, applies it, outputs a corrected fix |
| 2 × GNSS antenna | L1 + L5 | One per board |
| 2 × LoRa module | pre-paired | The correction link, up to ~1.5 km in the open |

RTK works because both receivers watch the same satellites through the same
patch of atmosphere. The base cannot fix its own position error in absolute
terms, but it *can* measure how wrong the current sky is, and within a few km
that error is common to both. So the rover's accuracy is relative to the base —
which means a base placed carelessly gives you a precise position of the wrong
thing.

## Wiring

**I2C, not UART.** The rover speaks the same register protocol on either
transport, but only I2C relays configuration commands through to the GNSS chip
— see [Solution rate](#solution-rate). `Serial` on a Nucleo-64 is USART2 on
PA2/PA3, soldered to the ST-LINK virtual COM port, and stays as the link
carrying fixes to the PC.

| Rover, Gravity 4-pin | Wire | Nucleo-G474RE | Arduino pin |
|---|---|---|---|
| GND | black | GND | — |
| VCC (3.3–5 V) | red | 5V | — |
| SDA | green | **PB9** | D14 |
| SCL | blue | **PB8** | D15 |

Three things to check on a board that will not talk:

* **The switch on the rover must be on I2C, not UART.** It is sampled at
  power-up, so move it before applying power, not after. This is the first
  thing to check after converting from the UART wiring.
* **SDA to SDA, SCL to SCL** — no crossover, unlike the UART wiring this
  replaces.
* **PB9/PB8 are named explicitly in the firmware.** The G474 variant does not
  define `PIN_WIRE_SDA`/`SCL`, so a bare `Wire` would fall back to PA8/PA9 —
  not the pins on the Arduino header, and PA9 was the old UART TX.

The base station is powered on its own, over its Type-C connector at 5–10 V —
it is not wired to the STM32 at all. Its only connection to the rest of the
system is the LoRa radio.

## Bringing the pair up

The LoRa link is pre-configured at the factory, so there is no pairing step and
nothing to bind. The order that matters is the base going first, because the
rover cannot fix until corrections are already flowing.

1. **Place the base and leave it alone.** Its antenna needs open sky — a roof
   edge or a tripod in the middle of a car park, not a windowsill. Anything
   that blocks half the sky costs you half the constellation, and a base
   standing next to a wall reflects signals into its own antenna. It must not
   move once it has started: everything the rover reports is measured from it.
2. **Power the base and wait.** It has to determine its own position by
   averaging before it can say anything useful about anyone else's. Give it a
   clear-sky start and several minutes. DFRobot do not publish the survey
   duration, so treat the rover reaching quality 4 as the signal that the base
   is ready, rather than watching for something on the base itself.
3. **Power the rover** with its own antenna also on open sky, and keep the two
   antennas well apart.
4. **Watch the rover's RTK LED**, which reports the same state the firmware
   prints:

   | LED | Meaning |
   |---|---|
   | Off | No RTK — quality 0, 1 or 2. Corrections are not arriving, or there is no fix at all |
   | Blinking at 1 Hz | RTK Float (quality 5). Corrections are arriving; the solution has not converged |
   | Steady | RTK Fixed (quality 4). Centimetre-level — this is the goal |

Float to Fixed is the step that takes patience. Float means the corrections are
getting through and the maths is running, but the carrier-phase ambiguities are
not yet resolved; it is decimetre-level. Under open sky it usually resolves
within a minute or two. If it never does, the cause is nearly always sky view
at one end or distance from the base.

**Bring the pair up close together — 10–20 m apart, both antennas on open sky.**
Separate them only after the rover has reached quality 4 once. Debugging at
range cannot distinguish a link that is broken from one that is merely long,
and it is the most common way to spend a day on this.

If the rover never leaves quality 1 with `base_id=0`, nothing is arriving over
LoRa at all, and [10hz_i2c/diag_rtk_link.cpp](10hz_i2c/diag_rtk_link.cpp) is the tool for
that specific fault:

```
pio run -e diag_rtk_link -t upload && pio device monitor
```

It reports the rover-side LoRa configuration, then watches `base_id` and
`corr_age` and latches whether a correction has **ever** arrived — which
separates "the link has never worked" from "it worked and stopped", two
problems that look identical in a single fix line. It runs until unplugged, so
you can move the base or the antennas and watch the line change. After two
minutes with nothing received it prints the physical checklist, in order of how
often each item is actually the cause.

## What the firmware publishes

One line per fix, NMEA-style so the host can resynchronise after opening the
port mid-sentence. The checksum is the XOR of everything between `$` and `*` —
a line that does not verify should be dropped, not parsed.

```
$VPRTK,<uptime_ms>,<utc>,<lat>,<lon>,<alt_m>,<geoid_sep_m>,<quality>,<sats>,<hdop>,<accuracy_m>,<corr_age_s>,<base_id>*<xx>
$VPSTA,<uptime_ms>,<message>*<xx>
```

```
$VPRTK,48210,2026-08-07T04:21:09Z,-6.9034521,107.6181043,768.42,-2.10,4,24,0.71,0.02,1.2,1201*3F
$VPSTA,48210,quality=rtk-fixed,base=1201*57
```

`quality` is the GGA digit as the receiver reports it: 0 no fix, 1 GPS, 2 DGPS,
**4 RTK Fixed**, 5 RTK Float. `accuracy_m` is not measured — the receiver does
not report one — it is the accuracy class the quality digit stands for, so the
host has something to weight the fix by.

`corr_age_s` and `base_id` are how you tell whether the base is reaching you,
and they are the first thing to read when the RTK LED stays dark:

| `base_id` | `corr_age_s` | Meaning |
|---|---|---|
| `0` | `0.0` | **No correction has ever arrived.** The rover is solving alone. Nothing is wrong with the rover or the UART — the base is not broadcasting, or the LoRa link is not reaching |
| non-zero | small, steady | Corrections are flowing. Quality should climb to 5, then 4 |
| non-zero | climbing | The link *was* working and has dropped out — range, or the base lost its own fix |

A rover reporting a healthy `quality=1` fix with `base_id=0` is a working
receiver with no correction source. That is a base-station problem every time,
never a rover or wiring one.

`$VPSTA` carries everything that is not a fix: boot, wiring failures, each
change in fix quality, and a measured rate line every 10 s. Nothing is ever
printed as bare prose, so a host parser can treat any line that does not
checksum as line noise.

## Solution rate

The firmware runs the rover at **10 Hz**. Set `kFixHz` in
[main.cpp](10hz_i2c/main.cpp) to change it; everything else, including the NMEA
checksums, is derived from it.

The receiver defaults to 1 Hz, so `setup()` raises it with `$PAIR050,<ms>` on
every boot and reads the result back with `$PAIR051` before publishing anything.
Both go through `transmitAT()`: the bus carries the register protocol to the
DFRobot co-processor, not NMEA, and the LC29H sits behind it — a `$PAIR`
sentence written straight to the bus is not a command, it is corruption of the
next register read.

**This is why the module is on I2C.** On UART the passthrough went out and
never landed — the module kept solving at 1 Hz with no error anywhere, and
only the measured `rate=` line showed it. DFRobot's own `configParam` example
is annotated `// must use iic config parameter` and instantiates the I2C class,
which is the documentation for that behaviour such as it is. If the rate is
wrong again, [10hz_i2c/diag_rate.cpp](10hz_i2c/diag_rate.cpp) tells you which layer is at
fault in about 25 s:

```
pio run -e diag_rate -t upload && pio device monitor
```

It probes the passthrough with `$PQTMVERNO` (DFRobot's own no-op test command),
queries and sets the interval, and measures the register file's update rate
before and after — separating "the relay is dead" from "the receiver refused"
from "the receiver is at 10 Hz but DFRobot's registers only refresh at 1 Hz".

Accepting the command and sustaining the rate are different things: a marginal
sky view or a weak LoRa link will make the RTK engine drop epochs. So the rate
is measured from published fixes rather than assumed —

```
$VPSTA,60312,rate=10.0hz,fixes=100,window=10004ms,target=10hz*4A
```

If that reports well under `kFixHz` while `quality` stays 4, use `kFixHz = 5`.
5 Hz is the usual stable fallback on this kit.

Two things not to do:

- **Do not send `$PAIR864` to change the receiver's baud.** That is the internal
  LC29H↔co-processor link, factory-matched at both ends; moving one end leaves
  the board unable to answer, including to the command that would undo it.
  `kRoverBaud` is a different wire and is already 115200. Nothing needs a baud
  change — a fix line is ~110 bytes, so 10 Hz uses under 10% of the host link.
- **`$PAIR513` (save to flash) is off by default** — `kSaveToFlash`. It outlives
  the firmware, so reflashing the old 1 Hz build would leave the rover at 10 Hz
  and the discrepancy would look like a bug somewhere else. `setup()`
  reconfigures on every boot, so there is nothing to persist.

One consequence of the rate: the register interface carries UTC as whole
seconds, with no fractional part, so **ten consecutive 10 Hz fixes report the
same `<utc>`**. Order and time fixes by `<uptime_ms>`, which is per-fix.
Sub-second UTC exists only in the receiver's own GGA sentence — set
`EMIT_RAW_GGA` if the host needs it.

### The previous 1 Hz firmware

Kept verbatim in [legacy_1hz/](legacy_1hz/) and still buildable, so a problem
that appears at 10 Hz can be bisected against a known-good baseline instead of
reconstructed:

```
pio run -e legacy_1hz -t upload
```

It sits in its own folder, like every firmware here, so that two of them cannot drift
into each other's builds — `[env:legacy_1hz]` reaches it with `+<../legacy_1hz/>`.
It does not configure the rate at all; it takes whatever the receiver is already
doing. That is worth knowing when you flash it over a running 10 Hz session: the
receiver stays at 10 Hz until it loses power, and this build will publish every
one of those fixes despite its name. Nothing writes flash, so a power cycle
returns it to the 1 Hz default.

## Traps

Three of these are live bugs in DFRobot's library rather than anything to do
with your wiring, and all three are silent — they produce plausible output.

**The library never applies the hemisphere.** `getLat()` fills in
`latitudeDegree` from the magnitude and hands you `latDirection` separately, so
using it as it comes puts every fix in the northern and eastern hemispheres.
Bandung at 6.9°S then reads as a coordinate in Malaysia — plausible enough that
a map will happily draw it. [main.cpp](10hz_i2c/main.cpp) applies the sign; if you
write your own reader, do the same.

**Zero is a valid-looking coordinate.** Before the first fix the direction
bytes are `0`, not `'N'`/`'S'`, and the degree fields are zero. Publishing that
puts the car in the Gulf of Guinea. The firmware requires a real hemisphere
byte before it will emit a `$VPRTK` line.

**A truncated reply desynchronises every read after it.** `readReg()` waits
200 ms for exactly `len` bytes and then gives up — but it leaves whatever did
arrive in the buffer, so the next read is offset by that many bytes and returns
a wrong-but-plausible position from then on. The library's own `flush()` does
not help, because on Arduino that drains the *transmit* buffer. `drain_rx()`
discards stale bytes at the top of each cycle, while the bus is idle.

Two more that are inherent rather than bugs:

**Register precision is coarser than the receiver's.** Latitude comes back as
degrees, minutes and five decimals of a minute, which quantises to about
**1.9 cm** — below the kit's accuracy, but on the same order. Set
`EMIT_RAW_GGA` to `1` in [main.cpp](10hz_i2c/main.cpp) to also forward the rover's
own unrounded `$GNGGA`.

**`%f` needs a linker flag.** newlib-nano drops float support out of `printf`
to save flash, and without it `snprintf("%.7f")` writes an empty field where
the coordinate should be — the sentence still checksums, it just has no
position in it. `-Wl,-u,_printf_float` in [platformio.ini](platformio.ini) is
what puts it back.

## When the rover answers `no-device`

`$VPSTA,...,no-device` means `begin()` asked for the device-ID register and did
not get `0x20` back. Three causes look identical from outside — the mode switch
on I2C, TX/RX not crossed, or the wrong baud rate — so rather than permuting
them by hand, flash the probe:

```bash
pio run -e diag -t upload && pio device monitor
```

It sweeps every plausible baud rate, listening passively and then sending a
register read at each, and repeats every six seconds so you can move a wire and
watch the verdict change without reflashing. The question it answers is not
"which baud?" but **"do any bytes come back at all?"**:

| Result | Meaning |
|---|---|
| Bytes, and `0x20` found | That is the baud rate. Set `kRoverBaud` in [main.cpp](10hz_i2c/main.cpp) and reflash the default env |
| Bytes, but never `0x20` | Wiring, power and switch are all fine — only the framing is wrong. Look for `$` (`0x24`) in the listen bytes, which means it is streaming NMEA rather than serving registers |
| **No bytes at any baud** | Baud is not the cause. Nothing is transmitting: it is power, the UART/I2C switch, or TX/RX |

Only one reader can have the port. Running `pio device monitor` and another
`cat /dev/ttyACM0` at the same time makes the two steal bytes from each other,
which shows up as dropped characters mid-word — a garbled line here is the two
readers fighting, not a flaky board.

## Integrating with VisionPilot

Wired up. `nav.gps_source = nucleo` in `vision_pilot.conf` makes
`navigation::NucleoGps` open the ST-LINK port, verify each line's checksum and
fill in a `GpsFix` — the same interface the phone sources implement, so the map,
the route and the panel neither know nor care where position comes from.

```bash
./scripts/run/inference_cam_nav_gps.sh              # camera + map, driven by this board
./scripts/run/inference_cam_nav_gps.sh --compare    # draw the raw receiver beside the fused one
```

It reads both position sentences, which is what makes one host source work with
every firmware here: `$VPFUS` wins while it is arriving, `$VPRTK` is the
fallback, and `raw_fix()` always returns the unfiltered receiver so
`nav.compare_gps` can draw the two against each other.

`nav.imu_enabled` stays **off** with this source. That flag runs the host's own
GPS/IMU filter, which is right when the IMU is plugged into the PC — but this
board has already fused the two, and filtering an already-filtered position does
not average out, it lags. `make_gps_source()` refuses the combination and says
so rather than doing it quietly.

## References

* [KIT0198 kit wiki](https://wiki.dfrobot.com/GNSS_RTK_High_Precision_Positioning_Kit_SKU_KIT0198_EN)
* [Arduino UART example](https://wiki.dfrobot.com/kit0198-eu/docs/21395)
* [DFRobot_RTK_LoRa library](https://github.com/DFRobot/DFRobot_RTK_LoRa)
