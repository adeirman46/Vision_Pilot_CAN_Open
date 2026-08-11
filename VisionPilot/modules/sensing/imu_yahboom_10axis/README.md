# Yahboom MRA02 10-axis IMU

Driver, attitude estimate and a live 3D view for the Yahboom MRA02-V1.0 10-axis
IMU (3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer, barometer),
attached over USB through a CH340 serial bridge.

The point of the 3D view is the functional check: turn the module in your hand
and the aircraft on screen turns with it. If roll, pitch and yaw follow what
your hands are doing, then the wiring, the axes, the scale factors and the
fusion are all correct — which is what has to be true before the IMU is worth
fusing with GPS.

```
scripts/sensors/imu_view.sh                 # live 3D window
scripts/sensors/imu_view.sh --monitor       # console readout, works over ssh
scripts/sensors/imu_view.sh --probe         # report port, baud rate and protocol
scripts/sensors/imu_view.sh --raw           # hex dump of the serial stream
scripts/sensors/imu_view.sh --fix-port      # rebind the USB-serial driver
```

## Layout

| Path | What it is |
|---|---|
| `include/imu/`, `src/` | The library: serial port, protocol decoder, attitude estimator, threaded reader. No UI dependencies. |
| `viewer/` | Qt5 3D window. Software-rendered with `QPainter`, so it needs no GL context. |
| `tools/imu_monitor.cpp` | Console monitor, raw sniffer and CSV logger. |
| `tools/imu_sim.py` | Fake IMU on a virtual serial port — checks the tools with no hardware attached. |
| `<repo>/scripts/sensors/imu_view.sh` | Launcher: builds if needed, sorts the serial port out, opens the view. |

Build on its own, or as an `add_subdirectory()` of a larger tree:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
```

## Wire protocol

Yahboom does not publish the MRA02 frame format — the protocol document in
their repository describes the older `0x55`-header module, which this board is
**not**. What follows was read off a live MRA02-V1.0 and cross-checked three
ways, so it is exact rather than assumed.

```
0x7E 0x23 | len | id | payload | checksum
```

* `len` counts the whole frame, header and checksum included.
* `checksum` is the low byte of the sum of every preceding byte.
* Four frames make one reading, sent back to back at 25 Hz on factory settings,
  115200 baud 8N1.

| id | len | payload |
|---|---|---|
| `0x04` | 23 | 9 × int16 LE — accelerometer, gyroscope, magnetometer |
| `0x16` | 21 | 4 × float32 LE — quaternion `w x y z` |
| `0x26` | 17 | 3 × float32 LE — Euler `roll pitch yaw`, in **radians** |
| `0x32` | 21 | 4 × float32 LE — altitude m, temperature °C, pressure Pa, reference pressure Pa captured at power-on |

Scale factors:

| Channel | Scale | How it was confirmed |
|---|---|---|
| Accelerometer | `value / 2048` g (±16 g full scale) | The vector length reads 2048 ± 4 LSB at rest — exactly 1 g. |
| Gyroscope | `value / 32768 × 2000` °/s (±2000 °/s) | Yahboom's own reference driver (`imu_usb.py`, `k_gyro = 2000.0`) uses this range for their IMU line, alongside `k_acc = 16.0` — and the ±16 g that implies is exactly what the accelerometer here measures. A hand rotation reads a plausible few tens of °/s. |
| Magnetometer | raw LSB | No published scale. Only the direction is used, so a scale is not needed. |
| Quaternion | float, normalise before use | Arrives at ‖q‖ ≈ 0.9983, not exactly 1. |

The three cross-checks that make this certain, on a stationary board:

1. The quaternion in `0x16`, decomposed ZYX, gives exactly the Euler angles in
   `0x26` — to five decimal places.
2. The accelerometer vector matches the attitude those angles describe to three
   decimals: measured `(-0.0356, -0.5493, 0.8364)` g against predicted
   `(-0.0344, -0.5461, 0.8371)` g.
3. Over 6 kB of capture, every frame passed its checksum — 0 bad, 0 discarded.

Axes are right-handed, as silk-screened on the board: **x forward** (roll axis),
**y left** (pitch axis), **z up** (yaw axis).

Yahboom's [10-axis_IMU_Module](https://github.com/YahboomTechnology/10-axis_IMU_Module)
repository is for their *older* board: its reference driver reads 11-byte
`0x55`-header frames at 9600 baud, and nothing in it mentions `0x7E`. It is still
worth having, because the ranges it hard-codes — `k_acc = 16.0`, `k_gyro =
2000.0` — are the ones this part uses too.

Other formats are still decoded, so the same tools work against a WitMotion
WT901/HWT906, a Yesense YIS unit, or firmware that just prints ASCII. The
decoder runs every parser it knows over the stream and locks onto whichever
produces checksum-clean frames. If none do, `--raw` dumps the bytes and reports
any repeating frame structure it can find.

## Attitude and position

`AttitudeEstimator` prefers the module's own quaternion, because its internal
filter runs on the factory magnetometer calibration and nothing here can
reproduce that. `--raw-attitude` ignores it and runs a Mahony complementary
filter over the raw channels instead — gyro for the short term, gravity to
correct roll and pitch, magnetometer to correct yaw. That is the mode to use
when checking whether the raw channels themselves are sane.

Gyro bias is learned automatically whenever the module is demonstrably at rest,
which is what stops the model creeping when nothing is touching it.

Position is dead reckoning, and it is the part that needed the most care. Three
things dominate the error, none of them the integration itself:

* **The accelerometer offset is in the body frame.** It belongs to the sensor,
  not to the room. Held in world axes it is right at one attitude and wrong at
  every other, so turning the board becomes metres of phantom travel.
* **Rest has to be detected by variance, not magnitude.** This gyro reads a
  steady −1.7 °/s at rest. A magnitude threshold calls that rotation, so the
  unit is never still, so the bias is never learned, so it is never still — a
  deadlock that ends with velocity ramping away. A constant offset has zero
  variance; a real movement does not.
* **But variance alone is not enough**, because steady acceleration is steady.
  Pushing the board at a constant 1 m/s² varies |a| no more than noise does, so
  a variance-only test calls a real movement "at rest" and the zero-velocity
  update deletes exactly the motion being measured. Rest needs both: low
  variance *and* small linear acceleration once the offset is removed.

On top of that, velocity is not integrated at all while the board turns faster
than 20 °/s. Gravity is 9.8 m/s² and every attitude solution lags reality
slightly; two degrees of lag leaks 0.34 m/s² sideways, which is larger than most
real movements and points wherever the turn happens to be facing.

`tests/test_dead_reckoning.cpp` pins all of this down against motion profiles
whose answers are known. Measured, with a simulated sensor carrying this board's
actual gyro bias, an accelerometer offset, 0.2° of attitude error and noise:

| Scenario | Result |
|---|---|
| Stationary 60 s | 1.1 cm drift |
| 1.00 m out, then stop | 1.6 cm error, held afterwards |
| 0.30 m out and back, ×3 | 0.7 cm at the far point, 1.4 cm back at the start |
| 90° yaw back and forth, ×8 | 0.9 cm — no translation from turning |

That is good for seconds to a minute, not for navigating. Bias is measured, not
eliminated, and what is left still grows with time — which is precisely the job
GPS does in the fusion. The EKF should consume the accelerometer and gyroscope
directly, with this position and velocity as the propagation model between
fixes, not as an independent measurement.

## Serial port

The board appears as `/dev/ttyUSB0` once the `ch341` driver owns the USB
interface. On Ubuntu, `brltty` — the braille display daemon — claims CH340
devices on sight and holds them through `usbfs`, so no serial node is ever
created and the sensor looks dead. `scripts/sensors/imu_view.sh --fix-port` diagnoses
and repairs it; the permanent fix is:

```bash
sudo systemctl mask brltty-udev.service
sudo pkill brltty
```

Undo with `sudo systemctl unmask brltty-udev.service` if you do use a braille
display.

If the port exists but is not readable, add yourself to `dialout`:

```bash
sudo usermod -aG dialout $USER    # then log out and back in
```

## Keys in the 3D view

| Key | Action |
|---|---|
| `Space` | Zero here — the current attitude becomes 0/0/0, so a crooked mounting reads straight |
| `R` | Reset the dead-reckoned position and clear the track |
| `C` | Recalibrate the gyro bias from scratch |
| `V` | Reset the camera |
| drag / wheel | Orbit / zoom |
