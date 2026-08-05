#!/usr/bin/env python3
"""Fake IMU on a virtual serial port.

Streams WitMotion-format frames describing a module being rotated by hand, so
the viewer and the monitor can be checked with no hardware attached — and so a
problem can be pinned on the sensor rather than on this code.

    ./imu_sim.py                 # prints the port to point the tools at
    ./imu_sim.py --motion spin   # continuous yaw, for the compass
    ./imu_sim.py --motion still  # flat and stationary, for the bias estimate

Then, in another terminal:

    ../../scripts/imu_view.sh --port /dev/pts/N
"""

import argparse
import math
import os
import pty
import struct
import sys
import time


def frame(kind, values, extra=0):
    """One 11-byte WitMotion frame: 0x55, type, 4 int16 LE, checksum."""
    body = struct.pack("<BB4h", 0x55, kind, *values[:3], extra)
    return body + bytes([sum(body) & 0xFF])


def clamp16(v):
    return max(-32768, min(32767, int(v)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--motion", default="wobble",
                    choices=["wobble", "spin", "still", "tumble"])
    ap.add_argument("--rate", type=float, default=100.0, help="Hz")
    args = ap.parse_args()

    master, slave = pty.openpty()
    port = os.ttyname(slave)
    print(f"fake IMU streaming on {port}  ({args.motion}, {args.rate:.0f} Hz)")
    print(f"  point the tools at it:  scripts/imu_view.sh --port {port} --baud 115200")
    print("  Ctrl-C to stop")

    g = 9.80665
    t0 = time.monotonic()
    period = 1.0 / args.rate

    try:
        while True:
            t = time.monotonic() - t0

            if args.motion == "still":
                roll = pitch = yaw = 0.0
                wx = wy = wz = 0.0
            elif args.motion == "spin":
                roll, pitch = 0.0, 0.0
                yaw = (t * 45.0 + 180.0) % 360.0 - 180.0
                wx = wy = 0.0
                wz = 45.0
            elif args.motion == "tumble":
                roll = 50.0 * math.sin(t * 1.1)
                pitch = 40.0 * math.sin(t * 0.7 + 1.0)
                yaw = (t * 30.0 + 180.0) % 360.0 - 180.0
                wx, wy, wz = 55.0 * math.cos(t * 1.1), 28.0 * math.cos(t * 0.7 + 1.0), 30.0
            else:  # wobble
                roll = 25.0 * math.sin(t * 0.9)
                pitch = 18.0 * math.sin(t * 0.6)
                yaw = 35.0 * math.sin(t * 0.4)
                wx, wy, wz = 22.5 * math.cos(t * 0.9), 10.8 * math.cos(t * 0.6), 14.0 * math.cos(t * 0.4)

            # Gravity as the accelerometer of a body at this attitude would see it.
            r, p = math.radians(roll), math.radians(pitch)
            ax = -math.sin(p) * g
            ay = math.sin(r) * math.cos(p) * g
            az = math.cos(r) * math.cos(p) * g

            y = math.radians(yaw)
            mx = 30.0 * math.cos(y)
            my = -30.0 * math.sin(y)
            mz = 40.0

            packet = b"".join([
                frame(0x51, [clamp16(v / (16 * g) * 32768) for v in (ax, ay, az)],
                      clamp16(31.5 * 100)),
                frame(0x52, [clamp16(v / 2000.0 * 32768) for v in (wx, wy, wz)]),
                frame(0x53, [clamp16(v / 180.0 * 32768) for v in (roll, pitch, yaw)]),
                frame(0x54, [clamp16(v) for v in (mx, my, mz)], clamp16(31.5 * 100)),
            ])
            os.write(master, packet)
            time.sleep(period)
    except (KeyboardInterrupt, OSError):
        print("\nstopped")
        return 0


if __name__ == "__main__":
    sys.exit(main())
