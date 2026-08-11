#!/usr/bin/env python3
"""Live view of the DDPai N5 dashcam — cv2, and nothing else.

The camera is reached over its own Wi-Fi as RTSP. It never appears as
/dev/video*: it is not a USB Video Class device, so no cable makes it one.

Run it through scripts/sensors/ddpai_stream.sh, which joins the camera's Wi-Fi
and finds the stream URL first. Directly:

    ./ddpai_stream.py rtsp://193.168.0.1/live

Keys: q/Esc quit · s snapshot · r record · h hide HUD · f fullscreen
"""

import argparse
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime

def set_ffmpeg_options(transport):
    """Demuxer options. FFmpeg buffers a live stream ahead of the decoder, and
    that buffer is where seconds of delay hide — draining faster on our side
    cannot help, because the queue refills at the rate the camera sends.

    OpenCV exposes no API for this, only this variable, read when the capture is
    opened, so it must be set before cv2.VideoCapture().
    """
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        f"rtsp_transport;{transport}"
        "|fflags;nobuffer"          # hand each frame over as it decodes
        "|flags;low_delay"
        "|max_delay;0"              # do not wait to reorder
        "|reorder_queue_size;0"
        "|probesize;32768"          # start fast, but still detect H.264
        "|analyzeduration;0"
        "|stimeout;5000000"
    )
# The pip wheel bundles Qt but no fonts, so every window prints five lines about
# a missing font directory.
os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts")

try:
    import cv2
except ImportError:
    sys.exit("error: cv2 is not installed — run: python3 -m pip install opencv-python")

# 193.168.0.1, not 192.168 — DDPai's AP really does hand out 193.168.0.x.
DEFAULT_URL = "rtsp://193.168.0.1/live"

RECONNECT_ATTEMPTS = 20
RECONNECT_DELAY_S = 1.0


class Grabber(threading.Thread):
    """Keeps only the newest frame.

    read() hands over the *oldest* frame still queued, so every frame the UI
    cannot keep up with would add a permanent frame of lag — the picture falls
    further behind the road the longer it runs. Dropping them here keeps the
    view in the present.
    """

    def __init__(self, cap, reopen):
        super().__init__(daemon=True)
        self.cap = cap
        self.reopen = reopen
        self.lock = threading.Lock()
        self.frame = None
        self.dropped = 0
        self.running = True
        self.dead = False

    def run(self):
        while self.running:
            ok, frame = self.cap.read()
            if not ok or frame is None:
                cap = self.reopen()
                if cap is None:
                    self.dead = True
                    return
                self.cap = cap
                continue
            with self.lock:
                if self.frame is not None:
                    self.dropped += 1   # the UI never saw the previous one
                self.frame = frame

    def take(self):
        with self.lock:
            frame, self.frame = self.frame, None
            return frame

    def stop(self):
        self.running = False


class FpsMeter:
    """Arrivals over the last second — what tells you the link is healthy.

    A stream's advertised CAP_PROP_FPS says what it intends, not what arrives.
    """

    def __init__(self):
        self.stamps = deque()

    def tick(self):
        now = time.monotonic()
        self.stamps.append(now)
        while self.stamps and now - self.stamps[0] > 1.0:
            self.stamps.popleft()

    @property
    def value(self):
        return len(self.stamps)


def draw_hud(frame, lines, recording):
    """A translucent panel, so the text stays readable over a bright road."""
    pad, line_h = 10, 22
    w = 8 + max(len(t) for t in lines) * 9
    h = pad * 2 + line_h * len(lines)

    panel = frame[0:h, 0:w].copy()
    cv2.rectangle(panel, (0, 0), (w, h), (0, 0, 0), -1)
    cv2.addWeighted(panel, 0.55, frame[0:h, 0:w], 0.45, 0, frame[0:h, 0:w])

    for i, text in enumerate(lines):
        cv2.putText(frame, text, (pad, pad + line_h * (i + 1) - 6),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    if recording:
        cv2.circle(frame, (frame.shape[1] - 26, 24), 8, (0, 0, 255), -1)
        cv2.putText(frame, "REC", (frame.shape[1] - 80, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)


def open_stream(url):
    # Params go in at open time, which is the only time they can take effect —
    # a decoder is configured when it is created, not afterwards.
    #
    # N_THREADS=1 is the big one for latency. Frame-threaded decoding does not
    # emit a frame until (threads-1) more have been fed in, so at 30 fps an
    # 8-thread decoder sits on ~230 ms of picture before it hands over the
    # first. One thread decodes 720p far faster than real time anyway, so this
    # costs nothing and removes that delay outright.
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG, [
        cv2.CAP_PROP_N_THREADS, 1,
        cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, 8000,
        cv2.CAP_PROP_READ_TIMEOUT_MSEC, 8000,
    ])
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # honoured by some backends only
    return cap


def main():
    p = argparse.ArgumentParser(
        description="DDPai N5 live view (cv2)",
        epilog="Keys: q/Esc quit · s snapshot · r record · h hide HUD · f fullscreen",
    )
    p.add_argument("url", nargs="?", default=DEFAULT_URL, help=f"stream URL (default: {DEFAULT_URL})")
    p.add_argument("--record", metavar="FILE", help="record to FILE (.mp4)")
    p.add_argument("--scale", type=float, default=1.0, help="resize the view, e.g. 0.5")
    p.add_argument("--hud", action="store_true", help="show the stats overlay (h toggles)")
    p.add_argument("--transport", choices=("tcp", "udp"), default="tcp",
                   help="RTSP transport; udp drops late packets instead of "
                        "stalling to retransmit them (default: tcp)")
    p.add_argument("--keep-watermark", action="store_true",
                   help="keep the camera's burnt-in name and clock")
    p.add_argument("--crop-bottom", type=float, default=0.12, metavar="FRAC",
                   help="fraction of the frame cropped off the bottom to remove "
                        "the watermark (default: 0.12)")
    args = p.parse_args()

    set_ffmpeg_options(args.transport)

    print(f"opening {args.url} ...")
    cap = open_stream(args.url)
    if cap is None:
        print(f"\nfailed to open {args.url}", file=sys.stderr)
        print("Run ./scripts/sensors/ddpai_stream.sh, which joins the camera's "
              "Wi-Fi and finds the URL first.", file=sys.stderr)
        return 1

    src_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    src_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    src_fps = cap.get(cv2.CAP_PROP_FPS)
    print(f"  {src_w}x{src_h} @ {src_fps:.0f} fps nominal")

    attempts = {"n": 0}

    def reopen():
        """A Wi-Fi blip should not end the session — the camera's AP is weak."""
        while attempts["n"] < RECONNECT_ATTEMPTS:
            attempts["n"] += 1
            print(f"stream stalled — reconnecting ({attempts['n']}/{RECONNECT_ATTEMPTS})",
                  file=sys.stderr)
            time.sleep(RECONNECT_DELAY_S)
            new = open_stream(args.url)
            if new is not None:
                attempts["n"] = 0
                return new
        return None

    grabber = Grabber(cap, reopen)
    grabber.start()

    window = "VisionPilot — DDPai N5"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window, 1280, 720)
    print("keys: q/Esc quit · s snapshot · r record · h hide HUD · f fullscreen")

    # The camera burns "DDPAI N5 Dual" and a clock into the bottom of every
    # frame. It cannot be turned off from here — that is a firmware OSD, drawn
    # before the picture is encoded — so the strip carrying it is cut away.
    # Switch the watermark off in the DDPai app (camera settings) and this can
    # come off with --keep-watermark, giving the full frame back.
    crop_rows = 0 if args.keep_watermark else max(0.0, min(0.4, args.crop_bottom))

    meter = FpsMeter()
    writer = None
    record_to = args.record
    show_hud = args.hud
    fullscreen = False
    shown = 0
    started = time.monotonic()

    while not grabber.dead:
        frame = grabber.take()
        if frame is None:
            if cv2.waitKey(5) & 0xFF in (ord("q"), 27):
                break
            if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
                break
            continue

        meter.tick()
        shown += 1

        if crop_rows:
            frame = frame[0:int(frame.shape[0] * (1.0 - crop_rows)), :]

        if args.scale != 1.0:
            frame = cv2.resize(frame, None, fx=args.scale, fy=args.scale,
                               interpolation=cv2.INTER_AREA)

        if record_to and writer is None:
            # The measured rate, not the advertised one — otherwise playback runs
            # fast or slow by however much the link is dropping.
            fps = meter.value if meter.value > 1 else (src_fps if src_fps > 1 else 25)
            writer = cv2.VideoWriter(record_to, cv2.VideoWriter_fourcc(*"mp4v"),
                                     fps, (frame.shape[1], frame.shape[0]))
            if not writer.isOpened():
                print(f"could not open {record_to} for writing", file=sys.stderr)
                writer, record_to = None, None
            else:
                print(f"recording to {record_to} at {fps:.0f} fps")
        if writer is not None:
            writer.write(frame)

        view = frame.copy() if show_hud else frame
        if show_hud:
            draw_hud(view, [
                f"stream   {args.url}",
                f"size     {frame.shape[1]}x{frame.shape[0]}",
                f"fps      {meter.value:.0f}   (nominal {src_fps:.0f})",
                f"frames   {shown}   dropped {grabber.dropped}",
                f"uptime   {time.monotonic() - started:.0f}s",
            ], writer is not None)

        cv2.imshow(window, view)

        key = cv2.waitKey(1) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("s"):
            name = f"ddpai_{datetime.now():%Y%m%d_%H%M%S}.jpg"
            cv2.imwrite(name, frame)
            print(f"saved {name}")
        elif key == ord("r"):
            if writer is not None:
                writer.release()
                writer, record_to = None, None
                print("recording stopped")
            else:
                record_to = f"ddpai_{datetime.now():%Y%m%d_%H%M%S}.mp4"
        elif key == ord("h"):
            show_hud = not show_hud
        elif key == ord("f"):
            fullscreen = not fullscreen
            cv2.setWindowProperty(
                window, cv2.WND_PROP_FULLSCREEN,
                cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)

        if cv2.getWindowProperty(window, cv2.WND_PROP_VISIBLE) < 1:
            break

    grabber.stop()
    if writer is not None:
        writer.release()
    cap.release()
    cv2.destroyAllWindows()
    print(f"\n{shown} frames shown")
    return 1 if shown == 0 else 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(130)
