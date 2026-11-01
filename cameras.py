import os
import re
import sys
import time
import glob
import math
import shutil
import threading
import subprocess
 
import cv2
import numpy as np
 
# Applied to every camera found.
#
# MJPG 640x480@30 for the six HD USB cameras (32e4:8830), measured 2026-10-03:
# all six hold 30 FPS together. Uncompressed YUYV at 640x480 has only a 30 FPS
# mode and needs about 18 MB/s per camera: two fit on the shared USB 2.0 bus,
# and a third stops ALL of them ("Failed to resubmit video URB"). MJPG relies
# on the patched uvcvideo (mjpeg_payload=800, see uvcvideo_mjpeg_patch/).
# Earlier Global Shutter cameras (32e4:0234) needed YUYV 320x240@20 instead,
# because without the patch their MJPG reserves about 24 MB/s at any size.
FOURCC = "MJPG"
WIDTH = 640
HEIGHT = 480
FPS = 30

# With the patched uvcvideo from uvcvideo_mjpeg_patch/ loaded
# (mjpeg_payload=800), all six also fit in MJPG up to 1280x720@30. Pick a
# mode on the command line without editing this file:
#   python cameras.py MJPG 1280 720 30
if len(sys.argv) == 5:
    FOURCC = sys.argv[1].upper()
    WIDTH, HEIGHT, FPS = (int(value) for value in sys.argv[2:])
 
# Per-camera exceptions, keyed by USB port. Use this if a camera has no
# mode in FOURCC at the default size (check with v4l2-ctl --list-formats-ext).
# Example:  "4.1.4.2": {"width": 640, "height": 480, "fps": 15},
#
# "exposure" switches that camera to manual exposure (units of 0.1 ms).
# Auto-exposure can hold the shutter open longer than a frame lasts and slow
# the camera down in dim light. Keep it under 10000 / fps: under 333 for a
# 30 fps camera, under 500 for 20 fps. "gain" (0-100) brightens the picture
# without costing frame rate.
#
# Cameras without their own "exposure" use EXPOSURE below. A fixed 30 ms
# washes out views facing daylight (up to 26% of pixels pure white), so set
# EXPOSURE = None (auto) or a shorter value outdoors.
# Example:  "4.1.3.4": {"exposure": 100, "gain": 20},
# The 16MP side cameras (32e4:0298 / 32e4:1298) come out much darker than the
# HD USB and Global Shutter cameras at the same 30 ms + gain 40 (smaller
# pixels). The shutter cannot go past 33 ms without losing 30 FPS, so they get
# more gain instead (2026-10-31).
OVERRIDES = {
    "4.1.2.4": {"gain": 80},   # front left
    "4.1.4.3": {"gain": 80},   # front right
    "4.1.3.4": {"gain": 80},   # back left
    "4.1.4.4": {"gain": 80},   # back right
    # Global Shutter and HD USB cameras were too bright at gain 40.
    "4.1.2.3": {"gain": 0},    # front mid (Global Shutter)
    "4.1.2.2": {"gain": 0},    # back (Global Shutter)
    "4.1.3.3": {"gain": 0},    # front narrow (HD USB)
}

# Defaults for every camera not overridden above. The HD USB cameras' auto
# exposure drops them to 17-20 FPS in dim light (measured 2026-09-30); a fixed
# 30 ms shutter with gain 40 holds 30 FPS at similar brightness indoors.
# It does not adapt to light: outdoors set EXPOSURE = None (auto) or a much
# shorter value. None leaves auto exposure / the current gain in place.
EXPOSURE = 300       # 100 us units (300 = 30 ms), or None for auto
GAIN = 40            # 0-100, or None
POWER_LINE_HZ = 60   # anti-flicker for US mains; the cameras default to 50
 
# For testing: list ports here to run ONLY those cameras, e.g.
# ONLY_PORTS = ["4.1.4.2", "4.1.3.3"]. Leave empty to run every camera.
ONLY_PORTS = []

# Tile names and number keys (1-9), by USB port, in reading order of GRID below.
# Cameras not listed are shown last, by port.
# Updated 2026-10-31: two Global Shutter (front mid, back), four 16MP and an
# HD USB narrow front.
CAMERA_ORDER = [
    ("4.1.3.3", "front narrow"),
    ("4.1.2.4", "front left"),
    ("4.1.2.3", "front mid"),
    ("4.1.4.3", "front right"),
    ("4.1.3.4", "back left"),
    ("4.1.2.2", "back"),
    ("4.1.4.4", "back right"),
]

# Fixed window layout, row by row; None leaves that slot empty. Connected
# cameras not placed here go on an extra row at the bottom. Set GRID = [] to
# fill the window automatically in CAMERA_ORDER instead.
GRID = [
    [None, "4.1.3.3", None],             # front narrow, on its own
    ["4.1.2.4", "4.1.2.3", "4.1.4.3"],   # front left, front mid (wide), front right
    ["4.1.3.4", "4.1.2.2", "4.1.4.4"],   # back left, back, back right (mirrored)
]
ROLES = dict(CAMERA_ORDER)

# Shown mirrored left-right, like a rear-view mirror, so the car's left side
# appears on the left of the screen for the rear cameras too. Display only;
# the frames themselves are not changed.
MIRRORED = {"4.1.3.4", "4.1.2.2", "4.1.4.4"}
_RANK = {port: rank for rank, (port, _) in enumerate(CAMERA_ORDER)}


def display_rank(port):
    return (_RANK.get(port, len(_RANK)), port)
 
# Many UVC cameras stretch exposure in dim light and quietly halve their
# frame rate to do it (15 -> 7.5, 25 -> 12.5). Turning this off keeps the
# rate fixed; the picture may get darker in low light instead.
LOCK_FRAMERATE = True
 
USB_BUDGET_MBS = 60.0    # rough usable ceiling for one high-speed bus
 
# Frame slots the driver keeps per camera. With only 1, the camera has
# nowhere to put the next frame while we handle the current one, so it
# drops every other frame (15 -> 7.5, 25 -> 12.5). The grabber threads read
# continuously, so a few extra slots don't add noticeable lag.
BUFFERS = 4
 
MAX_CAMERAS = 9          # more than this gets unreadable in one window
# Tiles follow the capture aspect ratio: 480x360 for 4:3, 600x338 for 16:9.
TILE_W = 480 if HEIGHT * 4 >= WIDTH * 3 else 600
TILE_H = round(TILE_W * HEIGHT / WIDTH)
OPEN_STAGGER = 0.6       # pause between opening cameras, so each can settle
STALL_AFTER = 2.0        # seconds without a frame before a camera reads stalled
RESCAN_EVERY = 5.0       # how often to look for newly plugged-in cameras
WINDOW = "Cameras"
 
HAVE_V4L2CTL = shutil.which("v4l2-ctl") is not None
 
 
def mjpeg_payload_cap():
    """The patched uvcvideo's mjpeg_payload (bytes per microframe), or 0."""
    try:
        with open("/sys/module/uvcvideo/parameters/mjpeg_payload") as handle:
            return int(handle.read())
    except (OSError, ValueError):
        return 0


def reserved_mbs(w, h, fps):
    """Roughly what USB reserves for a stream, in MB/s."""
    if FOURCC == "MJPG":
        # The camera, not the frame size, decides: the Global Shutter
        # cameras take about 24 MB/s unless the patched driver caps it
        # (the cap is per 125 us microframe).
        cap = mjpeg_payload_cap()
        return cap * 8000 / 1_000_000 if cap else 24.5
    return w * h * 2 * fps / 1_000_000
 
 
def grid_shape(n):
    """Rows and columns that fit n tiles with the fewest empty slots."""
    if n <= 1:
        return 1, 1
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)
    return rows, cols
 
 
def device_name(path):
    """Human-readable camera name straight from the kernel."""
    node = os.path.basename(os.path.realpath(path))
    try:
        with open(f"/sys/class/video4linux/{node}/name") as handle:
            name = handle.read().strip()
    except OSError:
        return node
    # Kernel names often repeat themselves: "HD USB Camera: HD USB Camera".
    half = name.split(":")[0].strip()
    return half if half and name.startswith(f"{half}:") else name
 
 
def port_of(path):
    """Short USB port id from the by-path name, e.g. 4.1.2.3."""
    match = re.search(r"-0:([0-9.]+):", path)
    return match.group(1) if match else "?"
 
 
def find_cameras():
    """Every capture device currently present, deduped, keyed by USB port.
 
    /dev/v4l/by-path/ is used rather than /dev/videoN because the numbering
    shuffles between boots. Each camera exposes a capture node (index0) and
    a metadata node (index1); only the former is useful. The same device can
    also appear under both usb-0: and usbv2-0: names, hence the dedupe.
    """
    found = {}
    for path in sorted(glob.glob("/dev/v4l/by-path/*video-index0")):
        port = port_of(path)
        if ONLY_PORTS and port not in ONLY_PORTS:
            continue
        if port not in found:
            found[port] = path
    return dict(sorted(found.items(), key=lambda item: display_rank(item[0])))
 
 
def fourcc_text(code):
    """Turn OpenCV's packed FOURCC number back into e.g. 'MJPG'."""
    code = int(code)
    text = "".join(chr((code >> 8 * i) & 0xFF) for i in range(4))
    return text if text.isprintable() and text.strip() else "?"
 
 
class Grabber(threading.Thread):
    """One thread per camera, always holding only the newest frame."""
 
    def __init__(self, path, port):
        super().__init__(daemon=True)
        self.path = path
        self.port = port
        self.name = device_name(path)
        role = ROLES.get(port)
        self.label = f"{role} ({port})" if role else f"{self.name} ({port})"
        self.frame = None
        self.lock = threading.Lock()
        self.running = True
        self.last_frame_at = 0.0
        self._fps = 0.0
        self.status = "starting"
        self.mode = ""
        cfg = OVERRIDES.get(port, {})
        self.want_w = cfg.get("width", WIDTH)
        self.want_h = cfg.get("height", HEIGHT)
        self.want_fps = cfg.get("fps", FPS)
        self.exposure = cfg.get("exposure", EXPOSURE)
        self.gain = cfg.get("gain", GAIN)
        self.cost = reserved_mbs(self.want_w, self.want_h, self.want_fps)
        self.reopens = 0
        self._announced = False
 
    @property
    def fps(self):
        """Reads zero once frames stop arriving, so a stall is visible."""
        if time.time() - self.last_frame_at > STALL_AFTER:
            return 0.0
        return self._fps
 
    def _v4l2(self, *args):
        """Run v4l2-ctl against this camera; failures are ignored."""
        if not HAVE_V4L2CTL:
            return ""
        try:
            return subprocess.run(["v4l2-ctl", "-d", self.path, *args],
                                  capture_output=True, text=True,
                                  timeout=5, check=False).stdout
        except Exception:
            return ""
 
    def _lock_framerate(self):
        # The control has two names depending on the kernel version; a
        # camera that lacks it simply rejects the call, which is harmless.
        self._v4l2("--set-ctrl=exposure_dynamic_framerate=0")
        self._v4l2("--set-ctrl=exposure_auto_priority=0")
 
    def _open(self):
        cap = cv2.VideoCapture(self.path, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        # The format must be set before width/height, or the driver picks
        # the size from whatever format is currently active.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*FOURCC))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.want_w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.want_h)
        cap.set(cv2.CAP_PROP_FPS, self.want_fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, BUFFERS)
 
        if LOCK_FRAMERATE:
            self._lock_framerate()
        if POWER_LINE_HZ in (50, 60):
            self._v4l2(f"--set-ctrl=power_line_frequency={1 if POWER_LINE_HZ == 50 else 2}")
 
        # Manual exposure: 1 = manual mode on UVC cameras. Exposure must be
        # set after switching modes or the camera ignores it.
        # Otherwise select auto (3 = aperture priority) explicitly: the camera
        # keeps the last mode set until it is unplugged, even by another program.
        if self.exposure is not None:
            self._v4l2("--set-ctrl=auto_exposure=1")
            self._v4l2(f"--set-ctrl=exposure_time_absolute={self.exposure}")
        else:
            self._v4l2("--set-ctrl=auto_exposure=3")
        if self.gain is not None:
            self._v4l2(f"--set-ctrl=gain={self.gain}")
 
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        rate = cap.get(cv2.CAP_PROP_FPS)
        fmt = fourcc_text(cap.get(cv2.CAP_PROP_FOURCC))
 
        # Some cameras ignore OpenCV's fps request while OpenCV still reports
        # the requested value, so always set it on the device directly and
        # accept whatever the device reports back. A camera running faster
        # than asked eats bandwidth the others on the same hub need.
        if HAVE_V4L2CTL:
            self._v4l2(f"--set-parm={self.want_fps}")
            out = self._v4l2("--get-parm")
            got = re.search(r"Frames per second\s*:\s*([\d.]+)", out)
            if got:
                rate = float(got.group(1))
 
        self.mode = f"{w}x{h} {fmt} @{rate:.0f}"
        self.cost = reserved_mbs(w, h, rate)
        if not self._announced:
            self._announced = True
            print(f"  {self.label}: {self.mode}   reserves {self.cost:5.1f} MB/s")
            if fmt != FOURCC:
                print(f"    ^ not {FOURCC}; this camera may have no {FOURCC} mode at "
                      f"{self.want_w}x{self.want_h}")
        return cap
 
    def run(self):
        cap = None
        last = time.time()
        fails = 0
        while self.running:
            try:
                if cap is None:
                    cap = self._open()
                    if cap is None:
                        self.status = "cannot open"
                        time.sleep(3.0)
                        continue
                    fails = 0
 
                ok, frame = cap.read()
                if not ok:
                    fails += 1
                    self.status = "no frames"
                    if fails >= 20:
                        cap.release()
                        cap = None
                        self.reopens += 1
                        # Back off progressively; hammering a camera that
                        # cannot stream burns time the others could use.
                        time.sleep(min(2.0 * self.reopens, 15.0))
                    continue
 
                fails = 0
                self.reopens = 0
                self.status = "ok"
                now = time.time()
                dt = now - last
                last = now
                self.last_frame_at = now
                if dt > 0:
                    self._fps = 0.9 * self._fps + 0.1 * (1.0 / dt)
 
                with self.lock:
                    self.frame = frame
 
            except cv2.error:
                # Truncated MJPEG frame. A symptom, not a fatal error.
                self.status = "corrupt frames"
                continue
            except Exception as exc:
                self.status = "error"
                print(f"[{self.label}] {exc}")
                if cap is not None:
                    cap.release()
                cap = None
                time.sleep(2.0)
 
        if cap is not None:
            cap.release()
 
    def read(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()
 
    def stop(self):
        self.running = False
 
 
def placeholder(text, sub=""):
    tile = np.zeros((TILE_H, TILE_W, 3), np.uint8)
    tile[:] = (24, 24, 24)
    cv2.putText(tile, text, (24, TILE_H // 2),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (70, 70, 190), 2)
    if sub:
        cv2.putText(tile, sub[:40], (24, TILE_H // 2 + 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (140, 140, 140), 1)
    return tile
 
 
def tile_for(grabber, position):
    frame = grabber.read()
    rate = grabber.fps
    if frame is None:
        tile = placeholder(grabber.status.upper(), grabber.label)
    elif rate == 0.0:
        tile = placeholder("STALLED", grabber.label)
    else:
        tile = cv2.resize(frame, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA)
        if grabber.port in MIRRORED:
            tile = cv2.flip(tile, 1)
 
    colour = ((120, 235, 120) if rate >= 10 else
              (90, 200, 235) if rate >= 3 else (90, 90, 235))
    label = f"{position + 1}  {grabber.label}   {rate:4.1f} fps"
    cv2.rectangle(tile, (0, TILE_H - 24), (TILE_W, TILE_H), (0, 0, 0), -1)
    cv2.putText(tile, label[:58], (8, TILE_H - 7),
                cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)
    cv2.rectangle(tile, (0, 0), (TILE_W - 1, TILE_H - 1), (70, 70, 70), 1)
    return tile
 
 
def arrange(grabbers):
    """Rows of grabbers (None = empty slot) following GRID, else auto-filled."""
    if not GRID:
        rows, cols = layout_shape(grabbers)
        slots = list(grabbers) + [None] * (rows * cols - len(grabbers))
        return [slots[r * cols:(r + 1) * cols] for r in range(rows)]
    by_port = {g.port: g for g in grabbers}
    cols = max(len(row) for row in GRID)
    layout = [[by_port.get(port) for port in row] + [None] * (cols - len(row)) for row in GRID]
    placed = {port for row in GRID for port in row if port}
    extra = [g for g in grabbers if g.port not in placed]
    for start in range(0, len(extra), cols):
        chunk = extra[start:start + cols]
        layout.append(chunk + [None] * (cols - len(chunk)))
    return layout


def layout_shape(grabbers):
    layout = arrange(grabbers)
    return len(layout), len(layout[0])


def compose(grabbers):
    if not grabbers:
        return placeholder("NO CAMERAS", "waiting for a device")
    index = {id(g): i for i, g in enumerate(grabbers)}
    rows = []
    for row in arrange(grabbers):
        tiles = []
        for g in row:
            if g is None:
                blank = np.zeros((TILE_H, TILE_W, 3), np.uint8)
                blank[:] = (18, 18, 18)
                tiles.append(blank)
            else:
                tiles.append(tile_for(g, index[id(g)]))
        rows.append(np.hstack(tiles))
    return np.vstack(rows)
 
 
def report_budget(grabbers):
    total = sum(g.cost for g in grabbers)
    verdict = "OK" if total < USB_BUDGET_MBS else "OVER BUDGET"
    print(f"\nTotal reserved: {total:.1f} MB/s of ~{USB_BUDGET_MBS:.0f} "
          f"available  [{verdict}]")
 
 
def main():
    if not HAVE_V4L2CTL:
        print("Note: v4l2-ctl not found, so frame-rate locking is skipped. "
              "Install it with: sudo apt install v4l-utils\n")
 
    found = find_cameras()
    print(f"Found {len(found)} camera(s)\n")
 
    grabbers = []
    known = set()
    for port, path in found.items():
        if len(grabbers) >= MAX_CAMERAS:
            print(f"  (skipping {port}; showing at most {MAX_CAMERAS})")
            break
        g = Grabber(path, port)
        g.start()
        grabbers.append(g)
        known.add(port)
        time.sleep(OPEN_STAGGER)
 
    time.sleep(1.0)          # let the last one negotiate before totalling
    report_budget(grabbers)
 
    rows, cols = layout_shape(grabbers)
    print(f"{len(grabbers)} camera(s) in a {cols}x{rows} grid. "
          f"Press q in the window to quit.\n")
 
    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW, TILE_W * cols, TILE_H * rows)
    solo = None
    fullscreen = False
    last_report = last_scan = time.time()
 
    try:
        while True:
            if solo is not None and solo < len(grabbers):
                rows, cols = layout_shape(grabbers)
                canvas = cv2.resize(tile_for(grabbers[solo], solo),
                                    (TILE_W * cols, TILE_H * rows))
            else:
                canvas = compose(grabbers)
 
            cv2.imshow(WINDOW, canvas)
            now = time.time()
 
            # Pick up cameras plugged in after startup, or ones that dropped
            # off the bus and came back.
            if now - last_scan > RESCAN_EVERY:
                last_scan = now
                for port, path in find_cameras().items():
                    if port in known or len(grabbers) >= MAX_CAMERAS:
                        continue
                    g = Grabber(path, port)
                    print(f"Camera connected: {g.label}")
                    g.start()
                    grabbers.append(g)
                    grabbers.sort(key=lambda grabber: display_rank(grabber.port))
                    known.add(port)
                    rows, cols = layout_shape(grabbers)
                    cv2.resizeWindow(WINDOW, TILE_W * cols, TILE_H * rows)
 
            if now - last_report > 10:
                last_report = now
                print("fps  " + "  ".join(f"{g.port}:{g.fps:.1f}"
                                          for g in grabbers))
 
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            elif key == ord("f"):
                fullscreen = not fullscreen
                cv2.setWindowProperty(
                    WINDOW, cv2.WND_PROP_FULLSCREEN,
                    cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
            elif ord("1") <= key <= ord("9"):
                solo = key - ord("1")
            elif key == ord("0"):
                solo = None
    except KeyboardInterrupt:
        pass
    finally:
        for g in grabbers:
            g.stop()
        for g in grabbers:
            g.join(timeout=1.0)
        cv2.destroyAllWindows()
 
 
if __name__ == "__main__":
    main()
 
