"""Stream all connected cameras at once in one mode and report delivered fps each."""
import glob
import re
import sys
import threading
import time

import cv2

fourcc, w, h, fps = sys.argv[1], int(sys.argv[2]), int(sys.argv[3]), int(sys.argv[4])
paths = sorted(glob.glob("/dev/v4l/by-path/*-usb-0:*video-index0"))
counts, caps = {}, []
for p in paths:
    port = re.search(r"-0:([0-9.]+):", p).group(1)
    cap = cv2.VideoCapture(p, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
    cap.set(cv2.CAP_PROP_FPS, fps)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 4)
    caps.append((port, cap))
    counts[port] = 0
    time.sleep(0.3)

stop = False


def run(port, cap):
    while not stop:
        ok, _ = cap.read()
        if ok:
            counts[port] += 1
        else:
            time.sleep(0.01)


threads = [threading.Thread(target=run, args=c, daemon=True) for c in caps]
for t in threads:
    t.start()
time.sleep(3)
base = dict(counts)
t0 = time.time()
time.sleep(5)
dt = time.time() - t0
live = sum(1 for p in counts if counts[p] > base[p])
print(f"{fourcc} {w}x{h}@{fps}: {live}/{len(counts)} live  "
      + "  ".join(f"{p}={(counts[p] - base[p]) / dt:4.1f}" for p in counts), flush=True)
stop = True
time.sleep(0.3)
for _, c in caps:
    c.release()
