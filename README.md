# Six USB cameras on Jetson AGX Thor

Live viewer and USB driver fix for running six UVC cameras at once on a Jetson
AGX Thor (Jetson Linux 38.2.x, kernel 6.8.12-tegra). With the patched
`uvcvideo` driver in [`uvcvideo_mjpeg_patch/`](uvcvideo_mjpeg_patch/), all six
stream MJPEG at **1280x720, 30 fps each**, on one USB 2.0 link.

## Demo

[![All six cameras live while driving](media/preview.jpg)](https://github.com/LittleWebCat/thor-six-usb-cameras/releases/download/v1.0/six_cameras_720p_drive.mp4)

[Download the demo video](https://github.com/LittleWebCat/thor-six-usb-cameras/releases/download/v1.0/six_cameras_720p_drive.mp4) (136 MB, from the [v1.0 release](https://github.com/LittleWebCat/thor-six-usb-cameras/releases/tag/v1.0)): a
7.8-minute drive recorded from the viewer window, all six cameras at about
30 fps. Top row: left front, narrow front, right front. Bottom row: left back,
wide front, right back.

## Cameras

Cameras are identified by USB port (from `/dev/v4l/by-path/`), because
`/dev/videoN` numbers change between boots.

| Port | Role | Camera | USB ID | Modes used |
|---|---|---|---|---|
| 4.1.4.3 | left front | Global Shutter (AR0234) | 32e4:0234 | 1280x720 MJPG @30 |
| 4.1.4.2 | narrow front | HD USB Camera | 32e4:8830 | 1024x768 MJPG @30 (no 720p mode) |
| 4.1.4.4 | right front | Global Shutter (AR0234) | 32e4:0234 | 1280x720 MJPG @30 |
| 4.1.3.3 | left back | Global Shutter (AR0234) | 32e4:0234 | 1280x720 MJPG @30 |
| 4.1.3.4 | wide front | Global Shutter (AR0234) | 32e4:0234 | 1280x720 MJPG @30 |
| 4.1.2.4 | right back | Global Shutter (AR0234) | 32e4:0234 | 1280x720 MJPG @30 |

All six hang off one hub chain on the Thor's USB 2.0 root port `1-4`
(480 Mbit/s shared). The Global Shutter cameras are USB 2.0 devices
(`bcdUSB 2.00`), so a USB 3 port does not add bandwidth.

## Quick start

```bash
python3 -m venv camvenv
camvenv/bin/pip install -r requirements.txt
sudo apt install v4l-utils          # v4l2-ctl, used for frame rate and exposure
```

Live view of every camera (the window opens on the Thor's display):

```bash
DISPLAY=:1 camvenv/bin/python cameras.py MJPG 1280 720 30
```

With no arguments the viewer uses YUYV 320x240 @20, which works with the stock
driver. Any `FOURCC WIDTH HEIGHT FPS` can be passed on the command line.

Keys in the window: `1`-`6` show one camera large, `0` returns to the grid,
`f` toggles full screen, `q` quits.

The grid follows `CAMERA_ORDER` in `cameras.py`: front cameras on the top row
(left front, narrow front, right front), rear and wide on the bottom (left
back, wide front, right back). Each tile shows its role, port and live fps.

## What limits six cameras, and the fix

In MJPEG the Global Shutter cameras ask for the largest USB 2.0 isochronous
slot (3 x 1020 bytes per microframe, about 24 MB/s) at every resolution. Only
two fit on the shared link, so the rest show `NO FRAMES`, and the kernel log
says:

```
usb 1-4.1.4.3: Not enough bandwidth for altsetting 11
```

The stock bandwidth quirk (`quirks=128` in `/etc/modprobe.d/uvcvideo.conf`)
only applies to uncompressed formats. The patched driver adds a
`mjpeg_payload` parameter that caps the MJPEG request; with
`mjpeg_payload=800` each camera reserves about 6.4 MB/s, while a 720p frame
stream actually needs about 4 MB/s. The HD USB camera uses bulk transfers,
which reserve nothing, so it is never the one that fails.

All six cameras streaming at once:

| Mode | Stock driver | Patched, `mjpeg_payload=800` |
|---|---|---|
| MJPG 320x240 @20 | 3 / 6 | 6 / 6 |
| MJPG 640x480 @30 | 3 / 6 | 6 / 6 at 30 fps |
| **MJPG 1280x720 @30** | 3 / 6 | **6 / 6 at 30 fps** |
| MJPG 1920x1080 @30 | 3 / 6 | 5 / 6 (the USB 2.0 link is full) |
| YUYV 320x240 @20 | 6 / 6 | 6 / 6 |
| YUYV 640x480 @10 | 5 / 6 | 5 / 6 |

Build, install and undo steps: [`uvcvideo_mjpeg_patch/README.md`](uvcvideo_mjpeg_patch/README.md).
Check that the patched driver is active:

```bash
cat /sys/module/uvcvideo/parameters/mjpeg_payload     # 800; missing = stock driver
```

## Exposure and frame rate

- All cameras run on **auto exposure**. The viewer selects it explicitly on
  every open, because a camera keeps its last exposure mode until it is
  unplugged, even when another program set it. A fixed 30 ms exposure left
  up to 26% of pixels pure white outdoors. For a fixed exposure on one
  camera, add it to `OVERRIDES` (units of 0.1 ms), for example
  `"4.1.3.4": {"exposure": 100, "gain": 20}`.
- `BUFFERS = 4` driver buffers per camera. With 1 buffer the cameras drop
  every other frame (20 fps becomes 10).
- The viewer sets the frame rate on the device with `v4l2-ctl` and prints the
  rate the device actually accepted, since some cameras ignore OpenCV's
  request.

## Troubleshooting

- **A tile says `NO FRAMES`:** run
  `grep "Not enough bandwidth" /var/log/kern.log | tail`. If it lists that
  camera, the patched driver is not loaded (see the check above), or the
  chosen mode needs more than the link has.
- **The driver cannot be swapped (`Module uvcvideo is in use`):** close every
  program using a camera. `fuser -v /dev/video*` shows which one.
- **After a JetPack or kernel update:** the patched module lives in the old
  kernel's `updates/` folder, so the stock driver comes back. Rebuild it
  against the new kernel as described in the patch README.

## Files

| Path | Contents |
|---|---|
| `cameras.py` | Live viewer for every connected camera |
| `requirements.txt` | Python packages for the viewer |
| `uvcvideo_mjpeg_patch/` | Driver patch, NVIDIA's original source, patched source, `check_all_cameras.py` |
| `camtest.txt`, `hdcam.txt` | `v4l2-ctl` format and control dumps of the two camera models |
| `media/preview.jpg` | Frame from the demo video (the video itself is on the v1.0 release) |
| `cameras_working.py` | Earlier version of the viewer, kept for reference |
