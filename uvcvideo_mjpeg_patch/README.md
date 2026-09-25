# uvcvideo MJPEG bandwidth cap (Jetson Linux 38.2.x, kernel 6.8.12-tegra)

## Problem

The Global Shutter cameras (USB ID `32e4:0234`, AR0234 sensor) ask for the
largest USB 2.0 isochronous slot (3 x 1020 bytes per microframe, about
24 MB/s) in MJPEG at every resolution. All six cameras share one USB 2.0
link, so only two Global Shutter cameras can stream MJPEG at once. The rest
fail with this in `/var/log/kern.log`:

```
usb 1-4.1.4.3: Not enough bandwidth for altsetting 11
```

The stock `quirks=128` (UVC_QUIRK_FIX_BANDWIDTH) fixes this only for
uncompressed formats such as YUYV.

## Fix

`mjpeg_payload.patch` adds one module parameter, `mjpeg_payload`, that caps
the payload size a camera may request for compressed formats. The default
is 0 (off), which behaves exactly like the stock driver. Bulk-transfer
cameras (such as the HD USB camera) are not affected.

With `mjpeg_payload=800` each Global Shutter camera reserves the 800-byte
slot (about 6.4 MB/s, 5 cameras = about 53% of the bus). Verified: all six
stream MJPEG 1280x720@30 at a full 30 fps. A 720p frame is 110-140 KB, about
4 MB/s at 30 fps. The 1024 cap (1280-byte slot, about 85% of the bus) also
fits, but one camera then lost frames (about 10 fps). If a mode needs more
than the cap, frames are dropped and the frame rate falls; raise the value.

- `nvidia_r38.2.1_orig/`: NVIDIA's unmodified driver from the Jetson Linux
  38.2.1 public sources. Built unmodified, it matches the installed
  module's `srcversion` (D783047085E9ED77D1D6DCF), so it is the same code.
- `build/`: patched source and the built `uvcvideo.ko`.
- `check_all_cameras.py`: streams every camera at once and prints fps each.

## Build

Needs the kernel headers (`nvidia-l4t-kernel-headers`, already installed on
Jetson Linux) and gcc. The built `uvcvideo.ko` is not kept in git because it
only loads on the exact kernel it was built for.

```bash
cd /home/agxthor/CameraSetup/uvcvideo_mjpeg_patch/build
make -C /lib/modules/$(uname -r)/build M=$PWD CONFIG_USB_VIDEO_CLASS=m modules
modinfo ./uvcvideo.ko | grep mjpeg_payload      # the new parameter is listed
```

For another Jetson Linux release, take `drivers/media/usb/uvc/` from that
release's `public_sources.tbz2` (inside `kernel_src.tbz2`, under
`kernel/kernel-noble/`). Build it unchanged first and compare
`modinfo ./uvcvideo.ko | grep srcversion` with `modinfo uvcvideo`: equal
values mean it is the installed driver's source. Then apply the patch in that
folder with `patch -p1 < mjpeg_payload.patch` and build as above.

## Try it (until the next reboot)

Close every program that uses a camera, then:

```bash
sudo rmmod uvcvideo && sudo insmod /home/agxthor/CameraSetup/uvcvideo_mjpeg_patch/build/uvcvideo.ko quirks=128 mjpeg_payload=800
```

Check it:

```bash
/home/agxthor/SingleCameraBev/.venv/bin/python /home/agxthor/CameraSetup/uvcvideo_mjpeg_patch/check_all_cameras.py MJPG 640 480 20
```

Undo without rebooting: `sudo rmmod uvcvideo && sudo modprobe uvcvideo`

## Make it permanent

```bash
sudo install -D -m 644 /home/agxthor/CameraSetup/uvcvideo_mjpeg_patch/build/uvcvideo.ko /lib/modules/$(uname -r)/updates/uvcvideo.ko
sudo depmod -a
echo "options uvcvideo quirks=128 mjpeg_payload=800" | sudo tee /etc/modprobe.d/uvcvideo.conf
```

Undo:

```bash
sudo rm /lib/modules/$(uname -r)/updates/uvcvideo.ko && sudo depmod -a
echo "options uvcvideo quirks=128" | sudo tee /etc/modprobe.d/uvcvideo.conf
```

A JetPack/kernel update installs a new kernel directory, so the patched
module stops being used there; rebuild it against the new kernel's source.
