"""Live 7-camera grid with a top-down Ouster LiDAR view beside it, and recording.

Reuses cameras.py for the cameras (layout, exposure, mirroring) and draws the
/points cloud from above: 40 m ahead, 20 m behind, 10 m to each side, with
distance rings every 10 m and points coloured by height.

Needs ROS 2 (rclpy), so run it with the system Python in the LiDAR driver's
ROS environment:

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    DISPLAY=:1 /usr/bin/python3 cameras_lidar.py            # add --record to start recording

Keys: r starts/stops recording, f toggles fullscreen, q quits.

A recording is one folder, recordings/YYYYMMDD_HHMMSS/:
    meta.json                 ports, roles, capture modes, LiDAR mounting
    cameras.csv               camera, port, index, wall/monotonic time, file
    cameras/<role>/NNNNNN.jpg every new frame from each camera (not mirrored)
    lidar.csv                 index, ROS stamp, wall/monotonic time, points, file
    lidar/NNNNNN.npz          xyz (float32, LiDAR frame) and intensity (uint16)
    lidar_imu.csv             Ouster IMU at 100 Hz (LiDAR frame)
LiDAR data stays in the LiDAR's own frame; turn it by LIDAR_YAW_DEG to get
the car frame (x forward, y left).
"""

import argparse
import csv
import json
import os
import queue
import shutil
import threading
import time
import traceback
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2

import cameras

TOPIC = "/points"
IMU_TOPIC = "/imu"
FORWARD_M = 40.0      # shown ahead of the LiDAR
REAR_M = 20.0         # shown behind it
SIDE_M = 10.0         # shown to each side
RING_EVERY_M = 10.0
MIN_RANGE_M = 1.0     # drop returns from the car itself
Z_MIN, Z_MAX = -2.0, 2.0   # height colour range, metres relative to the LiDAR
# The LiDAR is mounted facing backwards on the car, so its points are turned
# 180 degrees about the vertical axis to put the car's front at the top.
LIDAR_YAW_DEG = 180.0
WINDOW = "Cameras + LiDAR"

RECORD_ROOT = Path(__file__).resolve().parent / "recordings"
JPEG_QUALITY = 90
MIN_FREE_GB = 5.0         # recording stops itself below this much free disk
WRITER_THREADS = 4
QUEUE_LIMIT = 600         # items waiting for disk; beyond this, new items are dropped


def stamp_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec


class Recorder:
    """Writes camera frames, LiDAR scans and LiDAR IMU samples to one folder.

    Frames and scans are written by background threads so the display never
    waits for the disk. If the disk falls behind, new items are dropped and
    counted rather than piling up in memory.
    """

    def __init__(self):
        self.active = False
        self.folder = None
        self.started = 0.0
        self.counts = {"camera": 0, "lidar": 0, "imu": 0}
        self.dropped = 0
        self._queue = queue.Queue(maxsize=QUEUE_LIMIT)
        self._csv_lock = threading.Lock()
        self._files = {}
        self._writers = {}
        self._frame_index = {}
        self._lidar_index = 0
        self._workers = []
        self._log = None
        self._stall_reported = False

    def log(self, text):
        """Print an event and, while recording, append it to the folder's log.txt."""
        line = f"{time.strftime('%H:%M:%S')} {text}"
        print(line, flush=True)
        if self._log is not None:
            try:
                self._log.write(line + "\n")
                self._log.flush()
            except ValueError:
                pass

    # -- control -----------------------------------------------------------
    def start(self, grabbers):
        if self.active:
            return
        RECORD_ROOT.mkdir(parents=True, exist_ok=True)
        if shutil.disk_usage(RECORD_ROOT).free / 1e9 < MIN_FREE_GB:
            print(f"Not recording: less than {MIN_FREE_GB:g} GB free")
            return
        self.folder = RECORD_ROOT / time.strftime("%Y%m%d_%H%M%S")
        (self.folder / "cameras").mkdir(parents=True)
        (self.folder / "lidar").mkdir()
        meta = {
            "started_wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "cameras": [
                {"port": g.port, "role": cameras.ROLES.get(g.port, g.name),
                 "folder": f"cameras/{self._role_dir(g.port)}", "model": g.name,
                 "mode": g.mode, "displayed_mirrored": g.port in cameras.MIRRORED}
                for g in grabbers
            ],
            "lidar": {"topic": TOPIC, "imu_topic": IMU_TOPIC, "frame": "LiDAR",
                      "yaw_to_car_deg": LIDAR_YAW_DEG,
                      "points": "xyz float32 metres, intensity uint16"},
        }
        (self.folder / "meta.json").write_text(json.dumps(meta, indent=2))
        for name, filename, header in (
            ("camera", "cameras.csv",
             ["camera", "port", "index", "wall_time_ns", "monotonic_ns", "file"]),
            ("lidar", "lidar.csv",
             ["index", "ros_stamp_ns", "wall_time_ns", "monotonic_ns", "points", "file"]),
            ("imu", "lidar_imu.csv",
             ["ros_stamp_ns", "wall_time_ns", "ax", "ay", "az", "gx", "gy", "gz"]),
        ):
            handle = open(self.folder / filename, "w", newline="")
            self._files[name] = handle
            self._writers[name] = csv.writer(handle)
            self._writers[name].writerow(header)
        self.counts = {"camera": 0, "lidar": 0, "imu": 0}
        self.dropped = 0
        self._frame_index = {}
        self._lidar_index = 0
        self._workers = [threading.Thread(target=self._write_loop, daemon=True)
                         for _ in range(WRITER_THREADS)]
        for worker in self._workers:
            worker.start()
        self._log = open(self.folder / "log.txt", "a")
        self.started = time.time()
        self.active = True
        self.log(f"Recording to {self.folder}")

    def stop(self):
        if not self.active:
            return
        self.active = False
        for _ in self._workers:
            self._queue.put(None)
        for worker in self._workers:
            worker.join()
        with self._csv_lock:
            for handle in self._files.values():
                handle.close()
            self._files, self._writers = {}, {}
        seconds = time.time() - self.started
        self.log(f"Stopped recording after {seconds:.0f} s: {self.counts['camera']} camera frames, "
                 f"{self.counts['lidar']} LiDAR scans, {self.counts['imu']} IMU samples, "
                 f"{self.dropped} dropped -> {self.folder}")
        self._log.close()
        self._log = None

    def toggle(self, grabbers):
        self.stop() if self.active else self.start(grabbers)

    # -- inputs ------------------------------------------------------------
    @staticmethod
    def _role_dir(port):
        return cameras.ROLES.get(port, port).replace(" ", "_")

    def add_frame(self, grabber, frame, wall_time):
        if not self.active:
            return
        index = self._frame_index.get(grabber.port, 0)
        self._frame_index[grabber.port] = index + 1
        self._put(("camera", grabber.port, index, int(wall_time * 1e9), time.monotonic_ns(), frame))

    def add_scan(self, ros_stamp, xyz, intensity):
        if not self.active:
            return
        index = self._lidar_index
        self._lidar_index += 1
        self._put(("lidar", index, ros_stamp, time.time_ns(), time.monotonic_ns(), xyz, intensity))

    def add_imu(self, msg):
        if not self.active:
            return
        a, g = msg.linear_acceleration, msg.angular_velocity
        with self._csv_lock:
            if self.active and "imu" in self._writers:
                self._writers["imu"].writerow([stamp_ns(msg), time.time_ns(),
                                               a.x, a.y, a.z, g.x, g.y, g.z])
                self.counts["imu"] += 1

    def _put(self, item):
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self.dropped += 1

    # -- disk ----------------------------------------------------------------
    def _write_loop(self):
        while True:
            item = self._queue.get()
            if item is None:
                return
            try:
                if item[0] == "camera":
                    _, port, index, wall_ns, mono_ns, frame = item
                    rel = f"cameras/{self._role_dir(port)}/{index:06d}.jpg"
                    path = self.folder / rel
                    path.parent.mkdir(exist_ok=True)
                    cv2.imwrite(str(path), frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                    row = [cameras.ROLES.get(port, port), port, index, wall_ns, mono_ns, rel]
                else:
                    _, index, ros_ns, wall_ns, mono_ns, xyz, intensity = item
                    rel = f"lidar/{index:06d}.npz"
                    np.savez(self.folder / rel, xyz=xyz, intensity=intensity)
                    row = [index, ros_ns, wall_ns, mono_ns, len(xyz), rel]
                with self._csv_lock:
                    if item[0] in self._writers:
                        self._writers[item[0]].writerow(row)
                        self.counts[item[0]] += 1
            except Exception as exc:          # a full disk, say: count it, keep going
                self.dropped += 1
                self.log(f"recording write failed: {exc}")

    def check_disk(self):
        if self.active and shutil.disk_usage(RECORD_ROOT).free / 1e9 < MIN_FREE_GB:
            print(f"Less than {MIN_FREE_GB:g} GB free: stopping the recording")
            self.stop()

    def banner(self, canvas, lidar_age):
        if not self.active:
            return
        elapsed = int(time.time() - self.started)
        text = (f"REC {elapsed // 60:02d}:{elapsed % 60:02d}   cams {self.counts['camera']}   "
                f"lidar {self.counts['lidar']}   imu {self.counts['imu']}   dropped {self.dropped}"
                f"   queue {self._queue.qsize()}")
        if lidar_age > 1.0:
            text += f"   LIDAR STALLED {lidar_age:.0f}s"
            if not self._stall_reported:
                self._stall_reported = True
                self.log(f"LiDAR stalled: no scan for {lidar_age:.1f} s")
        elif self._stall_reported:
            self._stall_reported = False
            self.log("LiDAR resumed")
        cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 34), (0, 0, 170), -1)
        cv2.circle(canvas, (18, 17), 8, (255, 255, 255), -1)
        cv2.putText(canvas, text, (34, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                    (255, 255, 255), 2, cv2.LINE_AA)


class CloudListener(Node):
    """Keeps the newest point cloud for display and hands every scan to the recorder."""

    def __init__(self, recorder):
        super().__init__("cameras_lidar_view")
        self.recorder = recorder
        self.lock = threading.Lock()
        self.points = None
        self.stamp = 0.0
        self._rate = 0.0
        self.create_subscription(PointCloud2, TOPIC, self._safe_cloud, qos_profile_sensor_data)
        self.create_subscription(Imu, IMU_TOPIC, self._safe_imu, qos_profile_sensor_data)

    # An exception escaping a callback ends rclpy.spin, which silently stops
    # all LiDAR and IMU data, so every callback error is logged and survived.
    def _safe_cloud(self, msg):
        try:
            self._on_cloud(msg)
        except Exception:
            self.recorder.log("LiDAR callback error:\n" + traceback.format_exc())

    def _safe_imu(self, msg):
        try:
            self.recorder.add_imu(msg)
        except Exception:
            self.recorder.log("IMU callback error:\n" + traceback.format_exc())

    def _on_cloud(self, msg):
        if self.recorder.active:
            cloud = point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"),
                                             skip_nans=True)
            xyz = np.stack([cloud["x"], cloud["y"], cloud["z"]], axis=1).astype(np.float32)
            keep = np.any(xyz != 0.0, axis=1)                # Ouster marks no-return as 0,0,0
            xyz = xyz[keep]
            intensity = np.clip(cloud["intensity"][keep], 0, 65535).astype(np.uint16)
            self.recorder.add_scan(stamp_ns(msg), xyz, intensity)
        else:
            xyz = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
        now = time.time()
        with self.lock:
            if self.stamp:
                self._rate = 0.8 * self._rate + 0.2 / max(now - self.stamp, 1e-3)
            self.points = xyz.astype(np.float32, copy=False)
            self.stamp = now

    def latest(self):
        with self.lock:
            return self.points, self.stamp, self._rate


def frame_watcher(grabbers, recorder, running):
    """Hands every new camera frame (not just the displayed ones) to the recorder."""
    last = {g.port: 0.0 for g in grabbers}
    while running.is_set():
        if recorder.active:
            for g in grabbers:
                stamp = g.last_frame_at
                if stamp != last[g.port]:
                    last[g.port] = stamp
                    frame = g.read()
                    if frame is not None:
                        recorder.add_frame(g, frame, stamp)
        time.sleep(0.002)


def lidar_panel(points, stamp, rate, height):
    """Top-down view: forward is up, the car's left is on the left."""
    ppm = height / (FORWARD_M + REAR_M)          # pixels per metre
    width = int(round(2 * SIDE_M * ppm))
    panel = np.full((height, width, 3), 15, np.uint8)
    ox, oy = width / 2.0, FORWARD_M * ppm          # LiDAR position in the image

    # Distance rings and labels.
    for r in np.arange(RING_EVERY_M, max(FORWARD_M, REAR_M) + 0.1, RING_EVERY_M):
        cv2.circle(panel, (int(ox), int(oy)), int(r * ppm), (70, 70, 70), 1, cv2.LINE_AA)
        cv2.putText(panel, f"{r:.0f} m", (int(ox) + 4, int(oy - r * ppm) + 14),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (150, 150, 150), 1, cv2.LINE_AA)
    cv2.line(panel, (int(ox), 0), (int(ox), height), (45, 45, 45), 1)
    cv2.line(panel, (0, int(oy)), (width, int(oy)), (45, 45, 45), 1)

    stale = points is None or time.time() - stamp > 1.0
    if points is not None and len(points):
        x, y, z = points[:, 0], points[:, 1], points[:, 2]   # LiDAR frame
        yaw = np.deg2rad(LIDAR_YAW_DEG)                      # to car frame: x forward, y left
        x, y = x * np.cos(yaw) - y * np.sin(yaw), x * np.sin(yaw) + y * np.cos(yaw)
        keep = (np.hypot(x, y) > MIN_RANGE_M) & (x < FORWARD_M) & (x > -REAR_M) & (np.abs(y) < SIDE_M)
        x, y, z = x[keep], y[keep], z[keep]
        u = (ox - y * ppm).astype(np.int32)
        v = (oy - x * ppm).astype(np.int32)
        inside = (u >= 0) & (u < width) & (v >= 0) & (v < height)
        u, v, z = u[inside], v[inside], z[inside]
        level = np.clip((z - Z_MIN) / (Z_MAX - Z_MIN) * 255, 0, 255).astype(np.uint8)
        colours = cv2.applyColorMap(level.reshape(-1, 1), cv2.COLORMAP_TURBO).reshape(-1, 3)
        order = np.argsort(z)                     # draw higher points last
        panel[v[order], u[order]] = colours[order]

    # Ego marker: the LiDAR, pointing forward.
    tri = np.array([[ox, oy - 12], [ox - 7, oy + 7], [ox + 7, oy + 7]], np.int32)
    cv2.fillPoly(panel, [tri], (255, 255, 255))

    cv2.rectangle(panel, (0, 0), (width, 30), (0, 0, 0), -1)
    status = "NO DATA" if stale else f"{rate:4.1f} Hz  {0 if points is None else len(points):,} pts"
    colour = (90, 90, 235) if stale else (120, 235, 120)
    cv2.putText(panel, f"LIDAR top-down  {FORWARD_M:.0f} m ahead   {status}", (8, 21),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, colour, 1, cv2.LINE_AA)
    return panel


def main():
    parser = argparse.ArgumentParser(description="7-camera + LiDAR live view and recorder")
    parser.add_argument("--record", action="store_true", help="start recording at launch")
    args = parser.parse_args()

    recorder = Recorder()
    rclpy.init()
    listener = CloudListener(recorder)
    spinner = threading.Thread(target=rclpy.spin, args=(listener,), daemon=True)
    spinner.start()

    grabbers = []
    for port, path in cameras.find_cameras().items():
        grabber = cameras.Grabber(path, port)
        grabber.start()
        grabbers.append(grabber)
        time.sleep(cameras.OPEN_STAGGER)
    print(f"{len(grabbers)} camera(s); LiDAR topic {TOPIC}. "
          f"Press r in the window to record, q to quit.")

    running = threading.Event()
    running.set()
    watcher = threading.Thread(target=frame_watcher, args=(grabbers, recorder, running), daemon=True)
    watcher.start()
    if args.record:
        recorder.start(grabbers)

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    fullscreen = False
    last_disk_check = time.time()
    try:
        while True:
            grid = cameras.compose(grabbers)
            points, stamp, rate = listener.latest()
            canvas = np.hstack([grid, lidar_panel(points, stamp, rate, grid.shape[0])])
            recorder.banner(canvas, time.time() - stamp if stamp else 999.0)
            cv2.imshow(WINDOW, canvas)
            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                recorder.toggle(grabbers)
            if key == ord("f"):
                fullscreen = not fullscreen
                cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
            if time.time() - last_disk_check > 5.0:
                last_disk_check = time.time()
                recorder.check_disk()
    except KeyboardInterrupt:
        pass
    finally:
        recorder.stop()
        running.clear()
        for grabber in grabbers:
            grabber.stop()
        for grabber in grabbers:
            grabber.join(timeout=1.0)
        cv2.destroyAllWindows()
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
