"""Live 7-camera grid with a top-down Ouster LiDAR view beside it.

Reuses cameras.py for the cameras (layout, exposure, mirroring) and draws the
/points cloud from above: 40 m ahead, 20 m behind, 10 m to each side, with
distance rings every 10 m and points coloured by height.

Needs ROS 2 (rclpy), so run it with the system Python in the LiDAR driver's
ROS environment:

    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
    DISPLAY=:1 /usr/bin/python3 cameras_lidar.py

Keys: q quits, f toggles fullscreen.
"""

import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

import cameras

TOPIC = "/points"
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


class CloudListener(Node):
    """Keeps only the newest point cloud as an (N, 3) float32 array."""

    def __init__(self):
        super().__init__("cameras_lidar_view")
        self.lock = threading.Lock()
        self.points = None
        self.stamp = 0.0
        self._rate = 0.0
        self.create_subscription(PointCloud2, TOPIC, self._on_cloud, qos_profile_sensor_data)

    def _on_cloud(self, msg):
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
    rclpy.init()
    listener = CloudListener()
    spinner = threading.Thread(target=rclpy.spin, args=(listener,), daemon=True)
    spinner.start()

    grabbers = []
    for port, path in cameras.find_cameras().items():
        grabber = cameras.Grabber(path, port)
        grabber.start()
        grabbers.append(grabber)
        time.sleep(cameras.OPEN_STAGGER)
    print(f"{len(grabbers)} camera(s); LiDAR topic {TOPIC}. Press q in the window to quit.")

    cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
    fullscreen = False
    try:
        while True:
            grid = cameras.compose(grabbers)
            points, stamp, rate = listener.latest()
            canvas = np.hstack([grid, lidar_panel(points, stamp, rate, grid.shape[0])])
            cv2.imshow(WINDOW, canvas)
            key = cv2.waitKey(15) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("f"):
                fullscreen = not fullscreen
                cv2.setWindowProperty(WINDOW, cv2.WND_PROP_FULLSCREEN,
                                      cv2.WINDOW_FULLSCREEN if fullscreen else cv2.WINDOW_NORMAL)
    except KeyboardInterrupt:
        pass
    finally:
        for grabber in grabbers:
            grabber.stop()
        for grabber in grabbers:
            grabber.join(timeout=1.0)
        cv2.destroyAllWindows()
        listener.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
