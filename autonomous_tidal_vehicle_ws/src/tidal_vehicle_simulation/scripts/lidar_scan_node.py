#!/usr/bin/env python3
"""
lidar_scan_node — /points (3D, 16-channel) → /scan (2D LaserScan) for the
autonomy workstream's near-field obstacle sensing (docs/INTERFACES.md).

For every azimuth bin it keeps the closest return whose height lies in a band
above the ground, so low roots and debris show up as well as trunks. Returns
that land on the vehicle itself (the rear fan ducts and payload box are
inside the LiDAR's lower beams) are removed with a vehicle-footprint box.
Output frame: lidar_link (axis-aligned with base_link).

Parameters (config/vehicle_mobility.yaml, section lidar_scan):
  min_height_m / max_height_m  band relative to the LiDAR (default -0.62 .. 0.5:
                               from ~7 cm above ground in hover mode to 1.2 m)
  self_box_m                   [xmin, xmax, ymin, ymax] vehicle box in lidar_link
  range_min_m / range_max_m, bins
"""
import math

import numpy as np


def cloud_to_ranges(xyz, bins=720, min_h=-0.62, max_h=0.5, rmin=0.3, rmax=30.0,
                    self_box=(-1.10, 0.25, -0.52, 0.52)):
    """xyz: (N,3) points in the lidar frame -> (ranges[bins], angle_min, increment).
    Pure numpy, no ROS, so it can be tested offline."""
    xyz = np.asarray(xyz, dtype=float)
    xyz = xyz[np.all(np.isfinite(xyz), axis=1)]
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    keep = (z >= min_h) & (z <= max_h)
    xb0, xb1, yb0, yb1 = self_box
    keep &= ~((x > xb0) & (x < xb1) & (y > yb0) & (y < yb1))      # own body
    r = np.hypot(x, y)
    keep &= (r >= rmin) & (r <= rmax)
    ranges = np.full(bins, np.inf)
    if keep.any():
        a = np.arctan2(y[keep], x[keep])
        idx = np.clip(((a + math.pi) / (2 * math.pi) * bins).astype(int), 0, bins - 1)
        np.minimum.at(ranges, idx, r[keep])
    return ranges, -math.pi, 2 * math.pi / bins


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan, PointCloud2
    from sensor_msgs_py import point_cloud2

    class LidarScan(Node):
        def __init__(self):
            super().__init__("lidar_scan")
            p = self.declare_parameter
            self.bins = int(p("bins", 720).value)
            self.min_h = p("min_height_m", -0.62).value
            self.max_h = p("max_height_m", 0.5).value
            self.rmin = p("range_min_m", 0.3).value
            self.rmax = p("range_max_m", 30.0).value
            self.box = tuple(p("self_box_m", [-1.10, 0.25, -0.52, 0.52]).value)
            self.pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            self.create_subscription(PointCloud2, "/points", self.on_cloud, qos_profile_sensor_data)

        def on_cloud(self, msg):
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            ranges, a0, inc = cloud_to_ranges(pts, self.bins, self.min_h, self.max_h,
                                              self.rmin, self.rmax, self.box)
            s = LaserScan()
            s.header = msg.header
            s.angle_min, s.angle_increment = a0, inc
            s.angle_max = a0 + inc * (self.bins - 1)
            s.scan_time, s.time_increment = 0.1, 0.0
            s.range_min, s.range_max = float(self.rmin), float(self.rmax)
            s.ranges = ranges.astype(np.float32).tolist()
            self.pub.publish(s)

    rclpy.init()
    node = LidarScan()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
