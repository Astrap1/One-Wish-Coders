#!/usr/bin/env python3
"""
lidar_scan_node — /points (3D, 16-channel) → /scan (2D LaserScan) for the
autonomy workstream's near-field obstacle sensing (docs/INTERFACES.md).

For every azimuth bin it keeps the closest return whose height lies in a band
above the ground, so low roots and debris show up as well as trunks. Returns
that land on the vehicle itself (the rear fan ducts and payload box are
inside the LiDAR's lower beams) are removed with a vehicle-footprint box and,
for vehicles whose corners extend beyond that box, an optional footprint
radius.
Output frame: lidar_link (axis-aligned with base_link).

Parameters (config/vehicle_mobility.yaml, section lidar_scan):
  min_height_m / max_height_m  band relative to the LiDAR (default -0.62 .. 0.5:
                               from ~7 cm above ground in hover mode to 1.2 m)
  self_box_m                   [xmin, xmax, ymin, ymax] vehicle box in lidar_link
  self_radius_m                optional circular vehicle envelope (0 = disabled)
  corridor_ground_filter       remove returns on the known corridor terrain
  lidar_height_m               lidar_link height above base_link
  ground_clearance_m           required protrusion above the terrain surface
  ground_obstacle_height_m     top of the terrain-relative obstacle slice (0 = off)
  range_min_m / range_max_m, bins
"""
import math

import numpy as np


def cloud_to_ranges(
    xyz,
    bins=720,
    min_h=-0.62,
    max_h=0.5,
    rmin=0.3,
    rmax=30.0,
    self_box=(-1.10, 0.25, -0.52, 0.52),
    self_radius=0.0,
    height_above_ground=None,
    ground_clearance=0.0,
    ground_obstacle_height=None,
):
    """xyz: (N,3) points in the lidar frame -> (ranges[bins], angle_min, increment).
    Pure numpy, no ROS, so it can be tested offline."""
    xyz = np.asarray(xyz, dtype=float)
    finite = np.all(np.isfinite(xyz), axis=1)
    if height_above_ground is not None:
        height_above_ground = np.asarray(height_above_ground, dtype=float)
        if len(height_above_ground) != len(xyz):
            raise ValueError("height_above_ground must match the point count")
        height_above_ground = height_above_ground[finite]
    xyz = xyz[finite]
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    keep = (z >= min_h) & (z <= max_h)
    if height_above_ground is not None:
        keep &= height_above_ground > ground_clearance
        if ground_obstacle_height is not None:
            keep &= height_above_ground <= ground_obstacle_height
    xb0, xb1, yb0, yb1 = self_box
    r = np.hypot(x, y)
    # Boundaries are part of the body too.  The old strict comparison leaked
    # returns exactly on the configured V3 hull edge.  A radius is optional so
    # V1/V2 keep their existing rectangular filter while V3 can cover its
    # complete 1.75 m half-diagonal plus a small modelling margin.
    in_self_box = (x >= xb0) & (x <= xb1) & (y >= yb0) & (y <= yb1)
    in_self_radius = (self_radius > 0.0) & (r <= self_radius)
    keep &= ~(in_self_box | in_self_radius)
    keep &= (r >= rmin) & (r <= rmax)
    ranges = np.full(bins, np.inf)
    if keep.any():
        a = np.arctan2(y[keep], x[keep])
        idx = np.clip(((a + math.pi) / (2 * math.pi) * bins).astype(int), 0, bins - 1)
        np.minimum.at(ranges, idx, r[keep])
    return ranges, -math.pi, 2 * math.pi / bins


def corridor_terrain_height(x):
    """Vectorised copy of the tidal corridor's authoritative ground profile."""
    x = np.asarray(x, dtype=float)
    return np.select(
        [
            (x <= 4.0) | (x >= 100.0),
            x < 7.7320508,
            x < 50.0,
            x <= 54.0,
            x < 96.2679492,
        ],
        [
            0.0,
            -(x - 4.0) * 0.2679491924,
            -1.0 - (x - 7.7320508) * (2.0 / 42.2679492),
            -3.0,
            -3.0 + (x - 54.0) * (2.0 / 42.2679492),
        ],
        default=-1.0 + (x - 96.2679492) * 0.2679491924,
    )


def height_above_corridor_ground(
    xyz,
    base_position,
    base_orientation,
    lidar_height,
):
    """Return each LiDAR point's world height above the corridor surface."""
    points = np.asarray(xyz, dtype=float)
    heights = np.full(len(points), np.inf)
    finite = np.all(np.isfinite(points), axis=1)
    if not finite.any():
        return heights
    points = points[finite].copy()
    points[:, 2] += lidar_height
    qx, qy, qz, qw = base_orientation
    rotation = np.array([
        [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw),
         2.0 * (qx * qz + qy * qw)],
        [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz),
         2.0 * (qy * qz - qx * qw)],
        [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw),
         1.0 - 2.0 * (qx * qx + qy * qy)],
    ])
    world = points @ rotation.T + np.asarray(base_position, dtype=float)
    heights[finite] = world[:, 2] - corridor_terrain_height(world[:, 0])
    return heights


def main():
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from nav_msgs.msg import Odometry
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
            self.radius = float(p("self_radius_m", 0.0).value)
            self.ground_filter = bool(p("corridor_ground_filter", False).value)
            self.lidar_height = float(p("lidar_height_m", 0.0).value)
            self.ground_clearance = float(p("ground_clearance_m", 0.0).value)
            self.ground_obstacle_height = float(
                p("ground_obstacle_height_m", 0.0).value
            )
            self.odom = None
            self.pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            self.create_subscription(PointCloud2, "/points", self.on_cloud, qos_profile_sensor_data)
            if self.ground_filter:
                self.create_subscription(Odometry, "/odom", self.on_odom, 20)

        def on_odom(self, msg):
            self.odom = msg

        def on_cloud(self, msg):
            pts = point_cloud2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            above_ground = None
            if self.ground_filter:
                if self.odom is None:
                    return
                position = self.odom.pose.pose.position
                orientation = self.odom.pose.pose.orientation
                above_ground = height_above_corridor_ground(
                    pts,
                    (position.x, position.y, position.z),
                    (orientation.x, orientation.y, orientation.z, orientation.w),
                    self.lidar_height,
                )
            ranges, a0, inc = cloud_to_ranges(pts, self.bins, self.min_h, self.max_h,
                                              self.rmin, self.rmax, self.box,
                                              self.radius, above_ground,
                                              self.ground_clearance,
                                              (self.ground_obstacle_height
                                               if self.ground_obstacle_height > 0.0
                                               else None))
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
