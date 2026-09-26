#!/usr/bin/env python3
"""
lidar_scan_node — /points (3D, 16-channel) → /scan (2D LaserScan) for the
autonomy workstream's near-field obstacle sensing (docs/INTERFACES.md).

For every azimuth bin it keeps the closest return whose height lies in a band
above the ground, so low roots and debris show up as well as trunks. Returns
that land on the vehicle itself (the rear fan ducts and payload box are
inside the LiDAR's lower beams) are removed with an inclusive vehicle-footprint
box and, where configured, a circular envelope. The latter is used by Version
3 so returns from its hull, skirt and outboard corners cannot create an
obstacle immediately around itself.
Output frame: lidar_link (axis-aligned with base_link).

Parameters (config/vehicle_mobility.yaml, section lidar_scan):
  min_height_m / max_height_m  band relative to the LiDAR (default -0.62 .. 0.5:
                               from ~7 cm above ground in hover mode to 1.2 m)
  self_box_m                   [xmin, xmax, ymin, ymax] inclusive vehicle box
  self_footprint_radius_m      optional circular self-return envelope (metres)
  self_radius_m                legacy alias for self_footprint_radius_m
  corridor_ground_filter       remove returns on the known corridor terrain
  lidar_height_m               lidar_link height above base_link
  ground_clearance_m           required protrusion above the terrain surface
  ground_obstacle_height_m     top of the terrain-relative obstacle slice (0 = off)
  range_min_m / range_max_m, bins

Alternative ground filter (Person 4; OFF by default, kept for worlds whose
terrain profile is not built in -- the corridor filter above is the active one):
  slope_ground_filter_deg      along each bearing, a return that rises from the
  slope_ground_step_m          ground before it by no more than this slope (plus
                               a small step) is ground; anything steeper is an
                               obstacle. Needs no terrain map. 0 = off.
  level_with_imu               measure those slopes level, using the IMU's roll
                               and pitch (/imu), so a tilted vehicle does not
                               see level ground as a slope
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
    self_footprint_radius=None,
    height_above_ground=None,
    ground_clearance=0.0,
    ground_obstacle_height=None,
    *,
    exclude=None,
):
    """xyz: (N,3) points in the lidar frame -> (ranges[bins], angle_min, increment).
    Pure numpy, no ROS, so it can be tested offline."""
    xyz = np.asarray(xyz, dtype=float)
    finite = np.all(np.isfinite(xyz), axis=1)
    if exclude is not None:                    # e.g. slope_ground_mask() below
        finite &= ~np.asarray(exclude, dtype=bool)
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
    # Boundaries are part of the body too. V1/V2 retain their rectangular
    # filter; V3 adds a circular corner envelope with model tolerance.
    in_self_box = (x >= xb0) & (x <= xb1) & (y >= yb0) & (y <= yb1)
    if self_radius < 0.0:
        raise ValueError("self_radius must be non-negative")
    if self_footprint_radius is not None:
        if self_footprint_radius < 0.0:
            raise ValueError("self_footprint_radius must be non-negative")
        self_radius = max(self_radius, self_footprint_radius)
    in_self_radius = (self_radius > 0.0) & (r <= self_radius)
    keep &= ~(in_self_box | in_self_radius)
    keep &= (r >= rmin) & (r <= rmax)
    ranges = np.full(bins, np.inf)
    if keep.any():
        a = np.arctan2(y[keep], x[keep])
        idx = np.clip(((a + math.pi) / (2 * math.pi) * bins).astype(int), 0, bins - 1)
        np.minimum.at(ranges, idx, r[keep])
    return ranges, -math.pi, 2 * math.pi / bins


def level_points(xyz, roll, pitch):
    """Rotate lidar-frame points by the vehicle's roll and pitch (radians,
    REP-103) into a level frame with the same heading."""
    cr, sr, cp, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    return np.asarray(xyz, dtype=float) @ (ry @ rx).T


def slope_ground_mask(xyz, bins, max_slope_deg, lidar_height, step=0.15, attitude=None):
    """True for returns that belong to the ground, by slope rather than by a
    terrain map. Along each bearing the returns are walked outwards from the
    ground under the vehicle (lidar_height below the LiDAR): a return is ground
    if it rises from the last ground return by no more than max_slope_deg (plus
    `step` for small bumps); falling away is always ground. attitude = (roll,
    pitch) levels the points first. Non-finite points are never ground."""
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    ground = np.zeros(len(xyz), dtype=bool)
    fin = np.nonzero(np.all(np.isfinite(xyz), axis=1))[0]
    if len(fin) == 0:
        return ground
    pts = xyz[fin] if attitude is None else level_points(xyz[fin], *attitude)
    n = len(pts)
    r = np.hypot(pts[:, 0], pts[:, 1])
    b = np.clip(((np.arctan2(pts[:, 1], pts[:, 0]) + math.pi) / (2 * math.pi) * bins)
                .astype(int), 0, bins - 1)
    order = np.lexsort((r, b))
    bs, rs, zs = b[order], r[order], pts[order, 2]
    first = np.r_[True, bs[1:] != bs[:-1]]
    rank = np.arange(n) - np.maximum.accumulate(np.where(first, np.arange(n), 0))
    last_r = np.zeros(bins)
    last_z = np.full(bins, -float(lidar_height))
    tan_s = math.tan(math.radians(max_slope_deg))
    g_sorted = np.zeros(n, dtype=bool)
    for k in range(int(rank.max()) + 1):
        sel = np.nonzero(rank == k)[0]
        bb = bs[sel]
        dr = np.maximum(rs[sel] - last_r[bb], 1e-3)
        is_g = zs[sel] - last_z[bb] <= tan_s * dr + step
        g_sorted[sel] = is_g
        upd = sel[is_g]
        last_r[bs[upd]] = rs[upd]
        last_z[bs[upd]] = zs[upd]
    g = np.zeros(n, dtype=bool)
    g[order] = g_sorted
    ground[fin] = g
    return ground


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
            self.radius = max(
                float(p("self_radius_m", 0.0).value),
                float(p("self_footprint_radius_m", 0.0).value),
            )
            self.ground_filter = bool(p("corridor_ground_filter", False).value)
            self.lidar_height = float(p("lidar_height_m", 0.0).value)
            self.ground_clearance = float(p("ground_clearance_m", 0.0).value)
            self.ground_obstacle_height = float(
                p("ground_obstacle_height_m", 0.0).value
            )
            self.slope_deg = float(p("slope_ground_filter_deg", 0.0).value)
            self.slope_step = float(p("slope_ground_step_m", 0.15).value)
            self.level_imu = bool(p("level_with_imu", False).value)
            self.attitude = None
            self.odom = None
            self.pub = self.create_publisher(LaserScan, "/scan", qos_profile_sensor_data)
            self.create_subscription(PointCloud2, "/points", self.on_cloud, qos_profile_sensor_data)
            if self.ground_filter:
                self.create_subscription(Odometry, "/odom", self.on_odom, 20)
            if self.slope_deg > 0.0 and self.level_imu:
                from sensor_msgs.msg import Imu
                self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)

        def on_imu(self, msg):
            q = msg.orientation
            self.attitude = (
                math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)),
                math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))),
            )

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
            exclude = None
            if self.slope_deg > 0.0:
                exclude = slope_ground_mask(pts, self.bins, self.slope_deg,
                                            self.lidar_height, self.slope_step,
                                            self.attitude if self.level_imu else None)
            # Use names for optional filters: the footprint-radius alias
            # precedes height_above_ground in cloud_to_ranges(), so positional
            # arguments here can turn the scalar clearance into an invalid
            # height array and kill the live V3 LiDAR process.
            ranges, a0, inc = cloud_to_ranges(
                pts,
                bins=self.bins,
                min_h=self.min_h,
                max_h=self.max_h,
                rmin=self.rmin,
                rmax=self.rmax,
                self_box=self.box,
                self_radius=self.radius,
                height_above_ground=above_ground,
                ground_clearance=self.ground_clearance,
                ground_obstacle_height=(
                    self.ground_obstacle_height
                    if self.ground_obstacle_height > 0.0
                    else None
                ),
                exclude=exclude,
            )
            s = LaserScan()
            s.header = msg.header
            s.angle_min, s.angle_increment = a0, inc
            s.angle_max = a0 + inc * (self.bins - 1)
            s.scan_time, s.time_increment = 0.1, 0.0
            s.range_min, s.range_max = float(self.rmin), float(self.rmax)
            s.ranges = ranges.astype(np.float32).tolist()
            self.pub.publish(s)

    from rclpy.executors import ExternalShutdownException

    rclpy.init()
    node = LidarScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
