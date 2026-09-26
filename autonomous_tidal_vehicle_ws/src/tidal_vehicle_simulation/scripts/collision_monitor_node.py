#!/usr/bin/env python3
"""
collision_monitor_node.py — Version 3 collision detection (Person 4).

Publishes tidal_vehicle_interfaces/msg/Collision on /vehicle/collision when
the vehicle hits something. Two independent detectors:

  contact   Gazebo contact sensors on the hull, skirt and track collisions
            (bridged to /vehicle/contacts/<part>). Ground contact is ignored:
            a contact whose surface normal is mostly vertical is the vehicle
            resting or sliding on the ground or a slope. So is contact
            between the vehicle's own parts.
  imu       a sudden horizontal jolt: the horizontal acceleration departs from
            its recent (low-pass) value by more than jolt_threshold_mps2.
            Catches hits the contact sensors miss and would also work on a
            real vehicle.

Each (source, part, other) is reported at most once per repeat_s. The side is
taken from the contact point in the vehicle frame (or the jolt direction).
"""
import math

HALF_LENGTH, HALF_BEAM = 1.5, 0.9          # Version 3 footprint (m)


def side_of(dx: float, dy: float) -> str:
    """front / rear / left / right for a point (dx, dy) in the vehicle frame,
    normalised by the footprint so corners pick the nearer face."""
    if abs(dx) / HALF_LENGTH >= abs(dy) / HALF_BEAM:
        return "front" if dx >= 0 else "rear"
    return "left" if dy >= 0 else "right"


def is_ground_normal(nz: float, max_vertical: float = 0.7) -> bool:
    """A mostly vertical normal means ground or slope support, not a hit."""
    return abs(nz) > max_vertical


def jolt_side(ax: float, ay: float) -> str:
    """A hit on the front decelerates the vehicle (-x jolt), and so on."""
    return side_of(-ax * HALF_LENGTH, -ay * HALF_BEAM)


def model_of(scoped: str) -> str:
    return scoped.split("::", 1)[0] if scoped else ""


def main() -> None:
    import rclpy
    from nav_msgs.msg import Odometry
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from ros_gz_interfaces.msg import Contacts
    from sensor_msgs.msg import Imu
    from tidal_vehicle_interfaces.msg import Collision

    class CollisionMonitor(Node):
        def __init__(self) -> None:
            super().__init__("collision_monitor")
            p = lambda n, d: self.declare_parameter(n, d).value
            self.model = str(p("model_name", "hovercraft_v3"))
            self.parts = list(p("parts", ["hull", "skirt", "track_left", "track_right"]))
            self.jolt_threshold = float(p("jolt_threshold_mps2", 6.0))
            self.lowpass_s = float(p("jolt_lowpass_s", 0.5))
            self.repeat_s = float(p("repeat_s", 1.0))
            self.pub = self.create_publisher(Collision, "/vehicle/collision", 10)
            self.pose = (0.0, 0.0, 0.0)
            self.a_lp = None
            self.t_prev = None
            self.last = {}
            self.last_contact_t = -1e9
            for part in self.parts:
                self.create_subscription(
                    Contacts, f"/vehicle/contacts/{part}",
                    lambda m, part=part: self.on_contacts(part, m), 10)
            self.create_subscription(Imu, "/imu", self.on_imu, qos_profile_sensor_data)
            self.create_subscription(Odometry, "/odom", self.on_odom, 10)
            self.get_logger().info(f"collision monitor: {self.model}, parts {self.parts}")

        def now_s(self) -> float:
            return self.get_clock().now().nanoseconds * 1e-9

        def on_odom(self, m: Odometry) -> None:
            q = m.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
            self.pose = (m.pose.pose.position.x, m.pose.pose.position.y, yaw)

        def report(self, source, part, side, other, strength) -> None:
            key = (source, part, other)
            t = self.now_s()
            if t - self.last.get(key, -1e9) < self.repeat_s:
                return
            self.last[key] = t
            c = Collision()
            c.header.stamp = self.get_clock().now().to_msg()
            c.header.frame_id = "base_link"
            c.source, c.part, c.side, c.other = source, part, side, other
            c.strength = float(strength)
            self.pub.publish(c)
            self.get_logger().warn(f"COLLISION ({source}): {part} {side} hit '{other}' "
                                   f"strength {strength:.3f}")

        def on_contacts(self, part: str, m) -> None:
            x0, y0, yaw = self.pose
            cy, sy = math.cos(yaw), math.sin(yaw)
            for c in m.contacts:
                n1, n2 = c.collision1.name, c.collision2.name
                mine1, mine2 = model_of(n1) == self.model, model_of(n2) == self.model
                if mine1 and mine2:
                    continue                                  # own parts touching
                other = model_of(n2 if mine1 else n1)
                for i, pt in enumerate(c.positions):
                    nz = c.normals[i].z if i < len(c.normals) else 1.0
                    if is_ground_normal(nz):
                        continue
                    dx, dy = pt.x - x0, pt.y - y0
                    bx, by = cy * dx + sy * dy, -sy * dx + cy * dy
                    depth = abs(c.depths[i]) if i < len(c.depths) else 0.0
                    self.last_contact_t = self.now_s()
                    self.report("contact", part, side_of(bx, by), other, depth)
                    break

        def on_imu(self, m: Imu) -> None:
            t = self.now_s()
            a = (m.linear_acceleration.x, m.linear_acceleration.y)
            if self.a_lp is None or self.t_prev is None:
                self.a_lp, self.t_prev = a, t
                return
            dt = max(0.0, t - self.t_prev)
            self.t_prev = t
            k = min(1.0, dt / max(self.lowpass_s, 1e-3))
            dx, dy = a[0] - self.a_lp[0], a[1] - self.a_lp[1]
            self.a_lp = (self.a_lp[0] + k * dx, self.a_lp[1] + k * dy)
            jolt = math.hypot(dx, dy)
            # the contact sensors name the part when they fire; the IMU is the backup
            if jolt > self.jolt_threshold and t - self.last_contact_t > 0.5:
                self.report("imu", "unknown", jolt_side(dx, dy), "", jolt)

    rclpy.init()
    node = CollisionMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
