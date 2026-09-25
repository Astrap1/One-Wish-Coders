#!/usr/bin/env python3
"""
gen_description.py — build the vehicle description from the link_frames.json
that assets/vehicle_blender/build_vehicle.py exports:

  models/hovercraft/model.sdf + model.config   Gazebo model (physics, sensors, plugins)
  urdf/hovercraft.urdf                          ROS robot description (robot_state_publisher,
                                                RViz, Foxglove) with the SAME links/joints

    python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/scripts/gen_description.py
    python3 .../gen_description.py --no-render-sensors --out /tmp/x/model.sdf   # physics-only tests
    python3 .../gen_description.py --mesh glb                                   # force .glb visuals

Frame names follow config/sensors.yaml: the hull link is `base_link`, the
LiDAR `lidar_link`, the camera `camera_link`, and the IMU publishes in
`imu_link` (fixed at the hull origin).

Everything geometric (link origins, joint axes/limits, collision primitives,
masses) comes from link_frames.json, so the Blender model and the SDF can't
drift apart. Inertia tensors are computed here from the collision
primitives (box / cylinder formulas) and the Section 2 masses. Never leave
Gazebo's default inertia in place.

The hull's centre of mass is placed so that the whole vehicle's CoM sits
over the cushion centre (x = 0, y = 0). In practice that's where the
battery goes, and it lets the vehicle hover level.
"""
import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = "tidal_vehicle_description"
MODEL_DIR = HERE.parent / "models" / "hovercraft"
URDF_PATH = HERE.parent / "urdf" / "hovercraft.urdf"
PLUGIN_LIB = "tidal_vehicle_plugins"          # built by tidal_vehicle_simulation
G = 9.81

# Blender object name -> ROS / Gazebo link name (config/sensors.yaml frames)
RENAME = {"hull": "base_link", "lidar": "lidar_link", "camera": "camera_link"}
SENSOR_FRAME = {"lidar_3d": "lidar_link", "front_camera": "camera_link", "imu": "imu_link",
                "navsat": "base_link"}


def ln(name):
    return RENAME.get(name, name)


# --------------------------------------------------------------------------
# Inertia helpers (principal moments about the CoM, in the primitive's frame)
# --------------------------------------------------------------------------
def box_inertia(m, sx, sy, sz):
    return (m * (sy**2 + sz**2) / 12, m * (sx**2 + sz**2) / 12,
            m * (sx**2 + sy**2) / 12)


def cyl_inertia(m, r, l):
    """Cylinder along its local Z."""
    ia = m * r * r / 2
    ip = m * (3 * r * r + l * l) / 12
    return ip, ip, ia


def rotate_diag(ixyz, rpy):
    """Principal inertia of a primitive rotated by 90-degree roll/pitch.
    Only the axis permutations we use (roll pi/2: Y<->Z, pitch pi/2: X<->Z)."""
    ixx, iyy, izz = ixyz
    r, p, _ = rpy
    if abs(abs(r) - math.pi / 2) < 1e-6:
        iyy, izz = izz, iyy
    if abs(abs(p) - math.pi / 2) < 1e-6:
        ixx, izz = izz, ixx
    return ixx, iyy, izz


def link_inertia(mass, col, box_override=None):
    if box_override:
        return box_inertia(mass, *box_override)
    if col["type"] == "box":
        return box_inertia(mass, *col["size"])
    return rotate_diag(cyl_inertia(mass, col["radius"], col["length"]), col["pose"][3:])


# --------------------------------------------------------------------------
# SDF snippets
# --------------------------------------------------------------------------
def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def geometry(col):
    if col["type"] == "box":
        return f"<box><size>{fmt(col['size'])}</size></box>"
    return (f"<cylinder><radius>{col['radius']:.6g}</radius>"
            f"<length>{col['length']:.6g}</length></cylinder>")


FRICTION = {  # (mu, mu2)
    # skirt: low friction so a glancing contact slides off instead of snagging
    "wheel": (1.0, 1.0), "skirt": (0.2, 0.2), "hull": (0.3, 0.3), "default": (0.5, 0.5)}

NO_COLLISION = {"lift_fan", "thrust_fan_left", "thrust_fan_right",
                "rudder_left", "rudder_right"}          # cosmetic parts


def collision_xml(name, col):
    kind = ("wheel" if name.startswith("wheel_") and not name.startswith(("wheel_leg", "wheel_shock"))
            else "skirt" if name == "skirt" else "hull" if name == "hull" else "default")
    mu, mu2 = FRICTION[kind]
    return f"""
      <collision name="collision">
        <pose>{fmt(col['pose'])}</pose>
        <geometry>{geometry(col)}</geometry>
        <surface><friction><ode><mu>{mu}</mu><mu2>{mu2}</mu2></ode></friction></surface>
      </collision>"""


def sensors_xml(name, sensors, render_sensors):
    """Sensor elements attached to link `name` (poses relative to the link)."""
    out = []
    link_origin = LINKS[name]["origin_xyz"]
    for sname, s in sensors.items():
        if s["parent"] != name:
            continue
        rel = [s["xyz"][i] - link_origin[i] for i in range(3)]
        pose = f"<pose>{fmt(rel + s['rpy'])}</pose>"
        frame = SENSOR_FRAME.get(sname, ln(name))
        pose += f"<gz_frame_id>{frame}</gz_frame_id>"
        t = s["type"]
        if t == "imu":
            out.append(f"""
      <sensor name="{sname}" type="imu">{pose}
        <always_on>1</always_on><update_rate>50</update_rate><topic>__default__</topic>
      </sensor>""")
        elif t == "navsat":
            out.append(f"""
      <sensor name="{sname}" type="navsat">{pose}
        <always_on>1</always_on><update_rate>10</update_rate>
      </sensor>""")
        elif not render_sensors:
            continue
        elif sname == "lidar_3d":
            out.append(f"""
      <sensor name="{sname}" type="gpu_lidar">{pose}
        <always_on>1</always_on><update_rate>{s['rate_hz']}</update_rate><visualize>1</visualize>
        <lidar>
          <scan>
            <horizontal><samples>900</samples><resolution>1</resolution>
              <min_angle>-3.14159</min_angle><max_angle>3.14159</max_angle></horizontal>
            <vertical><samples>{s['channels']}</samples><resolution>1</resolution>
              <min_angle>{math.radians(s['vfov_deg'][0]):.5f}</min_angle>
              <max_angle>{math.radians(s['vfov_deg'][1]):.5f}</max_angle></vertical>
          </scan>
          <range><min>{s['range_m'][0]}</min><max>{s['range_m'][1]}</max><resolution>0.01</resolution></range>
          <noise><type>gaussian</type><mean>0</mean><stddev>0.01</stddev></noise>
        </lidar>
      </sensor>""")
        elif t == "camera":
            out.append(f"""
      <sensor name="{sname}" type="camera">{pose}
        <always_on>1</always_on><update_rate>15</update_rate>
        <camera><horizontal_fov>1.40</horizontal_fov>
          <image><width>640</width><height>480</height><format>R8G8B8</format></image>
          <clip><near>0.05</near><far>60</far></clip></camera>
      </sensor>""")
        elif sname.startswith("ranger_"):
            out.append(f"""
      <sensor name="{sname}" type="gpu_lidar">{pose}
        <always_on>1</always_on><update_rate>50</update_rate>
        <!-- 5 rays over +/-3 deg (a narrow ToF beam). A single-sample gpu_lidar
             returns wrong ranges in gz-sensors 8, so don't use samples = 1. -->
        <lidar><scan><horizontal><samples>5</samples><resolution>1</resolution>
          <min_angle>-0.05</min_angle><max_angle>0.05</max_angle></horizontal></scan>
          <range><min>0.01</min><max>2.0</max><resolution>0.001</resolution></range></lidar>
      </sensor>""")
    return "".join(out)


def joint_xml(name, e):
    j = e["joint"]
    jt = j["type"]
    sdf_type = {"continuous": "revolute"}.get(jt, jt)
    xml = f"""
    <joint name="{j['name']}" type="{sdf_type}">
      <parent>{ln(e['parent'])}</parent>
      <child>{ln(name)}</child>"""
    if jt != "fixed":
        if jt == "continuous":
            lim = "<lower>-1e16</lower><upper>1e16</upper>"
        else:
            lim = f"<lower>{j['lower']}</lower><upper>{j['upper']}</upper>"
        effort = {"revolute": 80, "prismatic": 600, "continuous": 15}[jt]
        if name.startswith(("lift_fan", "thrust_fan")):
            effort = 5
        if name.startswith("rudder"):
            effort = 5
        damping = 0.01 if jt == "continuous" else 0.5
        xml += f"""
      <axis>
        <xyz>{fmt(j['axis'])}</xyz>
        <limit>{lim}<effort>{effort}</effort></limit>
        <dynamics><damping>{damping}</damping></dynamics>
      </axis>"""
    return xml + "\n    </joint>"


def plugins_xml(links):
    corners = ["{:.3f} {:.3f} {:.3f}".format(sx * 0.42, sy * 0.20, SKIRT_BOTTOM_IN_HULL)
               for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
    hull_o = links["hull"]["origin_xyz"]
    tl = [links["thrust_fan_left"]["origin_xyz"][i] - hull_o[i] for i in range(3)]
    tr = [links["thrust_fan_right"]["origin_xyz"][i] - hull_o[i] for i in range(3)]
    legs = [n for n in links if n.startswith("wheel_leg_")]
    shocks = [n for n in links if n.startswith("wheel_shock_")]
    out = [f"""
    <!-- Air cushion + glide drag + fan thrust + mud resistance (tidal_vehicle_simulation) -->
    <plugin filename="{PLUGIN_LIB}" name="hover::AirCushion">
      <link_name>base_link</link_name>
      {''.join(f'<corner>{c}</corner>' for c in corners)}
      <target_gap>0.04</target_gap>
      <stiffness>1500</stiffness>
      <damping>130</damping>
      <max_force_factor>2.0</max_force_factor>
      <spinup_time>1.5</spinup_time>
      <glide_linear_drag>5</glide_linear_drag>
      <glide_quadratic_drag>6</glide_quadratic_drag>
      <glide_yaw_drag>4</glide_yaw_drag>
      <glide_lateral_factor>3</glide_lateral_factor>
      <max_thrust>30</max_thrust>
      <thrust_point_left>{fmt(tl)}</thrust_point_left>
      <thrust_point_right>{fmt(tr)}</thrust_point_right>
      <mud_viscous_drag>150</mud_viscous_drag>
      <mud_rolling_resistance>0.25</mud_rolling_resistance>
      <vent_length>0.05</vent_length>
      <rudder_wash_coeff>0.55</rudder_wash_coeff>
      <heading_kp>40</heading_kp><heading_kd>25</heading_kd>
      <!-- hover-mode velocity control on /model/<name>/cmd_vel_hover (gz.msgs.Twist) -->
      <speed_kp>40</speed_kp><speed_ki>10</speed_ki><yaw_rate_kp>30</yaw_rate_kp>
      <cmd_timeout>0.5</cmd_timeout>
      <use_raycast>true</use_raycast>
      <log_file>/tmp/hover_{{model}}.csv</log_file>
      {''.join(f'<log_joint>{links[n]["joint"]["name"]}</log_joint>' for n in links if n.startswith(("wheel_leg", "wheel_shock", "wheel_f", "wheel_r")))}
    </plugin>

    <!-- Ground-mode drive: 4-wheel skid steer on /model/<name>/cmd_vel. Its wheel
         odometry is kept internal; /odom comes from OdometryPublisher below. -->
    <plugin filename="gz-sim-diff-drive-system" name="gz::sim::systems::DiffDrive">
      <odom_topic>internal/wheel_odometry</odom_topic><tf_topic>internal/wheel_tf</tf_topic>
      <left_joint>wheel_fl_joint</left_joint><left_joint>wheel_rl_joint</left_joint>
      <right_joint>wheel_fr_joint</right_joint><right_joint>wheel_rr_joint</right_joint>
      <wheel_separation>{2 * links['wheel_fl']['origin_xyz'][1]:.3f}</wheel_separation>
      <wheel_radius>{WHEEL_R:.3f}</wheel_radius>
      <max_linear_velocity>2.0</max_linear_velocity>
      <max_angular_velocity>2.0</max_angular_velocity>
      <odom_publish_frequency>20</odom_publish_frequency>
    </plugin>
"""]
    up = links[legs[0]]["joint"]["upper"]
    out.append(f"""
    <!-- Leg retract: one controller per leg, all subscribed to the same topic
         /model/<name>/legs_cmd (gz.msgs.Double) so they get the command in the same step.
         0 = legs down (GROUND mode), {up:.4f} = folded (HOVER mode). -->""")
    for leg in legs:
        j = links[leg]["joint"]
        out.append(f"""
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{j['name']}</joint_name>
      <sub_topic>legs_cmd</sub_topic>
      <use_velocity_commands>true</use_velocity_commands>
      <p_gain>4.0</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>
      <cmd_max>1.2</cmd_max><cmd_min>-1.2</cmd_min>
      <initial_position>0</initial_position>
    </plugin>""")
    for sh in shocks:
        j = links[sh]["joint"]
        out.append(f"""
    <!-- Suspension spring-damper ({sh}): k = {j['stiffness']} N/m, c = {j['damping']} N s/m -->
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{j['name']}</joint_name>
      <p_gain>{j['stiffness']}</p_gain><i_gain>0</i_gain><d_gain>{j['damping']}</d_gain>
      <cmd_max>600</cmd_max><cmd_min>-600</cmd_min>
      <initial_position>{j['spring_reference']}</initial_position>
    </plugin>""")
    for r in ("rudder_left", "rudder_right"):
        out.append(f"""
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{r}_joint</joint_name><sub_topic>{r}_cmd</sub_topic>
      <p_gain>2</p_gain><d_gain>0.05</d_gain>
      <cmd_max>2</cmd_max><cmd_min>-2</cmd_min>
    </plugin>""")
    out.append(f"""
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>

    <!-- /odom + TF (odom frame -> base_link). Ground-truth pose, valid in BOTH modes
         (wheel odometry is meaningless while hovering). Idealised: stated in docs. -->
    <plugin filename="gz-sim-odometry-publisher-system" name="gz::sim::systems::OdometryPublisher">
      <odom_frame>{ODOM_FRAME}</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <dimensions>3</dimensions>
      <odom_publish_frequency>50</odom_publish_frequency>
    </plugin>""")
    return "".join(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mesh", choices=["auto", "dae", "glb"], default="auto")
    ap.add_argument("--no-render-sensors", action="store_true",
                    help="omit gpu_lidar/camera sensors (physics-only / headless tests)")
    ap.add_argument("--out", default=str(MODEL_DIR / "model.sdf"))
    ap.add_argument("--odom-frame", default="map",
                    help="frame_id of /odom and parent of base_link in TF. Default 'map' so "
                         "/odom, /terrain_costmap and /mission_goal share one frame "
                         "(ground-truth odometry, so map == odom). See docs/INTERFACES.md")
    ap.add_argument("--no-urdf", action="store_true")
    args = ap.parse_args()

    data = json.loads((MODEL_DIR / "link_frames.json").read_text())
    global LINKS, SKIRT_BOTTOM_IN_HULL, WHEEL_R, ODOM_FRAME
    ODOM_FRAME = args.odom_frame
    LINKS = data["links"]
    hull_o = LINKS["hull"]["origin_xyz"]
    skirt = LINKS["skirt"]
    SKIRT_BOTTOM_IN_HULL = skirt["origin_xyz"][2] + skirt["bbox_local"]["min"][2] - hull_o[2]
    WHEEL_R = LINKS["wheel_fl"]["collision"]["radius"]

    # Hull CoM: place it so the whole-vehicle CoM is over the cushion centre
    others = [(n, e) for n, e in LINKS.items() if n != "hull"]
    m_hull = LINKS["hull"]["mass_kg"]
    m_tot = sum(e["mass_kg"] for e in LINKS.values())

    def com_world(e):
        c = e["collision"]["pose"][:3]
        return [e["origin_xyz"][i] + c[i] for i in range(3)]

    sx = sum(e["mass_kg"] * com_world(e)[0] for _, e in others)
    hull_com_x = -sx / m_hull - hull_o[0]
    hull_com = [hull_com_x, 0.0, -0.03]        # batteries sit low in the hull
    global HULL_COM
    HULL_COM = hull_com

    links_xml = []
    for name, e in LINKS.items():
        col = e["collision"]
        m = e["mass_kg"]
        if name == "hull":
            ixx, iyy, izz = box_inertia(m, 1.10, 0.62, 0.20)
            com = hull_com
        else:
            ixx, iyy, izz = link_inertia(m, col)
            com = col["pose"][:3]
        mesh = e["mesh"]
        ext = ("dae" if "dae" in mesh else "glb") if args.mesh == "auto" else args.mesh
        visual = ""
        if ext in mesh:
            visual = f"""
      <visual name="visual">
        <geometry><mesh><uri>model://hovercraft/{mesh[ext]}</uri></mesh></geometry>
      </visual>"""
        coll = "" if name in NO_COLLISION else collision_xml(name, col)
        links_xml.append(f"""
    <link name="{ln(name)}">
      <pose>{fmt(e['origin_xyz'])} 0 0 0</pose>
      <inertial>
        <pose>{fmt(com)} 0 0 0</pose>
        <mass>{m}</mass>
        <inertia><ixx>{ixx:.6g}</ixx><ixy>0</ixy><ixz>0</ixz>
                 <iyy>{iyy:.6g}</iyy><iyz>0</iyz><izz>{izz:.6g}</izz></inertia>
      </inertial>{visual}{coll}{sensors_xml(name, data['sensors'], not args.no_render_sensors)}
    </link>""")

    joints_xml = [joint_xml(n, e) for n, e in LINKS.items() if e.get("joint")]

    sdf = f"""<?xml version="1.0"?>
<!-- GENERATED by tidal_vehicle_description/scripts/gen_description.py from link_frames.json. Do not edit by hand. -->
<!-- Model frame: ground plane under hull centre; vehicle standing on its wheels. -->
<!-- Total mass {m_tot:.2f} kg; hull CoM offset {fmt(hull_com)} (vehicle CoM over cushion centre). -->
<sdf version="1.9">
  <model name="hovercraft">
    <self_collide>false</self_collide>
{''.join(links_xml)}
{''.join(joints_xml)}
{plugins_xml(LINKS)}
  </model>
</sdf>
"""
    Path(args.out).write_text(sdf)
    (MODEL_DIR / "model.config").write_text("""<?xml version="1.0"?>
<model>
  <name>hovercraft</name>
  <version>0.2</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>One Wish Coders</name></author>
  <description>Unmanned air-cushion logistics vehicle: air cushion + retractable wheel legs with coil-over suspension, sealed payload box, 3D LiDAR, camera, IMU, GNSS. Generated from Blender (assets/vehicle_blender/build_vehicle.py).</description>
</model>
""")
    if not args.no_urdf:
        write_urdf(LINKS, ext_pref=args.mesh)
    # Report
    print(f"wrote {args.out}")
    print(f"links {len(LINKS)}, joints {len(joints_xml)}, total mass {m_tot:.2f} kg")
    print(f"hull CoM (hull frame) {fmt(hull_com)}; skirt bottom in hull frame {SKIRT_BOTTOM_IN_HULL:.3f} m")
    print(f"render sensors: {'off' if args.no_render_sensors else 'on'}; visual meshes: {args.mesh}")


# --------------------------------------------------------------------------
# URDF (ROS robot description) — same links, joints and origins as the SDF
# --------------------------------------------------------------------------
def write_urdf(links, ext_pref="auto"):
    out = ['<?xml version="1.0"?>',
           '<!-- GENERATED by gen_description.py from link_frames.json. Do not edit by hand. -->',
           '<robot name="hovercraft">']
    for name, e in links.items():
        mesh = e["mesh"]
        ext = ("dae" if "dae" in mesh else "glb") if ext_pref == "auto" else ext_pref
        vis = ""
        if ext in mesh:
            vis = (f'<visual><geometry><mesh filename="package://{PKG}/models/hovercraft/'
                   f'{mesh[ext]}"/></geometry></visual>')
        col = e["collision"]
        g = (f'<box size="{fmt(col["size"])}"/>' if col["type"] == "box" else
             f'<cylinder radius="{col["radius"]}" length="{col["length"]}"/>')
        c = col["pose"]
        coll = (f'<collision><origin xyz="{fmt(c[:3])}" rpy="{fmt(c[3:])}"/>'
                f'<geometry>{g}</geometry></collision>')
        m = e["mass_kg"]
        ixx, iyy, izz = (box_inertia(m, 1.10, 0.62, 0.20) if name == "hull"
                         else link_inertia(m, col))
        com = HULL_COM if name == "hull" else col["pose"][:3]
        inert = (f'<inertial><origin xyz="{fmt(com)}" rpy="0 0 0"/><mass value="{m}"/>'
                 f'<inertia ixx="{ixx:.6g}" ixy="0" ixz="0" iyy="{iyy:.6g}" iyz="0" izz="{izz:.6g}"/>'
                 f'</inertial>')
        out.append(f'  <link name="{ln(name)}">{inert}{vis}{coll}</link>')
    out.append('  <link name="imu_link"/>')
    out.append('  <joint name="imu_joint" type="fixed"><parent link="base_link"/>'
               '<child link="imu_link"/><origin xyz="0 0 0" rpy="0 0 0"/></joint>')
    for name, e in links.items():
        j = e.get("joint")
        if not j:
            continue
        o = e["origin_in_parent"]
        xml = (f'  <joint name="{j["name"]}" type="{j["type"]}"><parent link="{ln(e["parent"])}"/>'
               f'<child link="{ln(name)}"/><origin xyz="{fmt(o)}" rpy="0 0 0"/>')
        if j["type"] != "fixed":
            xml += f'<axis xyz="{fmt(j["axis"])}"/>'
        if j["type"] in ("revolute", "prismatic"):
            xml += f'<limit lower="{j["lower"]}" upper="{j["upper"]}" effort="100" velocity="2"/>'
        out.append(xml + "</joint>")
    out.append("</robot>")
    URDF_PATH.parent.mkdir(parents=True, exist_ok=True)
    URDF_PATH.write_text("\n".join(out) + "\n")
    print(f"wrote {URDF_PATH}")


if __name__ == "__main__":
    main()
