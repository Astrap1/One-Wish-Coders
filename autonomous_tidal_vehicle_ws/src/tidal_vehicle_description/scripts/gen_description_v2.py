#!/usr/bin/env python3
"""
gen_description_v2.py — build the Version 2 vehicle description from the
link_frames.json that assets/vehicle_blender/version_2/build_vehicle.py exports:

  models/hovercraft_v2/model.sdf + model.config   Gazebo model (physics, sensors, plugins)
  urdf/hovercraft_v2.urdf                          ROS robot description (robot_state_publisher,
                                                   RViz, Foxglove) with the SAME links/joints

    python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_description/scripts/gen_description_v2.py
    python3 .../gen_description_v2.py --no-render-sensors --out /tmp/x/model.sdf   # physics-only tests

Version 2 is a hovercraft-dominant amphibious vehicle with a retractable
tracked undercarriage and controlled air-cushion load sharing (AGENTS.md):

  * two inboard track links on prismatic retract joints. Gazebo's
    TrackController moves each track's contact surface and TrackedVehicle
    turns a Twist on /model/hovercraft_v2/cmd_vel into skid-steer track speeds;
  * hover::AirCushion (tidal_vehicle_simulation) with its gains scaled from
    the tested Version 1 values to the 300 kg vehicle, including the
    lift_share input used for TRACK-mode load sharing;
  * the same sensor frames as Version 1 (config/sensors.yaml): base_link,
    lidar_link, camera_link, imu_link.

Everything geometric comes from link_frames.json. Inertia tensors are
computed here from the collision primitives and the masses. The hull CoM is
placed so that the whole vehicle's CoM sits over the cushion centre in x.
Plugin numbers are stated simulation assumptions, not measured data.
"""
import argparse
import json
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
PKG = "tidal_vehicle_description"
MODEL = "hovercraft_v2"
MODEL_DIR = HERE.parent / "models" / MODEL
URDF_PATH = HERE.parent / "urdf" / f"{MODEL}.urdf"
PLUGIN_LIB = "tidal_vehicle_plugins"          # built by tidal_vehicle_simulation

# Blender object name -> ROS / Gazebo link name (config/sensors.yaml frames)
RENAME = {"hull": "base_link", "lidar": "lidar_link", "camera": "camera_link"}
SENSOR_FRAME = {"lidar_3d": "lidar_link", "front_camera": "camera_link", "imu": "imu_link",
                "navsat": "base_link"}

# Stated simulation assumptions (AGENTS.md "Vehicle Version 2")
MAX_THRUST_N = 200.0            # per ducted fan
TRACK_MAX_SPEED = 1.5           # m/s, TRACK mode
TRACK_MAX_YAW = 1.0             # rad/s, TRACK mode
RETRACT_SPEED = 0.2             # m/s, track retract / deploy
RETRACT_EFFORT = 6000.0         # N, holds the vehicle's weight on one track with margin


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
    """Principal inertia of a primitive rotated by 90-degree roll/pitch."""
    ixx, iyy, izz = ixyz
    r, p, _ = rpy
    if abs(abs(r) - math.pi / 2) < 1e-6:
        iyy, izz = izz, iyy
    if abs(abs(p) - math.pi / 2) < 1e-6:
        ixx, izz = izz, ixx
    return ixx, iyy, izz


def prims(col):
    return col if isinstance(col, list) else [col]


def link_inertia(mass, col):
    """Inertia of the link's main (first) collision primitive."""
    c = prims(col)[0]
    if c["type"] == "box":
        return box_inertia(mass, *c["size"])
    return rotate_diag(cyl_inertia(mass, c["radius"], c["length"]), c["pose"][3:])


def fmt(v):
    return " ".join(f"{x:.6g}" for x in v)


def geometry(c):
    if c["type"] == "box":
        return f"<box><size>{fmt(c['size'])}</size></box>"
    return (f"<cylinder><radius>{c['radius']:.6g}</radius>"
            f"<length>{c['length']:.6g}</length></cylinder>")


FRICTION = {  # (mu, mu2)
    # tracks: rubber on soil; skirt: low friction so a glancing contact slides off
    "track": (1.0, 1.0), "skirt": (0.2, 0.2), "hull": (0.3, 0.3), "default": (0.5, 0.5)}

NO_COLLISION = {"lift_fan", "thrust_fan_left", "thrust_fan_right",
                "rudder_left", "rudder_right"}          # cosmetic parts


def collision_xml(name, col):
    kind = ("track" if name.startswith("track_") else "skirt" if name == "skirt"
            else "hull" if name == "hull" else "default")
    mu, mu2 = FRICTION[kind]
    out = []
    for i, c in enumerate(prims(col)):
        cname = "collision" if i == 0 else f"collision_{i}"
        out.append(f"""
      <collision name="{cname}">
        <pose>{fmt(c['pose'])}</pose>
        <geometry>{geometry(c)}</geometry>
        <surface><friction><ode><mu>{mu}</mu><mu2>{mu2}</mu2></ode></friction></surface>
      </collision>""")
    return "".join(out)


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
        effort = {"revolute": 80, "prismatic": RETRACT_EFFORT, "continuous": 15}[jt]
        if name.startswith(("lift_fan", "thrust_fan", "rudder")):
            effort = 20
        damping = 0.01 if jt == "continuous" else (200.0 if jt == "prismatic" else 0.5)
        xml += f"""
      <axis>
        <xyz>{fmt(j['axis'])}</xyz>
        <limit>{lim}<effort>{effort}</effort></limit>
        <dynamics><damping>{damping}</damping></dynamics>
      </axis>"""
    return xml + "\n    </joint>"


def plugins_xml(links, data):
    hull_o = links["hull"]["origin_xyz"]
    cush = data["cushion"]
    skirt_bottom_in_hull = cush["skirt_bottom_track_mode"] - hull_o[2]
    cx, cy = cush["corner_xy"]
    corners = ["{:.3f} {:.3f} {:.3f}".format(sx * cx * 0.85, sy * cy * 0.80, skirt_bottom_in_hull)
               for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1))]
    tl = [links["thrust_fan_left"]["origin_xyz"][i] - hull_o[i] for i in range(3)]
    tr = [links["thrust_fan_right"]["origin_xyz"][i] - hull_o[i] for i in range(3)]
    rud = [links["rudder_left"]["origin_xyz"][0] - hull_o[0], 0.0,
           links["rudder_left"]["origin_xyz"][2] - hull_o[2]]
    trk = data["tracks"]
    out = [f"""
    <!-- Air cushion + glide drag + fan thrust + mud resistance (tidal_vehicle_simulation).
         Gains are the tested Version 1 values scaled to the 300 kg vehicle: same
         cushion natural frequency and damping ratio, drag sized for a ~3 m/s top
         speed at 2 x {MAX_THRUST_N:.0f} N, yaw gains scaled with the yaw inertia. -->
    <plugin filename="{PLUGIN_LIB}" name="hover::AirCushion">
      <link_name>base_link</link_name>
      {''.join(f'<corner>{c}</corner>' for c in corners)}
      <target_gap>{cush['hover_gap']}</target_gap>
      <stiffness>18000</stiffness>
      <damping>800</damping>
      <max_force_factor>2.0</max_force_factor>
      <spinup_time>2.0</spinup_time>
      <glide_linear_drag>50</glide_linear_drag>
      <glide_quadratic_drag>25</glide_quadratic_drag>
      <glide_yaw_drag>200</glide_yaw_drag>
      <glide_lateral_factor>3</glide_lateral_factor>
      <max_thrust>{MAX_THRUST_N:.0f}</max_thrust>
      <thrust_time_constant>0.4</thrust_time_constant>
      <thrust_point_left>{fmt(tl)}</thrust_point_left>
      <thrust_point_right>{fmt(tr)}</thrust_point_right>
      <rudder_point>{fmt(rud)}</rudder_point>
      <mud_viscous_drag>1800</mud_viscous_drag>
      <mud_rolling_resistance>0.25</mud_rolling_resistance>
      <mud_yaw_drag>1000</mud_yaw_drag>
      <water_hull_drag>720</water_hull_drag>
      <water_yaw_drag>500</water_yaw_drag>
      <vent_length>0.06</vent_length>
      <rudder_wash_coeff>0.55</rudder_wash_coeff>
      <heading_kp>600</heading_kp><heading_kd>400</heading_kd>
      <!-- hover-mode velocity control on /model/<name>/cmd_vel_hover (gz.msgs.Twist) -->
      <speed_kp>480</speed_kp><speed_ki>120</speed_ki><yaw_rate_kp>800</yaw_rate_kp>
      <cmd_timeout>0.5</cmd_timeout>
      <use_raycast>true</use_raycast>
      <log_file>/tmp/hover_{{model}}.csv</log_file>
      <log_joint>track_left_joint</log_joint><log_joint>track_right_joint</log_joint>
    </plugin>

    <!-- TRACK mode drive: skid steer on /model/<name>/cmd_vel (gz.msgs.Twist).
         TrackController moves each track's contact surface; its odometry is kept
         internal because /odom comes from OdometryPublisher below. -->
    <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
      <link>track_left</link>
      <min_velocity>-{TRACK_MAX_SPEED + TRACK_MAX_YAW * trk['gauge'] / 2:.2f}</min_velocity>
      <max_velocity>{TRACK_MAX_SPEED + TRACK_MAX_YAW * trk['gauge'] / 2:.2f}</max_velocity>
    </plugin>
    <plugin filename="gz-sim-track-controller-system" name="gz::sim::systems::TrackController">
      <link>track_right</link>
      <min_velocity>-{TRACK_MAX_SPEED + TRACK_MAX_YAW * trk['gauge'] / 2:.2f}</min_velocity>
      <max_velocity>{TRACK_MAX_SPEED + TRACK_MAX_YAW * trk['gauge'] / 2:.2f}</max_velocity>
    </plugin>
    <plugin filename="gz-sim-tracked-vehicle-system" name="gz::sim::systems::TrackedVehicle">
      <left_track><link>track_left</link></left_track>
      <right_track><link>track_right</link></right_track>
      <tracks_separation>{trk['gauge']:.3f}</tracks_separation>
      <tracks_height>{2 * trk['radius']:.3f}</tracks_height>
      <steering_efficiency>0.5</steering_efficiency>
      <linear_velocity><min_velocity>-{TRACK_MAX_SPEED}</min_velocity><max_velocity>{TRACK_MAX_SPEED}</max_velocity>
        <min_acceleration>-1.5</min_acceleration><max_acceleration>1.0</max_acceleration></linear_velocity>
      <angular_velocity><min_velocity>-{TRACK_MAX_YAW}</min_velocity><max_velocity>{TRACK_MAX_YAW}</max_velocity>
        <min_acceleration>-2.0</min_acceleration><max_acceleration>2.0</max_acceleration></angular_velocity>
      <odom_topic>internal/track_odometry</odom_topic><tf_topic>internal/track_tf</tf_topic>
      <odom_publish_frequency>20</odom_publish_frequency>
    </plugin>

    <!-- Track retract: one controller per track, both on /model/<name>/tracks_cmd
         (gz.msgs.Double) so they move together. 0 = deployed (TRACK mode),
         {trk['stroke']} = retracted into the hull (HOVER mode). -->"""]
    for side in ("left", "right"):
        out.append(f"""
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>track_{side}_joint</joint_name>
      <sub_topic>tracks_cmd</sub_topic>
      <use_velocity_commands>true</use_velocity_commands>
      <p_gain>4.0</p_gain><i_gain>0</i_gain><d_gain>0</d_gain>
      <cmd_max>{RETRACT_SPEED}</cmd_max><cmd_min>-{RETRACT_SPEED}</cmd_min>
      <initial_position>0</initial_position>
    </plugin>""")
    for r in ("rudder_left", "rudder_right"):
        out.append(f"""
    <plugin filename="gz-sim-joint-position-controller-system" name="gz::sim::systems::JointPositionController">
      <joint_name>{r}_joint</joint_name><sub_topic>{r}_cmd</sub_topic>
      <p_gain>20</p_gain><d_gain>0.5</d_gain>
      <cmd_max>20</cmd_max><cmd_min>-20</cmd_min>
    </plugin>""")
    out.append(f"""
    <plugin filename="gz-sim-joint-state-publisher-system" name="gz::sim::systems::JointStatePublisher"/>

    <!-- /odom + TF ({ODOM_FRAME} -> base_link). Ground-truth pose, valid in every mode.
         Idealised: stated in docs. -->
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
                    help="frame_id of /odom and parent of base_link in TF (see docs/INTERFACES.md)")
    ap.add_argument("--no-urdf", action="store_true")
    args = ap.parse_args()

    data = json.loads((MODEL_DIR / "link_frames.json").read_text())
    global LINKS, ODOM_FRAME, HULL_COM
    ODOM_FRAME = args.odom_frame
    LINKS = data["links"]
    hull_o = LINKS["hull"]["origin_xyz"]

    # Hull CoM: place it so the whole-vehicle CoM is over the cushion centre in x
    others = [(n, e) for n, e in LINKS.items() if n != "hull"]
    m_hull = LINKS["hull"]["mass_kg"]
    m_tot = sum(e["mass_kg"] for e in LINKS.values())

    def com_world(e):
        c = prims(e["collision"])[0]["pose"][:3]
        return [e["origin_xyz"][i] + c[i] for i in range(3)]

    sx = sum(e["mass_kg"] * com_world(e)[0] for _, e in others)
    HULL_COM = [-sx / m_hull - hull_o[0], 0.0, data["hull_com_z"] - hull_o[2]]
    com_z = (m_hull * data["hull_com_z"] + sum(e["mass_kg"] * com_world(e)[2]
                                              for _, e in others)) / m_tot

    links_xml = []
    for name, e in LINKS.items():
        col = e["collision"]
        m = e["mass_kg"]
        ixx, iyy, izz = link_inertia(m, col)
        com = HULL_COM if name == "hull" else prims(col)[0]["pose"][:3]
        mesh = e["mesh"]
        ext = ("dae" if "dae" in mesh else "glb") if args.mesh == "auto" else args.mesh
        visual = ""
        if ext in mesh:
            visual = f"""
      <visual name="visual">
        <geometry><mesh><uri>model://{MODEL}/{mesh[ext]}</uri></mesh></geometry>
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
<!-- GENERATED by tidal_vehicle_description/scripts/gen_description_v2.py from link_frames.json. Do not edit by hand. -->
<!-- Vehicle Version 2. Model frame: ground plane under hull centre; vehicle standing on its deployed tracks. -->
<!-- Total mass {m_tot:.2f} kg; hull CoM offset {fmt(HULL_COM)}; vehicle CoM height {com_z:.3f} m (TRACK mode). -->
<sdf version="1.9">
  <model name="{MODEL}">
    <self_collide>false</self_collide>
{''.join(links_xml)}
{''.join(joints_xml)}
{plugins_xml(LINKS, data)}
  </model>
</sdf>
"""
    Path(args.out).write_text(sdf)
    (MODEL_DIR / "model.config").write_text(f"""<?xml version="1.0"?>
<model>
  <name>{MODEL}</name>
  <version>2.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <author><name>One Wish Coders</name></author>
  <description>Vehicle Version 2: hovercraft-dominant amphibious vehicle (2.5 x 1.5 x 1.5 m, 300 kg incl. 30 kg payload) with a retractable tracked undercarriage and controlled air-cushion load sharing; 3D LiDAR on a centre mast, camera, IMU, GNSS. Generated from Blender (assets/vehicle_blender/version_2/build_vehicle.py).</description>
</model>
""")
    if not args.no_urdf:
        write_urdf(LINKS, ext_pref=args.mesh)
    print(f"wrote {args.out}")
    print(f"links {len(LINKS)}, joints {len(joints_xml)}, total mass {m_tot:.2f} kg, "
          f"CoM height {com_z:.3f} m")
    print(f"hull CoM (hull frame) {fmt(HULL_COM)}")
    print(f"render sensors: {'off' if args.no_render_sensors else 'on'}; visual meshes: {args.mesh}")


# --------------------------------------------------------------------------
# URDF (ROS robot description) — same links, joints and origins as the SDF
# --------------------------------------------------------------------------
def write_urdf(links, ext_pref="auto"):
    out = ['<?xml version="1.0"?>',
           '<!-- GENERATED by gen_description_v2.py from link_frames.json. Do not edit by hand. -->',
           f'<robot name="{MODEL}">']
    for name, e in links.items():
        mesh = e["mesh"]
        ext = ("dae" if "dae" in mesh else "glb") if ext_pref == "auto" else ext_pref
        vis = ""
        if ext in mesh:
            vis = (f'<visual><geometry><mesh filename="package://{PKG}/models/{MODEL}/'
                   f'{mesh[ext]}"/></geometry></visual>')
        coll = ""
        for c in prims(e["collision"]):
            g = (f'<box size="{fmt(c["size"])}"/>' if c["type"] == "box" else
                 f'<cylinder radius="{c["radius"]}" length="{c["length"]}"/>')
            coll += (f'<collision><origin xyz="{fmt(c["pose"][:3])}" rpy="{fmt(c["pose"][3:])}"/>'
                     f'<geometry>{g}</geometry></collision>')
        m = e["mass_kg"]
        ixx, iyy, izz = link_inertia(m, e["collision"])
        com = HULL_COM if name == "hull" else prims(e["collision"])[0]["pose"][:3]
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
            effort = RETRACT_EFFORT if j["type"] == "prismatic" else 100
            vel = RETRACT_SPEED if j["type"] == "prismatic" else 2
            xml += f'<limit lower="{j["lower"]}" upper="{j["upper"]}" effort="{effort:g}" velocity="{vel}"/>'
        out.append(xml + "</joint>")
    out.append("</robot>")
    URDF_PATH.parent.mkdir(parents=True, exist_ok=True)
    URDF_PATH.write_text("\n".join(out) + "\n")
    print(f"wrote {URDF_PATH}")


if __name__ == "__main__":
    main()
