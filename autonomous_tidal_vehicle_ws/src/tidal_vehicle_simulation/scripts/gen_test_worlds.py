#!/usr/bin/env python3
"""
gen_test_worlds.py — writes the VEHICLE test worlds (Person 4's test fixtures,
not the tidal-corridor demo world, which the environment workstream owns).

    python3 autonomous_tidal_vehicle_ws/src/tidal_vehicle_simulation/scripts/gen_test_worlds.py
    python3 .../gen_test_worlds.py --no-render-sensors --out /tmp/worlds   # headless physics tests

Output: tidal_vehicle_simulation/worlds/vehicle_tests/<name>.sdf

Worlds (all 1 ms physics step, DART + Bullet collision so the air-cushion
ray casts work):

  empty_test.sdf        Phase 2: spawn on wheels, fold legs (hull settles on
                        its skirt), unfold legs, short ground-mode drive.
  gap_hold_test.sdf     Phase 4: hover on, legs folded, hold 70 s -> gap log.
  hover_drive_test.sdf  Phase 4: full thrust run (top speed), then a
                        differential-thrust turn (turn radius).
  transition_test.sdf   Phase 4: hover across water -> mud -> firm ground.
  mud_ab_test.sdf       Checkpoint 2: two identical vehicles, two lanes, same
                        mud patch. Lane A drives in GROUND mode (wheels),
                        lane B switches to HOVER mode and uses its fans.

Every scenario is scripted with hover::ScriptedCommands (timed gz-transport
messages), so each run is repeatable and needs no ROS in the loop.
"""
import argparse
import math
from pathlib import Path

HERE = Path(__file__).resolve().parent
OUT = HERE.parent / "worlds" / "vehicle_tests"
PLUGIN_LIB = "tidal_vehicle_plugins"
FOLD = math.pi / 2
LEGS = ("fl", "fr", "rl", "rr")

COLORS = {
    "firm": ("0.36 0.38 0.26 1", 1.0),     # firm sandy/grassy ground, mu
    "mud": ("0.20 0.13 0.07 1", 0.08),     # wet mud: very low friction
    "bed": ("0.16 0.12 0.08 1", 0.3),      # channel bed
    "pad": ("0.95 0.55 0.05 1", 1.0),      # goal pad
}


def block(name, x0, x1, y0, y1, top, kind, thick=0.3):
    rgba, mu = COLORS[kind]
    sx, sy = x1 - x0, y1 - y0
    cx, cy, cz = (x0 + x1) / 2, (y0 + y1) / 2, top - thick / 2
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {cz} 0 0 0</pose>
      <link name="link">
        <collision name="c">
          <geometry><box><size>{sx} {sy} {thick}</size></box></geometry>
          <surface><friction><ode><mu>{mu}</mu><mu2>{mu}</mu2></ode></friction></surface>
        </collision>
        <visual name="v">
          <geometry><box><size>{sx} {sy} {thick}</size></box></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse>
            <specular>{'0.3 0.3 0.3 1' if kind == 'mud' else '0.05 0.05 0.05 1'}</specular></material>
        </visual>
      </link>
    </model>"""


def stripe(name, x, y0, y1, rgba="1 1 1 1", w=0.08):
    """Visual-only line on the ground (start / finish / lane lines)."""
    return f"""
    <model name="{name}"><static>true</static><pose>{x} {(y0 + y1) / 2} 0.002 0 0 0</pose>
      <link name="link"><visual name="v"><geometry><box><size>{w} {y1 - y0} 0.004</size></box></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>"""


def water(name, x0, x1, y0, y1, level):
    rgba = "0.15 0.35 0.45 0.65"
    return f"""
    <model name="{name}"><static>true</static><pose>{(x0 + x1) / 2} {(y0 + y1) / 2} {level} 0 0 0</pose>
      <link name="link"><visual name="v"><geometry><box><size>{x1 - x0} {y1 - y0} 0.01</size></box></geometry>
        <transparency>0.35</transparency>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse><specular>0.6 0.6 0.6 1</specular></material>
      </visual></link></model>"""


def backdrop():
    """Large visual-only surround (mangrove-green) so camera shots have no void."""
    rgba = "0.13 0.19 0.10 1"
    return f"""
    <model name="backdrop"><static>true</static><pose>10 0 -0.32 0 0 0</pose>
      <link name="link"><visual name="v"><geometry><box><size>300 300 0.02</size></box></geometry>
        <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>"""


def camera(name, pose, w=1280, h=720, fov=1.3, rate=12, path="/tmp/gz_clip"):
    """Static recording camera. Gazebo renders a camera only while something
    subscribes to its image topic: tools/record_ab.sh attaches tools/gz_sink."""
    return f"""
    <model name="{name}"><static>true</static><pose>{pose}</pose>
      <link name="link"><sensor name="cam" type="camera">
        <always_on>1</always_on><update_rate>{rate}</update_rate>
        <camera><horizontal_fov>{fov}</horizontal_fov>
          <image><width>{w}</width><height>{h}</height></image>
          <clip><near>0.1</near><far>150</far></clip>
          <save enabled="true"><path>{path}/{name}</path></save>
        </camera></sensor></link></model>"""


def cylinder(name, x, y, z, r, length, roll=0.0, pitch=0.0, yaw=0.0,
             rgba="0.30 0.22 0.14 1"):
    """Static obstacle: trunks, prop roots, fallen branches."""
    return f"""
    <model name="{name}"><static>true</static><pose>{x} {y} {z} {roll} {pitch} {yaw}</pose>
      <link name="link">
        <collision name="c"><geometry><cylinder><radius>{r}</radius><length>{length}</length></cylinder></geometry></collision>
        <visual name="v"><geometry><cylinder><radius>{r}</radius><length>{length}</length></cylinder></geometry>
          <material><ambient>{rgba}</ambient><diffuse>{rgba}</diffuse></material></visual></link></model>"""


def include(name, x, y, yaw=0.0):
    return f"""
    <include>
      <uri>model://hovercraft</uri>
      <name>{name}</name>
      <pose>{x} {y} 0.002 0 0 {yaw}</pose>
    </include>"""


def cmd(t, topic, mtype, data):
    return f"""
      <command><time>{t}</time><topic>{topic}</topic><type>{mtype}</type><data>{data}</data></command>"""


def hover_on(m, t):
    return cmd(t, f"/model/{m}/hover_enabled", "gz.msgs.Boolean", "data: true")


def hover_off(m, t):
    return cmd(t, f"/model/{m}/hover_enabled", "gz.msgs.Boolean", "data: false")


def legs(m, t, angle):
    return cmd(t, f"/model/{m}/legs_cmd", "gz.msgs.Double", f"data: {angle:.4f}")


def heading(m, t, yaw):
    return cmd(t, f"/model/{m}/cmd_heading", "gz.msgs.Double", f"data: {yaw}")


def thrust(m, t, left, right):
    return (cmd(t, f"/model/{m}/thrust_left", "gz.msgs.Double", f"data: {left}") +
            cmd(t, f"/model/{m}/thrust_right", "gz.msgs.Double", f"data: {right}"))


def hover_vel(m, t, v, wz=0.0):
    """Hover-mode velocity command (what ROS /cmd_vel becomes in HOVER mode)."""
    return cmd(t, f"/model/{m}/cmd_vel_hover", "gz.msgs.Twist",
               f"linear {{ x: {v} }} angular {{ z: {wz} }}")


def drive(m, t, v, wz=0.0):
    return cmd(t, f"/model/{m}/cmd_vel", "gz.msgs.Twist",
               f"linear {{ x: {v} }} angular {{ z: {wz} }}")


def world(name, body, zones, commands, sensors, ground_height=0.0, floaters=("hovercraft",)):
    sensors_sys = """
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>""" if sensors else ""
    zone_xml = "".join(
        f"""
      <zone><name>{z[0]}</name><type>{z[1]}</type><min>{z[2]} {z[4]}</min><max>{z[3]} {z[5]}</max>"""
        + (f"<level>{z[6]}</level>" if len(z) > 6 else "") + "</zone>" for z in zones)
    buoy = """
    <plugin filename="gz-sim-buoyancy-system" name="gz::sim::systems::Buoyancy">
      <!-- water (1000 kg/m^3) below z = 0, air above. Only the hull and skirt
           boxes float (the gz graded-buoyancy calculation misbehaves on the
           small rotated cylinders of the legs and wheels). -->
      <graded_buoyancy>
        <default_density>1000</default_density>
        <density_change><above_depth>0.0</above_depth><density>1.2</density></density_change>
      </graded_buoyancy>""" + "".join(
        f"<enable>{m}::base_link</enable><enable>{m}::skirt</enable>" for m in floaters) + """
    </plugin>""" if any(z[1] == "WATER" for z in zones) else ""
    return f"""<?xml version="1.0"?>
<!-- GENERATED by hover_gazebo/scripts/gen_worlds.py -->
<sdf version="1.9">
  <world name="{name}">
    <physics name="1ms" type="dart">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <dart><collision_detector>bullet</collision_detector></dart>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>1.4466</latitude_deg><longitude_deg>103.7300</longitude_deg>
      <elevation>0</elevation><heading_deg>0</heading_deg>
    </spherical_coordinates>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>{sensors_sys}{buoy}
    <plugin filename="{PLUGIN_LIB}" name="hover::TerrainZones">
      <ground_height>{ground_height}</ground_height>{zone_xml}
    </plugin>
    <plugin filename="{PLUGIN_LIB}" name="hover::ScriptedCommands">{''.join(commands)}
    </plugin>

    <scene><ambient>0.5 0.5 0.5 1</ambient><background>0.7 0.8 0.9 1</background><grid>false</grid></scene>
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows><pose>0 0 10 0 0 0</pose>
      <diffuse>0.9 0.9 0.85 1</diffuse><specular>0.2 0.2 0.2 1</specular>
      <direction>-0.4 0.3 -0.9</direction>
    </light>
{body}
  </world>
</sdf>
"""


def build(sensors, clip_cameras=False):
    W = {}
    # ---------------- Phase 2: spawn / fold / unfold / drive ----------------
    m = "hovercraft"
    W["empty_test"] = world(
        "empty_test",
        block("ground", -20, 20, -20, 20, 0.0, "firm") + include(m, 0, 0) + backdrop(),
        [], [legs(m, 3.0, FOLD), legs(m, 7.0, 0.0),
             drive(m, 10.0, 0.6), drive(m, 14.0, 0.0, 0.6), drive(m, 17.0, 0.0)],
        sensors)
    # ---------------- Phase 2: sensor check (LiDAR + camera) --------------
    obstacles = (cylinder("trunk_1", 3.0, 1.2, 1.0, 0.15, 2.0) +
                 cylinder("trunk_2", 4.5, -2.0, 1.0, 0.18, 2.0) +
                 cylinder("trunk_3", -2.5, 2.8, 1.0, 0.12, 2.0) +
                 cylinder("root_1", 2.6, 1.2, 0.35, 0.03, 1.0, roll=0.7) +
                 cylinder("root_2", 3.4, 1.2, 0.35, 0.03, 1.0, roll=-0.7) +
                 cylinder("root_3", 3.0, 0.8, 0.35, 0.03, 1.0, pitch=0.7) +
                 cylinder("fallen_log", 2.2, -1.0, 0.12, 0.12, 2.4, pitch=1.5708, yaw=0.4) +
                 block("rock", -3.2, -2.4, -2.6, -1.8, 0.35, "bed", thick=0.35) +
                 block("bank", 8.0, 9.0, -6, 6, 0.8, "firm", thick=0.8))
    W["sensor_test"] = world(
        "sensor_test",
        block("ground", -20, 20, -20, 20, 0.0, "firm") + include(m, 0, 0) + obstacles + backdrop(),
        [], [], sensors)
    # ---------------- Skirt vs debris: low branch passes, tall log blocks ---
    lo, tall = "hc_low", "hc_tall"
    body = (block("ground", -6, 20, -5, 5, 0.0, "firm") +
            cylinder("low_branch", 5.0, 1.5, 0.035, 0.035, 2.0, roll=1.5708) +     # 7 cm high
            cylinder("tall_log", 5.0, -1.5, 0.10, 0.10, 2.0, roll=1.5708) +        # 20 cm high
            include(lo, 0, 1.5) + include(tall, 0, -1.5) + backdrop())
    cmds = []
    for v in (lo, tall):
        cmds += [hover_on(v, 0.5), legs(v, 2.0, FOLD), hover_vel(v, 4.0, 1.0),
                 hover_vel(v, 11.9, 0.0)]
    # re-send the speed command so the 0.5 s command timeout doesn't stop it
    for v in (lo, tall):
        cmds += [hover_vel(v, 4.0 + 0.3 * k, 1.0) for k in range(1, 26)]
    W["debris_test"] = world("debris_test", body, [], cmds, sensors, floaters=(lo, tall))
    # ---------------- cmd_vel tracking in HOVER mode -------------------------
    seq = [(4.0, 1.5, 0.0), (12.0, 1.0, 0.5), (20.0, 0.0, 0.0)]
    cmds = [hover_on(m, 0.5), legs(m, 2.0, FOLD)]
    t = 4.0
    while t < 26.0:                                   # 10 Hz, like a ROS controller
        v, wz = [(vv, ww) for ts, vv, ww in seq if ts <= t][-1]
        cmds.append(hover_vel(m, round(t, 2), v, wz))
        t += 0.1
    W["cmd_vel_test"] = world(
        "cmd_vel_test", block("ground", -10, 60, -30, 30, 0.0, "firm") + include(m, 0, 0) + backdrop(),
        [], cmds, sensors)
    # ---------------- Phase 4: 60 s gap hold --------------------------------
    W["gap_hold_test"] = world(
        "gap_hold_test",
        block("ground", -20, 20, -20, 20, 0.0, "firm") + include(m, 0, 0),
        [], [hover_on(m, 0.5), legs(m, 2.0, FOLD)], sensors)
    # ---------------- Phase 4: top speed + turn radius ----------------------
    W["hover_drive_test"] = world(
        "hover_drive_test",
        block("ground", -10, 80, -40, 40, 0.0, "firm") + include(m, 0, 0),
        [], [hover_on(m, 0.5), legs(m, 2.0, FOLD), heading(m, 2.0, 0.0),
             thrust(m, 5.0, 30, 30),          # straight, full thrust, heading hold
             heading(m, 20.0, "nan"),         # release heading hold ...
             thrust(m, 20.0, 30, 3),          # ... hard right turn (left fan high)
             thrust(m, 40.0, 0, 0)], sensors)
    # ---------------- Phase 4: water -> mud -> firm -------------------------
    body = (block("launch", -6, 4, -6, 6, 0.0, "firm") +
            block("channel_bed", 4, 12, -6, 6, -0.45, "bed") +
            water("channel_water", 4, 12, -6, 6, 0.0) +
            block("mudflat", 12, 22, -6, 6, 0.0, "mud") +
            block("bank", 22, 36, -6, 6, 0.0, "firm") +
            block("goal_pad", 30, 32, -1, 1, 0.004, "pad", thick=0.01) +
            include(m, 0, 0) + backdrop())
    W["transition_test"] = world(
        "transition_test", body,
        [("channel", "WATER", 4, 12, -6, 6, 0.0), ("mudflat", "MUD", 12, 22, -6, 6)],
        [hover_on(m, 0.5), legs(m, 2.0, FOLD), heading(m, 2.0, 0.0), thrust(m, 5.0, 18, 18),
         thrust(m, 18.0, -8, -8), thrust(m, 19.5, 0, 0)], sensors)
    # ---------------- Checkpoint 2: A/B mud test ----------------------------
    ga, hb = "hc_ground", "hc_hover"
    body = (block("firm_start", -6, 4, -5, 5, 0.0, "firm") +
            block("mud_patch", 4, 12, -5, 5, 0.0, "mud") +
            block("firm_finish", 12, 26, -5, 5, 0.0, "firm") +
            stripe("start_line", 0.8, -4, 4) + stripe("finish_line", 16, -4, 4, "0.1 0.8 0.2 1") +
            stripe("lane_divider", 11, -0.04, 0.04, "1 1 0.2 1", w=30) +
            include(ga, 0, 1.6) + include(hb, 0, -1.6) + backdrop() +
            (camera("cam_wide", "9.0 -11.8 8.2 0 0.66 1.5708", fov=1.45) +
             camera("cam_close", "6.9 4.9 1.5 0 0.28 -2.13", w=640, h=360, fov=1.1)
             if clip_cameras else ""))
    W["mud_ab_test"] = world(
        "mud_ab_test", body, [("mud_patch", "MUD", 4, 12, -5, 5)],
        [# Lane A: GROUND mode, wheels drive at 1 m/s the whole run
         drive(ga, 2.0, 1.0),
         # Lane B: switch to HOVER mode, then fans
         hover_on(hb, 1.0), legs(hb, 3.0, FOLD), heading(hb, 3.0, 0.0), thrust(hb, 4.5, 20, 20),
         thrust(hb, 12.5, 0, 0), thrust(hb, 13.0, -10, -10), thrust(hb, 14.3, 0, 0)],
        sensors)
    return W


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render-sensors", action="store_true")
    ap.add_argument("--clip-cameras", action="store_true",
                    help="add the two recording cameras to mud_ab_test (used by tools/record_ab.sh)")
    ap.add_argument("--render-engine", default="ogre2",
                    help="ogre2 (default, GPU) or ogre (OGRE 1.9, works on software GL)")
    ap.add_argument("--out", default=str(OUT))
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    for name, xml in build(not a.no_render_sensors, a.clip_cameras).items():
        if a.clip_cameras and "gz-sim-sensors-system" not in xml:
            xml = xml.replace('<plugin filename="gz-sim-imu-system"',
                              '<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">'
                              '<render_engine>ogre2</render_engine></plugin>\n    <plugin filename="gz-sim-imu-system"', 1)
        xml = xml.replace("<render_engine>ogre2</render_engine>",
                          f"<render_engine>{a.render_engine}</render_engine>")
        (out / f"{name}.sdf").write_text(xml)
        print("wrote", out / f"{name}.sdf")


if __name__ == "__main__":
    main()
