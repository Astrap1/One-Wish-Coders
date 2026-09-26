// AirCushion — simplified air-cushion vehicle model for Gazebo Harmonic.
//
// This is a *simplified* hover model for a proof of concept, not CFD.
// The lift fan pressurises the cushion under the skirt. Near the design
// gap, the lift a real cushion gives behaves like a preloaded spring:
// less gap -> less air leakage -> more pressure -> more lift. So each hull
// corner gets
//
//     F_i = r * F0 + k * (h_target - h_i) - c * dh_i/dt,
//     clamped to [0, r * Fmax]
//
// where F0 = m*g/4 (preload: the cushion carries the weight at h_target),
// h_i is the gap under corner i and r in [0,1] is the lift-fan spin-up
// ramp. Above the design gap the cushion vents gradually:
//     F_i = r * F0 * exp(-(h_i - h_target) / lambda) - c * dh_i/dt
// so with the fan running and the wheels still down (10 cm clearance) the
// cushion already carries ~30 % of the weight. That unloads the wheels, so
// the legs fold without dragging the wheels across the ground.
//
// While the cushion carries the vehicle, ground friction is gone (the
// skirt isn't touching), so we add low "glide" drag instead:
// linear + quadratic in speed, plus yaw damping.
//
// The two rear ducted fans push along hull +X at their mount points.
// Differential thrust gives yaw. The rudder vanes sit in the fans'
// slipstream: each one makes a side force ~ 0.5 * A_rudder * CL_alpha /
// A_duct * T * delta (independent of forward speed), so the rudders steer
// even at low speed.
//
// Turning aids (Version 2; off unless the SDF enables them, so Version 1 is
// unchanged). In hover velocity mode the yaw-rate controller asks for a yaw
// moment M, which is shared out in this order:
//   1. rudders (<rudder_control>true</rudder_control>): the plugin sets both
//      rudder angles on rudder_left_cmd / rudder_right_cmd so their side force
//      gives M. Free while the fans push forward; useless at zero thrust, so
//      they fade in above <rudder_min_thrust> (N, both fans) and move no
//      faster than <rudder_rate> (rad/s, a servo slew limit): at low thrust
//      they would otherwise swing stop to stop chasing small yaw errors.
//   2. puff ports (<puff_port_force> > 0): four side vents, one on each side
//      at the bow and the stern, bleed cushion air sideways. Opening a bow
//      vent on one side and a stern vent on the other gives a pure yaw couple
//      that works at any speed, including a pivot in place. Their force scales
//      with cushion pressure (lift-fan ramp x cushion support). Spare vent
//      force damps sideways drift (<puff_lateral_gain>, N per m/s of slip).
//      The lift fan is assumed to have the flow margin to feed one pair.
//   3. differential fan thrust: whatever the rudders and vents can't supply,
//      within the same collective-first limits as before.
// The moments actually delivered (measured rudder angles, lagged vent force)
// are subtracted, so the three never add up to more than M. The aids are
// off while load sharing (the tracks steer) and can be switched off at run
// time on turn_aids for A/B tests.
//
// Low-level heading hold (the autopilot's inner loop, optional): once a
// target heading arrives on cmd_heading, the collective thrust is split
// left/right by a PD law on heading error. Phase 5's waypoint navigator
// sends headings; the scripted Phase 4 tests use it to hold a straight line.
//
// Mud resistance: in a MUD zone and NOT carried by the cushion (wheels or
// skirt in the mud) the hull gets a sinkage / rolling-resistance force
//     F = -(c_mud * v + F_rr * tanh(v / 0.05))
// which is larger than the traction wheels get on slippery mud. So a
// wheeled vehicle bogs down and a hovering one glides over.
//
// Gap sensing: physics ray casts straight down (hull -Z) from the four
// corners of the skirt bottom, the same places as the 4 downward range
// sensors in the SDF. Ray casting needs DART's Bullet collision detector
// (<physics><dart><collision_detector>bullet</collision_detector>).
// Otherwise it falls back to a flat ground height. Water surfaces come from
// the TerrainZones world system, because water has no collision geometry.
//
// High-speed drag (Version 3; off unless set): glide_constant_drag (N, skirt
// and spray drag), water_hump_drag / water_hump_speed (the over-water wave
// hump) and thrust_falloff_speed (ducted-fan thrust falls linearly to zero
// there). Version 2 keeps its deliberately high demo glide drag instead.
//
// Load sharing (Version 2, TRACK mode): a value s in (0, 1] on lift_share
// makes the cushion carry that fraction of the weight while the tracks carry
// the rest. Each corner then gets a constant r * s * F0 (no spring term, so
// the cushion never lifts the vehicle off its tracks), still venting above
// the design gap. While load sharing is on, the propulsion fans idle: the
// tracks propel and brake, and a fan speed loop would only fight them. A
// negative or NaN value switches load sharing off and restores the normal
// hover law and fan propulsion. Vehicles that never publish lift_share
// (Version 1) behave exactly as before.
//
// Topics (all under /model/<model_name>/):
//   subscribes  hover_enabled (gz.msgs.Boolean), thrust_left, thrust_right (gz.msgs.Double, N),
//               lift_share (gz.msgs.Double, 0..1; < 0 or NaN = off),
//               cmd_heading (gz.msgs.Double, rad; NaN = heading hold off),
//               turn_aids (gz.msgs.Boolean; rudders + puff ports on/off, default on),
//               cmd_vel_hover (gz.msgs.Twist: linear.x m/s, angular.z rad/s) —
//               hover-mode velocity control. The fan thrusts are then computed
//               here (feed-forward drag + PI on speed, P on yaw rate), which is
//               how ROS /cmd_vel drives the vehicle in HOVER mode. A command
//               older than <cmd_timeout> is treated as "stop".
//   publishes   cushion_gap (gz.msgs.Double, mean gap m), corner_gaps (gz.msgs.Double_V),
//               hover_state (gz.msgs.StringMsg: OFF | SPIN_UP | FAN_ON_NO_SUPPORT |
//               HOVER | LOAD_SHARE | SPIN_DOWN), terrain (gz.msgs.StringMsg),
//               rudder_left_cmd, rudder_right_cmd (gz.msgs.Double, rad; only
//               with rudder_control, read by the rudder JointPositionControllers),
//               puff_{bow,stern}_{left,right}_cmd (gz.msgs.Double, m of shutter
//               opening; only with puff_shutter_travel > 0, purely visual)

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/msgs/twist.pb.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/transport/Node.hh>

#include "hover_plugins/TerrainRegistry.hh"

namespace hover
{
using namespace gz;
using namespace gz::sim;

class AirCushion : public System,
                   public ISystemConfigure,
                   public ISystemPreUpdate
{
  public: void Configure(const Entity &_entity,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         EntityComponentManager &_ecm,
                         EventManager &) override
  {
    this->model = Model(_entity);
    if (!this->model.Valid(_ecm))
    {
      gzerr << "[AirCushion] must be attached to a model." << std::endl;
      return;
    }
    this->name = this->model.Name(_ecm);

    auto get = [&](const std::string &_k, double _d)
    { return _sdf->Get<double>(_k, _d).first; };

    const std::string linkName = _sdf->Get<std::string>("link_name", "hull").first;
    this->hullEntity = this->model.LinkByName(_ecm, linkName);
    if (this->hullEntity == kNullEntity)
    {
      gzerr << "[AirCushion] link [" << linkName << "] not found." << std::endl;
      return;
    }
    this->hull = Link(this->hullEntity);
    this->hull.EnableVelocityChecks(_ecm, true);

    // Skirt-bottom corner points in the hull link frame
    if (_sdf->HasElement("corner"))
    {
      for (auto e = _sdf->FindElement("corner"); e; e = e->GetNextElement("corner"))
        this->corners.push_back(e->Get<math::Vector3d>());
    }
    if (this->corners.size() != 4)
    {
      gzerr << "[AirCushion] need exactly 4 <corner> elements, got "
            << this->corners.size() << std::endl;
      return;
    }

    this->hTarget   = get("target_gap", 0.04);
    this->k         = get("stiffness", 1500.0);
    this->c         = get("damping", 130.0);
    this->fMaxFactor = get("max_force_factor", 2.0);
    this->spinup    = get("spinup_time", 1.5);
    this->b1        = get("glide_linear_drag", 5.0);
    this->b2        = get("glide_quadratic_drag", 6.0);
    this->bYaw      = get("glide_yaw_drag", 4.0);
    this->maxThrust = get("max_thrust", 30.0);
    this->thrustTau = get("thrust_time_constant", 0.3);
    this->reverseFrac = get("reverse_thrust_fraction", 0.5);
    this->yawReserve = get("yaw_reserve_fraction", 0.0);
    this->cMud      = get("mud_viscous_drag", 150.0);
    this->mudRR     = get("mud_rolling_resistance", 0.25);
    this->cMudYaw   = get("mud_yaw_drag", 20.0);
    this->rayRange  = get("ray_range", 1.0);
    this->lambda    = get("vent_length", 0.05);
    this->latFactor = get("glide_lateral_factor", 3.0);
    this->rudderK   = get("rudder_wash_coeff", 0.55);
    this->rudderArm = _sdf->Get<math::Vector3d>("rudder_point",
                        math::Vector3d(-0.62, 0, 0.2)).first;
    this->rudderControl = _sdf->Get<bool>("rudder_control", false).first;
    this->rudderMax = get("rudder_max_angle", 0.44);
    this->rudderMinThrust = get("rudder_min_thrust", 0.0);
    this->rudderRate = get("rudder_rate", 0.0);
    this->puffForce = get("puff_port_force", 0.0);
    this->puffPt    = _sdf->Get<math::Vector3d>("puff_port_point",
                        math::Vector3d(0.8, 0.68, 0.0)).first;
    this->puffTau   = get("puff_port_time_constant", 0.15);
    this->puffLatGain = get("puff_lateral_gain", 0.0);
    this->shutterTravel = get("puff_shutter_travel", 0.0);
    this->hdgKp     = get("heading_kp", 40.0);
    this->hdgKd     = get("heading_kd", 25.0);
    this->spdKp     = get("speed_kp", 40.0);
    this->spdKi     = get("speed_ki", 10.0);
    this->yawKp     = get("yaw_rate_kp", 30.0);
    this->cmdTimeout = get("cmd_timeout", 0.5);
    this->waterGlideFactor = get("water_glide_drag_factor", 1.3);
    this->cConst    = get("glide_constant_drag", 0.0);
    this->humpDrag  = get("water_hump_drag", 0.0);
    this->humpSpeed = get("water_hump_speed", 3.0);
    this->falloffSpeed = get("thrust_falloff_speed", 0.0);
    this->cWater    = get("water_hull_drag", 60.0);
    this->cWaterYaw = get("water_yaw_drag", 10.0);
    this->useRaycast = _sdf->Get<bool>("use_raycast", true).first;
    this->hoverEnabled = _sdf->Get<bool>("start_enabled", false).first;
    this->ramp = this->hoverEnabled ? 1.0 : 0.0;
    this->thrustPt[0] = _sdf->Get<math::Vector3d>("thrust_point_left",
                          math::Vector3d(-0.475, 0.175, 0.2)).first;
    this->thrustPt[1] = _sdf->Get<math::Vector3d>("thrust_point_right",
                          math::Vector3d(-0.475, -0.175, 0.2)).first;
    this->pubPeriod = 1.0 / get("publish_rate", 20.0);

    // Total model mass -> cushion preload
    double mass = 0.0;
    for (auto l : this->model.Links(_ecm))
    {
      auto in = _ecm.Component<components::Inertial>(l);
      if (in) mass += in->Data().MassMatrix().Mass();
    }
    this->weight = mass * 9.81;
    this->f0 = this->weight / 4.0;

    // Downward rays (hull -Z) just below each skirt corner
    if (this->useRaycast)
    {
      components::RaycastDataInfo data;
      for (const auto &cp : this->corners)
      {
        components::RayInfo ray;
        ray.start = cp - math::Vector3d(0, 0, kRayStartOffset);
        ray.end = cp - math::Vector3d(0, 0, kRayStartOffset + this->rayRange);
        data.rays.push_back(ray);
      }
      _ecm.CreateComponent(this->hullEntity, components::RaycastData(data));
    }

    // Transport
    const std::string ns = "/model/" + this->name + "/";
    this->node.Subscribe(ns + "hover_enabled", &AirCushion::OnEnable, this);
    this->node.Subscribe(ns + "lift_share", &AirCushion::OnLiftShare, this);
    this->node.Subscribe(ns + "thrust_left", &AirCushion::OnThrustLeft, this);
    this->node.Subscribe(ns + "thrust_right", &AirCushion::OnThrustRight, this);
    this->node.Subscribe(ns + "cmd_heading", &AirCushion::OnHeading, this);
    this->node.Subscribe(ns + "cmd_vel_hover", &AirCushion::OnCmdVel, this);
    this->node.Subscribe(ns + "turn_aids", &AirCushion::OnTurnAids, this);
    if (this->rudderControl)
    {
      this->pubRudder[0] = this->node.Advertise<msgs::Double>(ns + "rudder_left_cmd");
      this->pubRudder[1] = this->node.Advertise<msgs::Double>(ns + "rudder_right_cmd");
    }
    if (this->shutterTravel > 0.0)
    {
      const char *names[4] = {"puff_bow_left_cmd", "puff_bow_right_cmd",
                              "puff_stern_left_cmd", "puff_stern_right_cmd"};
      for (int i = 0; i < 4; ++i)
        this->pubShutter[i] = this->node.Advertise<msgs::Double>(ns + names[i]);
    }
    this->pubGap = this->node.Advertise<msgs::Double>(ns + "cushion_gap");
    this->pubCorners = this->node.Advertise<msgs::Double_V>(ns + "corner_gaps");
    this->pubState = this->node.Advertise<msgs::StringMsg>(ns + "hover_state");
    this->pubTerrain = this->node.Advertise<msgs::StringMsg>(ns + "terrain");

    // Rudder joints (their angle sets the slipstream side force)
    for (const char *rn : {"rudder_left_joint", "rudder_right_joint"})
    {
      const Entity je = this->model.JointByName(_ecm, rn);
      if (je == kNullEntity) continue;
      if (!_ecm.Component<components::JointPosition>(je))
        _ecm.CreateComponent(je, components::JointPosition());
      this->rudderJoints.push_back(je);
    }

    // Joints whose positions are added to the CSV log (for replay / plots)
    if (_sdf->HasElement("log_joint"))
    {
      for (auto e = _sdf->FindElement("log_joint"); e; e = e->GetNextElement("log_joint"))
      {
        const std::string jn = e->Get<std::string>();
        const Entity je = this->model.JointByName(_ecm, jn);
        if (je == kNullEntity) { gzwarn << "[AirCushion] no joint " << jn << std::endl; continue; }
        if (!_ecm.Component<components::JointPosition>(je))
          _ecm.CreateComponent(je, components::JointPosition());
        this->logJoints.emplace_back(jn, je);
      }
    }

    const std::string logPath = _sdf->Get<std::string>("log_file", "").first;
    if (!logPath.empty())
    {
      std::string p = logPath;
      const auto pos = p.find("{model}");
      if (pos != std::string::npos) p.replace(pos, 7, this->name);
      this->log.open(p);
      this->log << "t,x,y,z,roll,pitch,yaw,vx,vy,speed,gap_fl,gap_fr,gap_rl,gap_rr,"
                   "gap_mean,cushion_force,support,hover_state,terrain,"
                   "thrust_left,thrust_right,rudder_cmd,puff_bow,puff_stern";
      for (const auto &j : this->logJoints) this->log << ',' << j.first;
      this->log << '\n';
    }

    this->configured = true;
    gzmsg << "[AirCushion] " << this->name << ": mass " << mass << " kg, preload "
          << this->f0 << " N/corner, k " << this->k << ", c " << this->c
          << ", target gap " << this->hTarget << " m, raycast "
          << (this->useRaycast ? "on" : "off") << std::endl;
  }

  public: void PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm) override
  {
    if (!this->configured || _info.paused) return;
    const double dt = std::chrono::duration<double>(_info.dt).count();
    const double t = std::chrono::duration<double>(_info.simTime).count();
    if (dt <= 0) return;

    const auto poseOpt = this->hull.WorldPose(_ecm);
    const auto vOpt = this->hull.WorldLinearVelocity(_ecm);
    const auto wOpt = this->hull.WorldAngularVelocity(_ecm);
    if (!poseOpt || !vOpt || !wOpt) return;
    const math::Pose3d pose = *poseOpt;
    const math::Vector3d v = *vOpt, w = *wOpt;

    bool enabled, velMode, aidsOn;
    double cmdL, cmdR, hdg, vRef, rRef, cmdAge, share;
    {
      std::lock_guard<std::mutex> lock(this->mutex);
      enabled = this->hoverEnabled;
      aidsOn = this->turnAids;
      share = this->liftShare;
      cmdL = this->thrustCmd[0];
      cmdR = this->thrustCmd[1];
      hdg = this->headingCmd;
      velMode = this->velMode;
      vRef = this->velCmd[0];
      rRef = this->velCmd[1];
      if (this->velCmdStamp < 0) this->velCmdStamp = t;      // first msg: stamp in sim time
      cmdAge = t - this->velCmdStamp;
    }
    // Turning aids apply only in hover velocity mode, never while the tracks
    // carry the vehicle. Targets fall back to zero otherwise.
    const bool aids = velMode && aidsOn && !(std::isfinite(share) && share >= 0.0);
    double rudderTarget = 0.0;
    std::array<double, 2> puffTarget{{0.0, 0.0}};      // bow, stern side force (N, +Y)

    if (velMode)
    {
      if (cmdAge > this->cmdTimeout) { vRef = 0.0; rRef = 0.0; }   // stale -> stop
      const math::Vector3d fwdW = pose.Rot().RotateVector(math::Vector3d::UnitX);
      const double vFwd = v.Dot(fwdW);
      const double e = vRef - vFwd;
      // feed-forward: thrust that balances glide drag at the target speed
      double ff = this->b1 * vRef + this->b2 * vRef * std::abs(vRef);
      if (std::abs(vRef) > 1e-6)
      {
        // physical terms (Version 3): constant skirt drag, the hump over water,
        // and the extra command needed because thrust falls off with speed
        const double mag = this->cConst * std::tanh(std::abs(vRef) / 0.3) +
                           (this->overWater ? this->HumpDrag(std::abs(vRef)) : 0.0);
        ff += std::copysign(mag, vRef);
        if (vRef > 0.0)
          ff /= std::max(0.2, this->FalloffFactor(vRef));
      }
      double coll = ff + this->spdKp * e + this->spdKi * this->spdInt;
      const double collMax = 2.0 * this->maxThrust;
      const double collMin = -2.0 * this->reverseFrac * this->maxThrust;
      if (coll < collMax && coll > collMin) this->spdInt += e * dt;  // anti-windup
      this->spdInt = std::clamp(this->spdInt, -3.0, 3.0);
      if (vRef == 0.0 && std::abs(vFwd) < 0.05) this->spdInt = 0.0;
      coll = std::clamp(coll, collMin, collMax);
      // yaw: differential thrust (R - L) acting on the 2 x 0.175 m fan spacing
      const double arm = std::max(std::abs(this->thrustPt[0].Y()), 0.05);
      // Yaw moment wanted (N m, + = nose left): feed-forward on glide yaw drag
      // plus P on yaw rate. Same gains as the fans-only law it replaces.
      const double mWant = this->bYaw * rRef + arm * this->yawKp * (rRef - w.Z());
      double mLeft = mWant;
      if (aids && this->rudderControl && this->rudderJoints.size() == 2)
      {
        // 1. Rudders. Side force k*T*sin(delta) at the stern gives a yaw moment
        // x_rudder * k * T * sin(delta) (x_rudder < 0, so +delta turns right).
        // With little forward thrust there is little slipstream: the rudders
        // could only reach their stops chasing small errors (chatter), so they
        // fade in between rudder_min_thrust and twice that, and centre below.
        const double tPos = std::max(this->thrust[0], 0.0) + std::max(this->thrust[1], 0.0);
        const double perSin = this->rudderArm.X() * this->rudderK * tPos;
        const double sMax = std::sin(this->rudderMax);
        const double fade = this->rudderMinThrust > 0.0
            ? std::clamp((tPos - this->rudderMinThrust) / this->rudderMinThrust, 0.0, 1.0)
            : 1.0;
        if (std::abs(perSin) > 1e-3 && fade > 0.0)
          rudderTarget = fade * std::asin(std::clamp(mWant / perSin, -sMax, sMax));
        mLeft -= this->RudderMoment(_ecm);           // what the rudders give now
      }
      if (aids && this->puffForce > 0.0)
      {
        // 2. Puff ports: a bow/stern pair on opposite sides is a pure couple.
        const double fMax = this->puffForce * this->ramp *
                            std::clamp(this->lastSupport, 0.0, 1.0);
        const double xp = std::abs(this->puffPt.X());
        if (fMax > 1e-6 && xp > 1e-3)
        {
          const double u = std::clamp(mLeft / (2.0 * fMax * xp), -1.0, 1.0);
          // Spare vent force (after the yaw couple) damps sideways drift.
          const math::Vector3d latW = pose.Rot().RotateVector(math::Vector3d::UnitY);
          const double spare = fMax * (1.0 - std::abs(u));
          const double fLat = std::clamp(-0.5 * this->puffLatGain * v.Dot(latW), -spare, spare);
          puffTarget = {{u * fMax + fLat, -u * fMax + fLat}};
        }
        mLeft -= xp * (this->puffF[0] - this->puffF[1]);   // what the vents give now
      }
      // 3. Differential thrust supplies the rest.
      double diff = mLeft / arm;
      // Collective (speed / braking) has priority: limit the differential so
      // both fans stay inside [-reverse_thrust_fraction, 1] x max thrust.
      // Otherwise a hard turn clips the reversing fan and the pair pushes
      // forward while pivoting. yaw_reserve_fraction keeps some steering even
      // while braking at full reverse (e.g. holding station on a slope).
      const double half = 0.5 * coll;
      const double rev = this->reverseFrac * this->maxThrust;
      const double dMax = std::max(2.0 * std::max(0.0, std::min(this->maxThrust - half, half + rev)),
                                   2.0 * this->yawReserve * this->maxThrust);
      diff = std::clamp(diff, -dMax, dMax);
      cmdL = half - 0.5 * diff;
      cmdR = half + 0.5 * diff;
    }
    else if (std::isfinite(hdg))
    {
      // PD on heading: split the collective thrust into left/right
      const double yaw = pose.Rot().Yaw();
      const double err = std::atan2(std::sin(hdg - yaw), std::cos(hdg - yaw));
      const double diff = this->hdgKp * err - this->hdgKd * w.Z();   // + = turn left
      const double coll = 0.5 * (cmdL + cmdR);
      cmdL = coll - 0.5 * diff;      // left fan weaker -> nose left (CCW)
      cmdR = coll + 0.5 * diff;
    }

    // On the tracks (load sharing): the tracks propel and brake, fans idle.
    if (std::isfinite(share) && share >= 0.0)
    {
      cmdL = 0.0;
      cmdR = 0.0;
      this->spdInt = 0.0;
    }

    // Lift-fan spin-up / spin-down ramp
    const double dr = dt / std::max(this->spinup, 1e-3);
    this->ramp = std::clamp(this->ramp + (enabled ? dr : -dr), 0.0, 1.0);

    // Thrust fans respond with a first-order lag
    const double a = std::min(1.0, dt / std::max(this->thrustTau, 1e-3));
    const double tMin = -this->reverseFrac * this->maxThrust;
    this->thrust[0] += (std::clamp(cmdL, tMin, maxThrust) - this->thrust[0]) * a;
    this->thrust[1] += (std::clamp(cmdR, tMin, maxThrust) - this->thrust[1]) * a;

    // Puff-port vent valves respond with a first-order lag
    const double ap = std::min(1.0, dt / std::max(this->puffTau, 1e-3));
    for (int j = 0; j < 2; ++j)
      this->puffF[j] += (puffTarget[j] - this->puffF[j]) * ap;

    // Rudder servo: slew-rate limited towards the target
    if (this->rudderRate > 0.0)
    {
      const double step = this->rudderRate * dt;
      this->rudderCmd += std::clamp(rudderTarget - this->rudderCmd, -step, step);
    }
    else
    {
      this->rudderCmd = rudderTarget;
    }

    // Rudder angles and puff-port shutters (50 Hz is plenty for the joint
    // controllers). A +Y force comes from the right-hand vent, so each end
    // opens the shutter on the side opposite the push, in proportion to the
    // vent force.
    if (t - this->lastRudderPub >= 0.02 - 1e-9)
    {
      this->lastRudderPub = t;
      if (this->rudderControl)
      {
        msgs::Double rm; rm.set_data(this->rudderCmd);
        this->pubRudder[0].Publish(rm);
        this->pubRudder[1].Publish(rm);
      }
      if (this->shutterTravel > 0.0 && this->puffForce > 0.0)
      {
        for (int j = 0; j < 2; ++j)                  // 0 = bow, 1 = stern
        {
          double open = std::min(1.0, std::abs(this->puffF[j]) / this->puffForce);
          if (open < 0.05) open = 0.0;               // don't twitch on tiny forces
          msgs::Double left, right;
          left.set_data(this->puffF[j] < 0 ? open * this->shutterTravel : 0.0);
          right.set_data(this->puffF[j] > 0 ? open * this->shutterTravel : 0.0);
          this->pubShutter[2 * j].Publish(left);
          this->pubShutter[2 * j + 1].Publish(right);
        }
      }
    }

    // --- Gaps under each corner --------------------------------------------
    auto &reg = TerrainRegistry::Instance();
    const components::RaycastDataInfo *rays = nullptr;
    if (this->useRaycast)
    {
      auto comp = _ecm.Component<components::RaycastData>(this->hullEntity);
      if (comp) rays = &comp->Data();
    }

    const bool sharing = std::isfinite(share) && share >= 0.0;
    share = sharing ? std::min(share, 1.0) : 1.0;

    math::Vector3d fTot, tTot;
    double fCushion = 0.0;
    std::array<double, 4> gap{};
    double groundGapSum = 0.0, waterGapSum = 0.0;
    for (size_t i = 0; i < 4; ++i)
    {
      const math::Vector3d rW = pose.Rot().RotateVector(this->corners[i]);
      const math::Vector3d pW = pose.Pos() + rW;

      double gGround = pW.Z() - reg.GroundHeight();       // flat fallback
      if (rays && rays->results.size() == 4)
      {
        const double f = rays->results[i].fraction;
        if (std::isfinite(f))
        {
          this->rayWorks = true;
          gGround = kRayStartOffset + f * this->rayRange;
        }
        else if (this->rayWorks)
        {
          // Miss. Either nothing within range below, or the corner is already
          // pressed into the ground (the ray then starts inside the terrain
          // collision and cannot hit it). Tell the two apart by the last gap.
          gGround = this->prevGap[i] < 0.03 ? 0.0 : kRayStartOffset + this->rayRange;
        }
      }
      const double gWater = pW.Z() - reg.WaterLevel(pW.X(), pW.Y());
      gap[i] = std::min(gGround, gWater);
      groundGapSum += gGround;
      waterGapSum += gWater;
      this->prevGap[i] = gap[i];

      const auto vpOpt = this->hull.WorldLinearVelocity(_ecm, this->corners[i]);
      const double hdot = vpOpt ? vpOpt->Z() : v.Z();

      double F = 0.0;
      if (this->ramp > 0.0)
      {
        const double dh = gap[i] - this->hTarget;
        if (sharing)
        {
          // Load share: constant fraction of the corner load, venting above
          // the design gap; the tracks carry the rest.
          const double f = this->ramp * share * this->f0;
          F = (dh <= 0 ? f : f * std::exp(-dh / this->lambda)) - this->c * hdot;
        }
        else
        {
          F = (dh <= 0 ? this->ramp * this->f0 - this->k * dh
                       : this->ramp * this->f0 * std::exp(-dh / this->lambda))
              - this->c * hdot;
        }
        F = std::clamp(F, 0.0, this->ramp * this->fMaxFactor * this->f0);
      }
      fCushion += F;
      const math::Vector3d fW = pose.Rot().RotateVector(math::Vector3d(0, 0, F));
      fTot += fW;
      tTot += rW.Cross(fW);
    }
    const double support = fCushion / std::max(this->weight, 1e-6);
    this->lastSupport = support;
    const bool onCushion = support > 0.5;
    const double gapMean = (gap[0] + gap[1] + gap[2] + gap[3]) / 4.0;

    // --- Glide drag while riding on the cushion ------------------------------
    const math::Vector3d vxy(v.X(), v.Y(), 0);
    const double speed = vxy.Length();
    // Over water only where the water surface is above the ground measured
    // under the skirt (a large tide zone floods sloping terrain gradually).
    Terrain terrain = reg.Classify(pose.Pos().X(), pose.Pos().Y());
    if (terrain == Terrain::WATER && waterGapSum >= groundGapSum)
      terrain = reg.ClassifyLand(pose.Pos().X(), pose.Pos().Y());
    if (support > 0.05)
    {
      // Over water the cushion also pushes a spray/wave "hump" drag
      const double s = std::min(1.0, support) *
          (terrain == Terrain::WATER ? this->waterGlideFactor : 1.0);
      // Anisotropic: sideways sliding is resisted more (skirt side walls,
      // fingers and rudder/duct "keel" area), so turns don't turn into drifts.
      const math::Vector3d fwdXY = pose.Rot().RotateVector(math::Vector3d::UnitX);
      math::Vector3d f2(fwdXY.X(), fwdXY.Y(), 0);
      f2.Normalize();
      const math::Vector3d lat2(-f2.Y(), f2.X(), 0);
      const double vf = vxy.Dot(f2), vl = vxy.Dot(lat2);
      fTot += -(this->b1 + this->b2 * speed) * s * (vf * f2 + this->latFactor * vl * lat2);
      tTot.Z() += -this->bYaw * s * w.Z();
      // Physical high-speed terms (Version 3; all zero by default):
      // skirt / spray drag that barely depends on speed, and the wave "hump"
      // over water, which peaks at water_hump_speed (Froude ~0.56) and decays
      // above it as (v_hump / v)^2.
      if (speed > 1e-6 && (this->cConst > 0.0 || this->humpDrag > 0.0))
      {
        const math::Vector3d dir = (vf * f2 + this->latFactor * vl * lat2) / speed;
        double f = this->cConst * std::tanh(speed / 0.3);
        if (terrain == Terrain::WATER)
          f += this->HumpDrag(speed);
        fTot += -std::min(1.0, support) * f * dir;
      }
    }
    this->overWater = (terrain == Terrain::WATER);

    // --- Rear fan thrust ----------------------------------------------------
    // Ducted-fan thrust falls off with forward speed (the jet has less to add
    // to the incoming air); thrust_falloff_speed ~ static jet speed, 0 = off.
    const math::Vector3d fwd = pose.Rot().RotateVector(math::Vector3d::UnitX);
    const double falloff = this->FalloffFactor(v.Dot(fwd));
    for (int j = 0; j < 2; ++j)
    {
      const math::Vector3d fW = fwd * this->thrust[j] * (this->thrust[j] > 0.0 ? falloff : 1.0);
      fTot += fW;
      tTot += pose.Rot().RotateVector(this->thrustPt[j]).Cross(fW);
    }

    // --- Rudders in the slipstream ------------------------------------------
    if (this->rudderJoints.size() == 2)
    {
      for (int j = 0; j < 2; ++j)
      {
        auto jp = _ecm.Component<components::JointPosition>(this->rudderJoints[j]);
        const double delta = (jp && !jp->Data().empty()) ? jp->Data()[0] : 0.0;
        // +delta (vane rotated CCW) deflects the wash to -Y -> force +Y at stern
        const double side = this->rudderK * std::max(this->thrust[j], 0.0) * std::sin(delta);
        const math::Vector3d fW = pose.Rot().RotateVector(math::Vector3d(0, side, 0));
        fTot += fW;
        tTot += pose.Rot().RotateVector(this->rudderArm).Cross(fW);
      }
    }

    // --- Puff ports: side jets of cushion air at the bow and stern ----------
    // A +Y force on the craft comes from the vent on the right (-Y) side,
    // whose jet blows to -Y; so each force acts at that side's vent.
    if (this->puffForce > 0.0)
    {
      for (int j = 0; j < 2; ++j)                    // 0 = bow, 1 = stern
      {
        const double f = this->puffF[j];
        if (std::abs(f) < 1e-9) continue;
        const math::Vector3d pt((j == 0 ? 1.0 : -1.0) * std::abs(this->puffPt.X()),
                                (f > 0 ? -1.0 : 1.0) * std::abs(this->puffPt.Y()),
                                this->puffPt.Z());
        const math::Vector3d fW = pose.Rot().RotateVector(math::Vector3d(0, f, 0));
        fTot += fW;
        tTot += pose.Rot().RotateVector(pt).Cross(fW);
      }
    }

    // --- Mud: sinkage / rolling resistance when not on the cushion ----------
    if (terrain == Terrain::MUD && !onCushion && speed > 1e-6)
    {
      const double fMud = this->cMud * speed +
          this->mudRR * this->weight * std::tanh(speed / 0.05);
      fTot += -fMud * (vxy / speed);
      tTot.Z() += -this->cMudYaw * w.Z();
    }

    // --- Water: hull drag when floating off-cushion (buoyancy comes from the
    // world's Buoyancy system) --------------------------------------------
    const double wl = reg.WaterLevel(pose.Pos().X(), pose.Pos().Y());
    if (terrain == Terrain::WATER && !onCushion && gapMean < 0.02)
    {
      fTot += -this->cWater * v;                     // incl. vertical (heave)
      tTot.Z() += -this->cWaterYaw * w.Z();
    }
    (void)wl;

    this->hull.AddWorldWrench(_ecm, fTot, tTot);

    // --- State, publishing, logging ----------------------------------------
    std::string state;
    if (!enabled) state = this->ramp > 0 ? "SPIN_DOWN" : "OFF";
    else if (this->ramp < 1.0) state = "SPIN_UP";
    else if (sharing) state = "LOAD_SHARE";
    else state = onCushion ? "HOVER" : "FAN_ON_NO_SUPPORT";

    if (t - this->lastPub >= this->pubPeriod - 1e-9)
    {
      this->lastPub = t;
      msgs::Double g; g.set_data(gapMean); this->pubGap.Publish(g);
      msgs::Double_V cg; for (double x : gap) cg.add_data(x); this->pubCorners.Publish(cg);
      msgs::StringMsg sm; sm.set_data(state); this->pubState.Publish(sm);
      msgs::StringMsg tm; tm.set_data(TerrainName(terrain)); this->pubTerrain.Publish(tm);
      if (this->log.is_open())
      {
        const auto e = pose.Rot().Euler();
        this->log << t << ',' << pose.Pos().X() << ',' << pose.Pos().Y() << ','
                  << pose.Pos().Z() << ',' << e.X() << ',' << e.Y() << ',' << e.Z() << ','
                  << v.X() << ',' << v.Y() << ',' << speed << ','
                  << gap[0] << ',' << gap[1] << ',' << gap[2] << ',' << gap[3] << ','
                  << gapMean << ',' << fCushion << ',' << support << ','
                  << state << ',' << TerrainName(terrain) << ','
                  << this->thrust[0] << ',' << this->thrust[1] << ','
                  << this->rudderCmd << ',' << this->puffF[0] << ',' << this->puffF[1];
        for (const auto &j : this->logJoints)
        {
          auto jp = _ecm.Component<components::JointPosition>(j.second);
          this->log << ',' << ((jp && !jp->Data().empty()) ? jp->Data()[0] : NAN);
        }
        this->log << '\n';
      }
    }
  }

  private: void OnEnable(const msgs::Boolean &_m)
  { std::lock_guard<std::mutex> l(this->mutex); this->hoverEnabled = _m.data(); }
  private: void OnLiftShare(const msgs::Double &_m)
  { std::lock_guard<std::mutex> l(this->mutex); this->liftShare = _m.data(); }
  // Direct fan commands switch velocity mode off (manual / scripted tests)
  private: void OnThrustLeft(const msgs::Double &_m)
  { std::lock_guard<std::mutex> l(this->mutex); this->thrustCmd[0] = _m.data(); this->velMode = false; }
  private: void OnThrustRight(const msgs::Double &_m)
  { std::lock_guard<std::mutex> l(this->mutex); this->thrustCmd[1] = _m.data(); this->velMode = false; }
  private: void OnHeading(const msgs::Double &_m)
  { std::lock_guard<std::mutex> l(this->mutex); this->headingCmd = _m.data(); }
  private: void OnTurnAids(const msgs::Boolean &_m)
  { std::lock_guard<std::mutex> l(this->mutex); this->turnAids = _m.data(); }

  // Yaw moment (N m, + = nose left) the rudders make at their current angles
  private: double RudderMoment(const EntityComponentManager &_ecm) const
  {
    double m = 0.0;
    for (size_t j = 0; j < this->rudderJoints.size() && j < 2; ++j)
    {
      auto jp = _ecm.Component<components::JointPosition>(this->rudderJoints[j]);
      const double delta = (jp && !jp->Data().empty()) ? jp->Data()[0] : 0.0;
      m += this->rudderArm.X() * this->rudderK * std::max(this->thrust[j], 0.0) *
           std::sin(delta);
    }
    return m;
  }

  private: void OnCmdVel(const msgs::Twist &_m)
  {
    std::lock_guard<std::mutex> l(this->mutex);
    this->velMode = true;
    this->velCmd[0] = _m.linear().x();
    this->velCmd[1] = _m.angular().z();
    this->velCmdStamp = -1;          // re-stamped with sim time in PreUpdate
  }

  private: static constexpr double kRayStartOffset = 0.002;

  private: Model model{kNullEntity};
  private: Entity hullEntity{kNullEntity};
  private: Link hull{kNullEntity};
  private: std::string name;
  private: std::vector<math::Vector3d> corners;
  private: std::array<math::Vector3d, 2> thrustPt;
  private: double hTarget{0.04}, k{1500}, c{130}, fMaxFactor{2}, spinup{1.5};
  private: double b1{5}, b2{6}, bYaw{4}, maxThrust{30}, thrustTau{0.3};
  private: double reverseFrac{0.5}, yawReserve{0.0};
  private: double cMud{150}, mudRR{0.25}, cMudYaw{20}, rayRange{1.0};
  private: double waterGlideFactor{1.3}, cWater{60}, cWaterYaw{10};
  private: double cConst{0}, humpDrag{0}, humpSpeed{3.0}, falloffSpeed{0};
  private: bool overWater{false};

  // Wave-making drag over water: rises to water_hump_drag at the hump speed,
  // then decays as (v_hump / v)^2 once the craft is over the hump
  private: double HumpDrag(double _v) const
  {
    if (this->humpDrag <= 0.0 || this->humpSpeed <= 0.0) return 0.0;
    const double r = _v / this->humpSpeed;
    return this->humpDrag * (r <= 1.0 ? r * r : 1.0 / (r * r));
  }

  // Share of static thrust a ducted fan still makes at forward speed _v
  private: double FalloffFactor(double _v) const
  {
    if (this->falloffSpeed <= 0.0 || _v <= 0.0) return 1.0;
    return std::max(0.0, 1.0 - _v / this->falloffSpeed);
  }
  private: double lambda{0.05}, rudderK{0.55}, hdgKp{40}, hdgKd{25};
  private: double spdKp{40}, spdKi{10}, yawKp{30}, cmdTimeout{0.5}, spdInt{0};
  private: double latFactor{3.0};
  private: bool velMode{false};
  private: std::array<double, 2> velCmd{{0, 0}};
  private: double velCmdStamp{-1};
  private: double headingCmd{std::numeric_limits<double>::quiet_NaN()};
  private: math::Vector3d rudderArm{-0.62, 0, 0.2};
  private: std::vector<Entity> rudderJoints;
  private: bool rudderControl{false}, turnAids{true};
  private: double rudderMax{0.44}, rudderCmd{0}, lastRudderPub{-1e9};
  private: double rudderMinThrust{0}, rudderRate{0};
  private: double puffForce{0}, puffTau{0.15}, puffLatGain{0}, lastSupport{0};
  private: math::Vector3d puffPt{0.8, 0.68, 0.0};
  private: std::array<double, 2> puffF{{0, 0}};      // bow, stern side force (N, +Y)
  private: std::array<transport::Node::Publisher, 2> pubRudder;
  private: double shutterTravel{0};
  private: std::array<transport::Node::Publisher, 4> pubShutter;   // bow L/R, stern L/R
  private: double weight{0}, f0{0}, ramp{0};
  private: std::array<double, 2> thrust{{0, 0}};
  private: std::array<double, 4> prevGap{{1, 1, 1, 1}};
  private: std::array<double, 2> thrustCmd{{0, 0}};
  private: bool useRaycast{true}, rayWorks{false}, hoverEnabled{false};
  private: double liftShare{std::numeric_limits<double>::quiet_NaN()};
  private: bool configured{false};
  private: double pubPeriod{0.05}, lastPub{-1e9};
  private: std::mutex mutex;
  private: transport::Node node;
  private: transport::Node::Publisher pubGap, pubCorners, pubState, pubTerrain;
  private: std::ofstream log;
  private: std::vector<std::pair<std::string, Entity>> logJoints;
};
}  // namespace hover

GZ_ADD_PLUGIN(hover::AirCushion, gz::sim::System,
              hover::AirCushion::ISystemConfigure,
              hover::AirCushion::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(hover::AirCushion, "hover::AirCushion")
