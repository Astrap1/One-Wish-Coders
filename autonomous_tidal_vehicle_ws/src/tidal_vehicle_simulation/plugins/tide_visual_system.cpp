#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/Model.hh>
#include <sdf/Element.hh>

namespace tidal_vehicle_simulation
{
class TideVisualSystem final : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &entity, const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    this->waterEntity = entity;
    this->disabled = std::getenv("TIDE_VISUAL_DISABLED") != nullptr;
    if (sdf && sdf->HasElement("water_model"))
      this->waterModel = sdf->Get<std::string>("water_model");
    if (sdf && sdf->HasElement("initial_level_m"))
      this->initialLevel = sdf->Get<double>("initial_level_m");
    if (sdf && sdf->HasElement("rise_m"))
      this->rise = sdf->Get<double>("rise_m");
    if (sdf && sdf->HasElement("duration_s"))
      this->duration = std::max(0.1, sdf->Get<double>("duration_s"));
  }

  void PreUpdate(const gz::sim::UpdateInfo &info, gz::sim::EntityComponentManager &ecm) override
  {
    if (this->disabled)
      return;
    if (this->waterEntity == gz::sim::kNullEntity)
      return;
    if (!this->basePoseCaptured)
    {
      if (const auto *pose = ecm.Component<gz::sim::components::Pose>(this->waterEntity))
      {
        this->basePose = pose->Data();
        this->basePoseCaptured = true;
      }
      else
        return;
    }
    const double elapsed = std::max(0.0, std::chrono::duration<double>(info.simTime).count());
    const double fraction = std::min(1.0, elapsed / this->duration);
    auto pose = this->basePose;
    pose.Pos().Z(this->basePose.Pos().Z() + this->rise * fraction);
    // Keep the water level in world coordinates; only its height changes.
    pose.Rot().Set(1, 0, 0, 0);
    gz::sim::Model water(this->waterEntity);
    // The water has no collision geometry. Keep it kinematic so its visual
    // pose can be commanded without gravity or physics interaction.
    water.SetStatic(ecm, false);
    water.SetGravityEnabled(ecm, false);
    water.SetWorldPoseCmd(ecm, pose);
  }

private:
  std::string waterModel{"tidal_channel_water"};
  gz::sim::Entity waterEntity{gz::sim::kNullEntity};
  gz::math::Pose3d basePose{};
  bool basePoseCaptured{false};
  bool disabled{false};
  double initialLevel{0.02};
  double rise{0.55};
  double duration{120.0};
};
}

GZ_ADD_PLUGIN(tidal_vehicle_simulation::TideVisualSystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
