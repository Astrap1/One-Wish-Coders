// TerrainZones — world system that declares where the mud and water are.
//
// This is the "map" that the simulated terrain classifier and the AirCushion
// mud / water logic read. It is ground truth, not perception.
//
//   <plugin filename="hover_plugins" name="hover::TerrainZones">
//     <ground_height>0</ground_height>
//     <zone><name>mudflat</name><type>MUD</type><min>3 -5</min><max>9 5</max></zone>
//     <zone><name>channel</name><type>WATER</type><min>12 -5</min><max>18 5</max>
//           <level>0.25</level></zone>
//     <!-- rising tide: surface goes from level to level + rise between
//          rise_start and rise_start + rise_duration (sim time, s) -->
//     <zone><name>tide</name><type>WATER</type><min>-60 -60</min><max>60 60</max>
//           <level>-1.8</level><rise>2.4</rise><rise_start>0</rise_start>
//           <rise_duration>30</rise_duration></zone>
//   </plugin>
//
// A WATER zone only counts where its surface is above the ground the vehicle
// measures (AirCushion ray casts), so one large zone can flood sloping
// terrain gradually.
#include <memory>
#include <string>
#include <vector>

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/math/Vector2.hh>
#include <gz/plugin/RegisterMore.hh>  // 2nd+ file in this library
#include <gz/sim/System.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include "hover_plugins/TerrainRegistry.hh"
#include "hover_plugins/TideProgress.hh"

namespace hover
{
class TerrainZones : public gz::sim::System, public gz::sim::ISystemConfigure,
                     public gz::sim::ISystemPreUpdate
{
  public: void Configure(const gz::sim::Entity &,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &,
                         gz::sim::EventManager &) override
  {
    std::vector<Zone> zones;
    const double ground = _sdf->Get<double>("ground_height", 0.0).first;
    if (_sdf->HasElement("zone"))
    {
      for (auto e = _sdf->FindElement("zone"); e; e = e->GetNextElement("zone"))
      {
        Zone z;
        z.name = e->Get<std::string>("name", "zone").first;
        const std::string type = e->Get<std::string>("type", "FIRM").first;
        z.type = type == "MUD" ? Terrain::MUD
               : type == "WATER" ? Terrain::WATER : Terrain::FIRM;
        const auto mn = e->Get<gz::math::Vector2d>("min", gz::math::Vector2d()).first;
        const auto mx = e->Get<gz::math::Vector2d>("max", gz::math::Vector2d()).first;
        z.xmin = std::min(mn.X(), mx.X()); z.xmax = std::max(mn.X(), mx.X());
        z.ymin = std::min(mn.Y(), mx.Y()); z.ymax = std::max(mn.Y(), mx.Y());
        z.level = e->Get<double>("level", 0.0).first;
        z.rise = e->Get<double>("rise", 0.0).first;
        z.riseStart = e->Get<double>("rise_start", 0.0).first;
        z.riseDuration = e->Get<double>("rise_duration", 1.0).first;
        this->rising |= z.type == Terrain::WATER && z.rise != 0.0;
        zones.push_back(z);
        gzmsg << "[TerrainZones] " << z.name << " " << TerrainName(z.type)
              << " x[" << z.xmin << ", " << z.xmax << "] y[" << z.ymin << ", "
              << z.ymax << "]" << (z.type == Terrain::WATER ?
                 " level " + std::to_string(z.level) +
                 (z.rise != 0.0 ? " rising " + std::to_string(z.rise) + " m over " +
                  std::to_string(z.riseDuration) + " s" : "") : "") << std::endl;
      }
    }
    TerrainRegistry::Instance().Set(zones, ground);
    this->node.Subscribe("/scenario_event", &TerrainZones::OnScenarioEvent, this);
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &) override
  {
    if (!this->rising) return;
    const double simTime = std::chrono::duration<double>(_info.simTime).count();
    TerrainRegistry::Instance().Update(this->progress.Advance(simTime, _info.paused));
  }

  private: void OnScenarioEvent(const gz::msgs::StringMsg &_message)
  {
    this->progress.QueueEvent(_message.data());
  }

  private: bool rising{false};
  private: TideProgress progress;
  private: gz::transport::Node node;
};
}  // namespace hover

GZ_ADD_PLUGIN(hover::TerrainZones, gz::sim::System,
              hover::TerrainZones::ISystemConfigure,
              hover::TerrainZones::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(hover::TerrainZones, "hover::TerrainZones")
