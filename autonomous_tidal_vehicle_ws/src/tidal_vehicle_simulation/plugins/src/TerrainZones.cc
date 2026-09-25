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
//   </plugin>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Vector2.hh>
#include <gz/plugin/RegisterMore.hh>  // 2nd+ file in this library
#include <gz/sim/System.hh>

#include "hover_plugins/TerrainRegistry.hh"

namespace hover
{
class TerrainZones : public gz::sim::System, public gz::sim::ISystemConfigure
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
        zones.push_back(z);
        gzmsg << "[TerrainZones] " << z.name << " " << TerrainName(z.type)
              << " x[" << z.xmin << ", " << z.xmax << "] y[" << z.ymin << ", "
              << z.ymax << "]" << (z.type == Terrain::WATER ?
                 " level " + std::to_string(z.level) : "") << std::endl;
      }
    }
    TerrainRegistry::Instance().Set(zones, ground);
  }
};
}  // namespace hover

GZ_ADD_PLUGIN(hover::TerrainZones, gz::sim::System,
              hover::TerrainZones::ISystemConfigure)
GZ_ADD_PLUGIN_ALIAS(hover::TerrainZones, "hover::TerrainZones")
