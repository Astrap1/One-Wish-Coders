// TerrainRegistry: process-wide table of terrain zones.
// TerrainZones (a world system) fills it from the world SDF; AirCushion
// (a model system) reads it. Both systems are compiled into the same shared
// library, so they share this singleton.
#ifndef HOVER_PLUGINS_TERRAIN_REGISTRY_HH_
#define HOVER_PLUGINS_TERRAIN_REGISTRY_HH_

#include <mutex>
#include <string>
#include <vector>

namespace hover
{
enum class Terrain { FIRM = 0, MUD = 1, WATER = 2 };

inline const char *TerrainName(Terrain _t)
{
  switch (_t)
  {
    case Terrain::MUD: return "MUD";
    case Terrain::WATER: return "WATER";
    default: return "FIRM";
  }
}

/// Axis-aligned rectangle in world XY.
struct Zone
{
  Terrain type{Terrain::FIRM};
  double xmin{0}, ymin{0}, xmax{0}, ymax{0};
  double level{0};        // WATER: water surface height (m)
  std::string name;
  bool Contains(double _x, double _y) const
  {
    return _x >= xmin && _x <= xmax && _y >= ymin && _y <= ymax;
  }
};

class TerrainRegistry
{
  public: static TerrainRegistry &Instance()
  {
    static TerrainRegistry reg;
    return reg;
  }

  public: void Set(const std::vector<Zone> &_zones, double _groundHeight)
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->zones = _zones;
    this->groundHeight = _groundHeight;
    this->configured = true;
  }

  /// Terrain class at (x, y). WATER wins over MUD wins over FIRM.
  public: Terrain Classify(double _x, double _y) const
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    Terrain t = Terrain::FIRM;
    for (const auto &z : this->zones)
    {
      if (!z.Contains(_x, _y)) continue;
      if (z.type == Terrain::WATER) return Terrain::WATER;
      if (z.type == Terrain::MUD) t = Terrain::MUD;
    }
    return t;
  }

  /// Water surface height at (x, y), or -inf if not over water.
  public: double WaterLevel(double _x, double _y) const
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    double level = -1e9;
    for (const auto &z : this->zones)
      if (z.type == Terrain::WATER && z.Contains(_x, _y) && z.level > level)
        level = z.level;
    return level;
  }

  public: double GroundHeight() const
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    return this->groundHeight;
  }

  private: mutable std::mutex mutex;
  private: std::vector<Zone> zones;
  private: double groundHeight{0.0};
  private: bool configured{false};
};
}  // namespace hover
#endif
