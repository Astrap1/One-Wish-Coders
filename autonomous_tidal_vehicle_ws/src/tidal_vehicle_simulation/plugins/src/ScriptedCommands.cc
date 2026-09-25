// ScriptedCommands — world system that publishes gz-transport messages at
// given simulation times. Used to script repeatable test scenarios (the A/B
// mud test, the 60 s gap-hold test) without a ROS stack in the loop.
//
//   <plugin filename="hover_plugins" name="hover::ScriptedCommands">
//     <command>
//       <time>2.0</time>
//       <topic>/model/hc_hover/hover_enabled</topic>
//       <type>gz.msgs.Boolean</type>
//       <data>data: true</data>          <!-- protobuf text format -->
//     </command>
//     ...
//   </plugin>
#include <gz/msgs/Factory.hh>

#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/plugin/RegisterMore.hh>  // 2nd+ file in this library
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace hover
{
class ScriptedCommands : public gz::sim::System,
                         public gz::sim::ISystemConfigure,
                         public gz::sim::ISystemPreUpdate
{
  private: struct Cmd
  {
    double time{0};
    std::string topic, type, data;
    bool sent{false};
  };

  public: void Configure(const gz::sim::Entity &,
                         const std::shared_ptr<const sdf::Element> &_sdf,
                         gz::sim::EntityComponentManager &,
                         gz::sim::EventManager &) override
  {
    if (!_sdf->HasElement("command")) return;
    for (auto e = _sdf->FindElement("command"); e; e = e->GetNextElement("command"))
    {
      Cmd c;
      c.time = e->Get<double>("time", 0.0).first;
      c.topic = e->Get<std::string>("topic", "").first;
      c.type = e->Get<std::string>("type", "gz.msgs.Double").first;
      c.data = e->Get<std::string>("data", "").first;
      if (c.topic.empty()) continue;
      const std::string key = c.topic + "|" + c.type;
      if (!this->pubs.count(key))
        this->pubs[key] = this->node.Advertise(c.topic, c.type);
      this->cmds.push_back(c);
    }
    gzmsg << "[ScriptedCommands] " << this->cmds.size() << " commands loaded" << std::endl;
  }

  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                         gz::sim::EntityComponentManager &) override
  {
    if (_info.paused) return;
    const double t = std::chrono::duration<double>(_info.simTime).count();
    for (auto &c : this->cmds)
    {
      if (c.sent || t < c.time) continue;
      auto msg = gz::msgs::Factory::New(c.type, c.data);
      if (!msg)
      {
        gzerr << "[ScriptedCommands] cannot build " << c.type << " from ["
              << c.data << "]" << std::endl;
        c.sent = true;
        continue;
      }
      this->pubs[c.topic + "|" + c.type].Publish(*msg);
      gzmsg << "[ScriptedCommands] t=" << t << " " << c.topic << " <- " << c.data
            << std::endl;
      c.sent = true;
    }
  }

  private: gz::transport::Node node;
  private: std::map<std::string, gz::transport::Node::Publisher> pubs;
  private: std::vector<Cmd> cmds;
};
}  // namespace hover

GZ_ADD_PLUGIN(hover::ScriptedCommands, gz::sim::System,
              hover::ScriptedCommands::ISystemConfigure,
              hover::ScriptedCommands::ISystemPreUpdate)
GZ_ADD_PLUGIN_ALIAS(hover::ScriptedCommands, "hover::ScriptedCommands")
