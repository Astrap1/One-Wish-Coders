// gz_sink: subscribe to a gz-transport image topic and discard the messages.
// Gazebo only renders a camera when something is subscribed to it; this
// "connects" the clip camera so its <save> option writes frames to disk.
#include <gz/msgs/image.pb.h>
#include <gz/transport/Node.hh>
#include <iostream>
int main(int argc, char **argv)
{
  if (argc < 2) { std::cerr << "usage: gz_sink <topic>\n"; return 1; }
  gz::transport::Node node;
  size_t n = 0;
  std::function<void(const gz::msgs::Image &)> cb =
      [&n](const gz::msgs::Image &) { ++n; };
  node.Subscribe(argv[1], cb);
  gz::transport::waitForShutdown();
  std::cerr << "gz_sink received " << n << " images\n";
  return 0;
}
