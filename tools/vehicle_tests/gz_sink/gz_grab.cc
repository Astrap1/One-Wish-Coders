// gz_grab: save ONE message from a Gazebo sensor topic, for checking the sensors.
//   gz_grab image <topic> out.ppm       camera image  -> binary PPM (RGB8)
//   gz_grab scan  <topic> out.csv       gpu_lidar LaserScan -> CSV (ring, azimuth, elevation, range)
#include <gz/msgs/image.pb.h>
#include <gz/msgs/laserscan.pb.h>
#include <gz/transport/Node.hh>
#include <atomic>
#include <chrono>
#include <fstream>
#include <iostream>
#include <thread>
int main(int argc, char **argv)
{
  if (argc < 4) { std::cerr << "usage: gz_grab image|scan <topic> <out>\n"; return 1; }
  std::string mode = argv[1], topic = argv[2], out = argv[3];
  gz::transport::Node node;
  std::atomic<bool> done{false};
  if (mode == "image")
  {
    std::function<void(const gz::msgs::Image &)> cb = [&](const gz::msgs::Image &m) {
      if (done) return;
      std::ofstream f(out, std::ios::binary);
      f << "P6\n" << m.width() << " " << m.height() << "\n255\n";
      f.write(m.data().data(), m.width() * m.height() * 3);
      done = true;
    };
    node.Subscribe(topic, cb);
  }
  else
  {
    std::function<void(const gz::msgs::LaserScan &)> cb = [&](const gz::msgs::LaserScan &m) {
      if (done) return;
      std::ofstream f(out);
      f << "ring,azimuth,elevation,range\n";
      const int nh = m.count(), nv = std::max<int>(1, m.vertical_count());
      for (int v = 0; v < nv; ++v)
        for (int h = 0; h < nh; ++h)
        {
          const double az = m.angle_min() + h * m.angle_step();
          const double el = nv > 1 ? m.vertical_angle_min() + v * m.vertical_angle_step() : 0.0;
          f << v << ',' << az << ',' << el << ',' << m.ranges(v * nh + h) << '\n';
        }
      done = true;
    };
    node.Subscribe(topic, cb);
  }
  for (int i = 0; i < 1200 && !done; ++i)
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  std::cerr << (done ? "saved " + out : "timeout waiting for " + topic) << "\n";
  return done ? 0 : 2;
}
