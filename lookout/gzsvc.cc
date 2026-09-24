// Persistent Gazebo service client for the stepped walks (built by
// lookout/build_gzsvc.sh into /runs/lookout/bin/gzsvc).
//
// `ign service` is a Ruby CLI that builds a fresh transport node per call:
// ~0.4 s of start-up per call, and ~3 % of calls lost their reply ("Host
// unreachable" in the server log, 10 s timeout) in the walk-driver test. This
// keeps one node and serves requests read from stdin, one per line:
//   pose  <world> <model> <x> <y> <z> <qx> <qy> <qz> <qw>
//   step  <world> <n>          (pause: true, multi_step: n)
//   pause <world> <0|1>
//   remove <world> <model>     (Entity type MODEL)
// and answers each with one line: "ok <tries>" or "fail <tries> <reason>".
// Every request is safe to repeat, so a lost reply is retried (4 tries).
#include <ignition/msgs/boolean.pb.h>
#include <ignition/msgs/entity.pb.h>
#include <ignition/msgs/pose.pb.h>
#include <ignition/msgs/world_control.pb.h>
#include <ignition/transport/Node.hh>

#include <iostream>
#include <sstream>
#include <string>

namespace {
constexpr unsigned kTimeoutMs = 10000;
constexpr int kTries = 4;

template <typename Req>
void call(ignition::transport::Node &node, const std::string &service, const Req &req) {
  std::string why = "timeout";
  for (int i = 1; i <= kTries; ++i) {
    ignition::msgs::Boolean rep;
    bool result = false;
    if (node.Request(service, req, kTimeoutMs, rep, result)) {
      if (result && rep.data()) {
        std::cout << "ok " << i << std::endl;
        return;
      }
      why = "refused";
    } else {
      why = "timeout";
    }
  }
  std::cout << "fail " << kTries << " " << why << std::endl;
}
}  // namespace

int main() {
  ignition::transport::Node node;
  std::string line;
  while (std::getline(std::cin, line)) {
    std::istringstream in(line);
    std::string cmd, world;
    in >> cmd >> world;
    const std::string base = "/world/" + world + "/";
    if (cmd == "pose") {
      std::string model;
      double x, y, z, qx, qy, qz, qw;
      if (!(in >> model >> x >> y >> z >> qx >> qy >> qz >> qw)) {
        std::cout << "fail 0 parse" << std::endl;
        continue;
      }
      ignition::msgs::Pose req;
      req.set_name(model);
      req.mutable_position()->set_x(x);
      req.mutable_position()->set_y(y);
      req.mutable_position()->set_z(z);
      req.mutable_orientation()->set_x(qx);
      req.mutable_orientation()->set_y(qy);
      req.mutable_orientation()->set_z(qz);
      req.mutable_orientation()->set_w(qw);
      call(node, base + "set_pose", req);
    } else if (cmd == "step" || cmd == "pause") {
      int n;
      if (!(in >> n)) {
        std::cout << "fail 0 parse" << std::endl;
        continue;
      }
      ignition::msgs::WorldControl req;
      if (cmd == "step") {
        req.set_pause(true);
        req.set_multi_step(n);
      } else {
        req.set_pause(n != 0);
      }
      call(node, base + "control", req);
    } else if (cmd == "remove") {
      std::string model;
      in >> model;
      ignition::msgs::Entity req;
      req.set_name(model);
      req.set_type(ignition::msgs::Entity::MODEL);
      call(node, base + "remove", req);
    } else {
      std::cout << "fail 0 unknown-command" << std::endl;
    }
  }
  return 0;
}
