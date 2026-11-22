// Behavior regressions against the actual robot model and Gazebo DART physics.
#include <chrono>
#include <cmath>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <regex>
#include <stdexcept>
#include <string>
#include <thread>

#include <gz/msgs/twist.pb.h>
#include <gz/msgs/world_control.pb.h>
#include <gz/msgs/boolean.pb.h>
#include <gz/sim/Server.hh>
#include <gz/sim/ServerConfig.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

using namespace std::chrono_literals;

void Check(bool value, const std::string &message) {
  if (!value) throw std::runtime_error(message);
}

class Observe final : public gz::sim::System,
                      public gz::sim::ISystemPreUpdate,
                      public gz::sim::ISystemPostUpdate {
public:
  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &ecm) override {
    if (entity == gz::sim::kNullEntity) {
      entity = ecm.EntityByComponents(gz::sim::components::Name("base_link"),
                                      gz::sim::components::Link());
      if (entity != gz::sim::kNullEntity)
        gz::sim::Link(entity).EnableVelocityChecks(ecm);
    }
  }
  void PostUpdate(const gz::sim::UpdateInfo &,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (entity == gz::sim::kNullEntity) return;
    pose = gz::sim::worldPose(entity, ecm);
    velocity = gz::sim::Link(entity).WorldLinearVelocity(ecm).value_or(
        gz::math::Vector3d::Zero);
    angular = gz::sim::Link(entity).WorldAngularVelocity(ecm).value_or(
        gz::math::Vector3d::Zero);
  }
  gz::sim::Entity entity{gz::sim::kNullEntity};
  gz::math::Pose3d pose;
  gz::math::Vector3d velocity, angular;
};

int main(int argc, char **argv) {
  try {
    Check(argc == 2, "robot SDF path required");
    std::ifstream input(argv[1]);
    std::string model((std::istreambuf_iterator<char>(input)), {});
    Check(!model.empty(), "robot model could not be read");
    model = model.substr(model.find("<model "));
    model = model.substr(0, model.rfind("</model>") + 8);
    // Keep the production inertias, joints, drive and collisions. Rendering and
    // lidar are unrelated to these contact/watchdog tests and need no GPU.
    model = std::regex_replace(model, std::regex("<sensor[\\s\\S]*?</sensor>"), "");
    model = std::regex_replace(model,
        std::regex("<plugin filename=\"gz-sim-imu-system\"[^>]*/>"), "");
    model.insert(model.find('>') + 1, "<pose>0 0 0.2 0 0 0</pose>");
    const std::string world = R"(<sdf version="1.9"><world name="drive_test">
      <physics name="p" type="dart"><max_step_size>0.002</max_step_size>
        <real_time_factor>0</real_time_factor>
        <dart><collision_detector>bullet</collision_detector></dart></physics>
      <gravity>0 0 -9.81</gravity>
      <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
      <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
      <model name="floor"><static>true</static><link name="l"><collision name="c">
        <pose>0 0 -0.1 0 0 0</pose><geometry><box><size>30 30 0.2</size></box></geometry>
      </collision></link></model>
      <model name="wall"><static>true</static><pose>2 0 0.6 0 0 0</pose>
        <link name="l"><collision name="c"><geometry><box><size>0.1 20 1.2</size></box></geometry>
        </collision></link></model>
      <model name="ramp"><static>true</static><pose>0.46 -3 -0.25 0 -0.297 0</pose>
        <link name="l"><collision name="c"><geometry><box><size>4 2 0.5</size></box></geometry>
        </collision></link></model>)" + model + "</world></sdf>";
    gz::sim::ServerConfig config;
    config.SetSdfString(world);
    config.SetPhysicsEngine("gz-physics-dartsim-plugin");
    gz::sim::Server server(config);
    auto observe = std::make_shared<Observe>();
    Check(server.AddSystem(observe).value_or(false), "observer not installed");
    gz::transport::Node transport;
    auto publisher = transport.Advertise<gz::msgs::Twist>("/cmd_vel_chassis");
    for (int i = 0; i < 100 && !publisher.HasConnections(); ++i)
      std::this_thread::sleep_for(10ms);
    Check(publisher.HasConnections(), "drive did not subscribe");
    auto run = [&](int steps, bool paused = false) {
      Check(server.Run(true, steps, paused), "Gazebo run failed");
    };
    auto send = [&](double x, double y = 0, double yaw = 0) {
      gz::msgs::Twist message;
      message.mutable_linear()->set_x(x);
      message.mutable_linear()->set_y(y);
      message.mutable_angular()->set_z(yaw);
      Check(publisher.Publish(message), "publish failed");
      std::this_thread::sleep_for(10ms);
    };
    auto stopped = [&](const std::string &label) {
      const double speed = std::hypot(observe->velocity.X(), observe->velocity.Y());
      Check(speed < 0.02 && std::abs(observe->angular.Z()) < 0.02,
            label + ": still moving, speed=" + std::to_string(speed));
      std::cout << "PASS " << label << " speed=" << speed << std::endl;
    };
    run(700); // settle onto the floor
    Check(observe->pose.Pos().Z() > 0.04 && observe->pose.Pos().Z() < 0.09,
          "chassis fell through floor");
    for (int i = 0; i < 4; ++i) { send(1); run(50); }
    Check(observe->velocity.X() > 0.8, "drive did not accelerate");
    // No further messages: accelerated sim time must also expire the command.
    run(600);
    stopped("command stream timeout");
    auto position = observe->pose.Pos();
    run(500);
    Check(position.Distance(observe->pose.Pos()) < 0.01,
          "stale command resumed after timeout");
    for (int i = 0; i < 4; ++i) { send(0, 0.8, 0.5); run(50); }
    send(0); run(300);
    stopped("explicit zero including lateral and yaw");
    for (int i = 0; i < 3; ++i) { send(0.7); run(50); }
    send(std::numeric_limits<double>::quiet_NaN()); run(300);
    stopped("non-finite command");
    for (int i = 0; i < 3; ++i) { send(-0.7); run(50); }
    std::this_thread::sleep_for(550ms); run(300);
    stopped("wall-clock timeout");
    send(1); run(150);
    Check(server.RunOnce(true), "paused iteration failed");
    run(500);
    stopped("pause/resume discards old command");
    // Repeated commands remain fresh while pushing into a solid wall.
    for (int i = 0; i < 100; ++i) { send(1); run(50); }
    Check(observe->pose.Pos().X() < 1.81, "chassis crossed wall");
    Check(observe->pose.Pos().X() > 1.65, "chassis never reached wall");
    Check(std::abs(observe->pose.Rot().Roll()) < 0.2 &&
          std::abs(observe->pose.Rot().Pitch()) < 0.2,
          "chassis ejected on wall contact");
    std::cout << "PASS continuous wall contact x=" << observe->pose.Pos().X() << '\n';
    // Actual Gazebo world reset must clear commands in ISystemReset.
    gz::msgs::WorldControl reset;
    reset.mutable_reset()->set_all(true);
    gz::msgs::Boolean reply;
    bool result = false;
    Check(transport.Request("/world/drive_test/control", reset, 1000, reply, result)
              && result && reply.data(), "world reset request failed");
    run(5); run(700);
    stopped("world reset discards old command");
    Check(std::abs(observe->pose.Pos().X()) < 0.02, "reset replayed old velocity");
    // A slow command must still climb a 17 deg ramp (the RMUC arena's slope),
    // the way a speed-controlled real chassis does. P-only control stalled at
    // the foot of the ramp below ~0.3 m/s.
    // Ramp surface meets the floor at x~0.5 and rises toward +x (y -4..-2).
    for (int i = 0; i < 25; ++i) { send(0, -0.6); run(100); }
    send(0); run(300);
    const double ramp_x = observe->pose.Pos().X();
    const double ramp_start_z = observe->pose.Pos().Z();
    for (int i = 0; i < 100; ++i) { send(0.1); run(50); }
    Check(observe->pose.Pos().X() > ramp_x + 0.6,
          "slow command stalled on ramp, x=" + std::to_string(observe->pose.Pos().X()));
    Check(observe->pose.Pos().Z() > ramp_start_z + 0.1, "chassis did not climb the ramp");
    std::cout << "PASS slow ramp climb z=" << observe->pose.Pos().Z() << '\n';
    // Zero command holds position on the slope instead of sliding back.
    const auto on_slope = observe->pose.Pos();
    send(0); run(1000);
    Check(on_slope.Distance(observe->pose.Pos()) < 0.05, "chassis slid down the ramp");
    std::cout << "PASS hold on ramp drift=" << on_slope.Distance(observe->pose.Pos()) << '\n';
    return 0;
  } catch (const std::exception &error) {
    std::cerr << "FAIL " << error.what() << '\n';
    return 1;
  }
}
