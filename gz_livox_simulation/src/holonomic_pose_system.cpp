#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/twist.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>
#include <sdf/Element.hh>

namespace gz_livox_simulation {
namespace {
constexpr char kDefaultCommandTopic[] = "/cmd_vel_chassis";
using SteadyClock = std::chrono::steady_clock;

struct PlanarVelocity final {
  double forward{0.0};
  double lateral{0.0};
  double yaw{0.0};
};
}

// Keep the installed plugin name for existing launch/model compatibility.
// Motion is produced by bounded forces, never by teleporting the model pose.
class HolonomicPoseSystem final : public gz::sim::System,
                                   public gz::sim::ISystemConfigure,
                                   public gz::sim::ISystemPreUpdate,
                                   public gz::sim::ISystemReset {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    gz::sim::Model model(entity);
    link_ = gz::sim::Link(model.CanonicalLink(ecm));
    if (!model.Valid(ecm) || !link_.Valid(ecm)) {
      gzerr << "Holonomic drive requires a model with a canonical link\n";
      return;
    }
    timeout_ = sdf->Get<double>("command_timeout", 0.5).first;
    gain_ = sdf->Get<double>("velocity_gain", 20.0).first;
    // Default Ki = Kp^2/4 makes the loop critically damped.
    integral_gain_ =
        sdf->Get<double>("velocity_integral_gain", gain_ * gain_ / 4.0).first;
    max_acceleration_ = sdf->Get<double>("max_acceleration", 6.0).first;
    max_alpha_ = sdf->Get<double>("max_angular_acceleration", 6.0).first;
    if (!std::isfinite(timeout_) || timeout_ <= 0.0 ||
        !std::isfinite(gain_) || gain_ <= 0.0 ||
        !std::isfinite(integral_gain_) || integral_gain_ < 0.0 ||
        !std::isfinite(max_acceleration_) || max_acceleration_ <= 0.0 ||
        !std::isfinite(max_alpha_) || max_alpha_ <= 0.0) {
      gzerr << "Invalid holonomic drive timeout/gain/acceleration limits\n";
      return;
    }
    link_.EnableVelocityChecks(ecm);
    const std::string topic =
        sdf->Get<std::string>("topic", kDefaultCommandTopic).first;
    configured_ = transport_.Subscribe(topic, &HolonomicPoseSystem::OnCommand,
                                       this);
    if (!configured_) {
      gzerr << "Failed to subscribe to holonomic command topic " << topic
            << '\n';
    }
  }

  void Reset(const gz::sim::UpdateInfo &,
             gz::sim::EntityComponentManager &) override {
    ClearCommand();
    last_sim_time_ = {};
    was_paused_ = true;
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!configured_) {
      return;
    }
    if (info.paused || info.simTime < last_sim_time_ ||
        info.dt < SteadyClock::duration::zero()) {
      ClearCommand();
      last_sim_time_ = info.simTime;
      was_paused_ = true;
      return;
    }
    if (was_paused_) {
      ClearCommand();
      was_paused_ = false;
    }
    last_sim_time_ = info.simTime;
    if (info.dt <= SteadyClock::duration::zero()) {
      return;
    }

    PlanarVelocity command;
    {
      const std::lock_guard<std::mutex> lock(command_mutex_);
      command_sim_age_ += std::chrono::duration<double>(info.dt).count();
      const double wall_age =
          std::chrono::duration<double>(SteadyClock::now() - received_at_).count();
      if (command_received_ && wall_age <= timeout_ &&
          command_sim_age_ <= timeout_) {
        command = command_;
      } else {
        command_received_ = false;
      }
    }

    const auto velocity = link_.WorldLinearVelocity(ecm);
    const auto angular = link_.WorldAngularVelocity(ecm);
    const auto inertia = link_.WorldInertial(ecm);
    if (!velocity || !angular || !inertia) {
      return;
    }
    const double yaw = gz::sim::worldPose(link_.Entity(), ecm).Rot().Yaw();
    const gz::math::Vector3d error{
        std::cos(yaw) * command.forward - std::sin(yaw) * command.lateral -
            velocity->X(),
        std::sin(yaw) * command.forward + std::cos(yaw) * command.lateral -
            velocity->Y(),
        0.0};
    // PI velocity loop, like a real chassis motor controller. With P only,
    // a slope's gravity component (g*sin(theta), ~2.9 m/s^2 on the arena's
    // 17 deg ramps) needs a large steady velocity error, so low commands
    // stall at the foot of a ramp and zero commands slide back down.
    gz::math::Vector3d acceleration = gain_ * error + integral_;
    if (acceleration.Length() > max_acceleration_) {
      acceleration *= max_acceleration_ / acceleration.Length();
    } else {
      // Conditional integration: no windup while saturated, e.g. pushing a wall.
      integral_ += integral_gain_ *
                   std::chrono::duration<double>(info.dt).count() * error;
      if (integral_.Length() > max_acceleration_) {
        integral_ *= max_acceleration_ / integral_.Length();
      }
    }
    const double alpha = std::clamp(gain_ * (command.yaw - angular->Z()),
                                    -max_alpha_, max_alpha_);
    // Leave gravity, vertical motion, roll/pitch and contact resolution to
    // Physics. A velocity setter would discard the contact wrench each step.
    link_.AddWorldWrench(
        ecm, inertia->MassMatrix().Mass() * acceleration,
        {0.0, 0.0, inertia->MassMatrix().Izz() * alpha});
  }

private:
  void ClearCommand() {
    const std::lock_guard<std::mutex> lock(command_mutex_);
    command_ = {};
    command_received_ = false;
    command_sim_age_ = 0.0;
    integral_ = gz::math::Vector3d::Zero;
  }

  void OnCommand(const gz::msgs::Twist &message) {
    const std::lock_guard<std::mutex> lock(command_mutex_);
    const PlanarVelocity next{message.linear().x(), message.linear().y(),
                              message.angular().z()};
    const bool finite = std::isfinite(next.forward) &&
                        std::isfinite(next.lateral) && std::isfinite(next.yaw);
    command_ = finite ? next : PlanarVelocity{};
    command_received_ = finite;
    received_at_ = SteadyClock::now();
    command_sim_age_ = 0.0;
  }

  gz::sim::Link link_{gz::sim::kNullEntity};
  gz::transport::Node transport_;
  std::mutex command_mutex_;
  PlanarVelocity command_;
  SteadyClock::time_point received_at_{};
  SteadyClock::duration last_sim_time_{};
  double command_sim_age_{0.0};
  double timeout_{0.5};
  double gain_{20.0};
  double integral_gain_{100.0};
  gz::math::Vector3d integral_{gz::math::Vector3d::Zero};
  double max_acceleration_{6.0};
  double max_alpha_{6.0};
  bool configured_{false};
  bool command_received_{false};
  bool was_paused_{false};
};
}

GZ_ADD_PLUGIN(gz_livox_simulation::HolonomicPoseSystem, gz::sim::System,
              gz_livox_simulation::HolonomicPoseSystem::ISystemConfigure,
              gz_livox_simulation::HolonomicPoseSystem::ISystemPreUpdate,
              gz_livox_simulation::HolonomicPoseSystem::ISystemReset)
GZ_ADD_PLUGIN_ALIAS(gz_livox_simulation::HolonomicPoseSystem,
                    "gz_livox_simulation::HolonomicPoseSystem")
