#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <gz/math/Quaternion.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/RaycastData.hh>
#include <gz/sim/components/Pose.hh>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sdf/Element.hh>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "gz_livox_simulation/scan_pattern.hpp"

namespace gz_livox_simulation {
namespace {
constexpr std::int64_t kNanosecondsPerSecond = 1000000000LL;
constexpr std::int64_t kScanPeriodNanoseconds = 100000000LL;
constexpr char kPackageUriPrefix[] = "package://";

} // namespace

class LivoxRaycastSystem final : public gz::sim::System,
                                 public gz::sim::ISystemConfigure,
                                 public gz::sim::ISystemPreUpdate,
                                 public gz::sim::ISystemPostUpdate {
public:
  void Configure(const gz::sim::Entity &entity,
                 const std::shared_ptr<const sdf::Element> &sdf,
                 gz::sim::EntityComponentManager &ecm,
                 gz::sim::EventManager &) override {
    try {
      entity_ = entity;
      samples_ = static_cast<std::int64_t>(sdf->Get<int>("samples", 30000).first);
      downsample_ = static_cast<std::int64_t>(sdf->Get<int>("downsample", 1).first);
      min_range_ = sdf->Get<double>("min_range", 0.1).first;
      max_range_ = sdf->Get<double>("max_range", 200.0).first;
      frame_id_ = sdf->Get<std::string>("frame_id", "livox_frame").first;
      topic_ = sdf->Get<std::string>("topic", "/livox/lidar").first;
      // CustomMsg is only consumed by super_lio; set false to skip it.
      publish_custom_ = sdf->Get<bool>("publish_custom_msg", true).first;
      const double scan_rate_hz = sdf->Get<double>("scan_rate_hz", 10.0).first;
      // Livox on-device install attitude (MID360_config.json
      // extrinsic_parameter): rotates the published points from the device
      // frame into the mounting frame. Rays are still cast along the device
      // axes, so an inverted mount (sensor <pose> roll pi) looks down while
      // the published cloud stays upright, as on the real car.
      const auto attitude =
          sdf->Get<gz::math::Vector3d>("install_attitude_rpy",
                                       gz::math::Vector3d::Zero)
              .first;
      install_rotation_ = gz::math::Quaterniond(attitude.X(), attitude.Y(),
                                                attitude.Z());
      if (samples_ <= 0 || downsample_ <= 0 || min_range_ < 0.0 ||
          max_range_ <= min_range_) {
        throw std::invalid_argument(
            "samples/downsample and range limits are invalid");
      }
      if (scan_rate_hz != 10.0) {
        throw std::invalid_argument(
            "scan_rate_hz must be 10 to preserve Livox offset_time semantics");
      }

      std::string csv_path = sdf->Get<std::string>("csv_file_name");
      if (csv_path.rfind(kPackageUriPrefix, 0) == 0U) {
        const std::string package_path =
            csv_path.substr(sizeof(kPackageUriPrefix) - 1U);
        const auto separator = package_path.find('/');
        if (separator == std::string::npos) {
          throw std::runtime_error(
              "package CSV URI must include a relative path: " + csv_path);
        }
        csv_path = ament_index_cpp::get_package_share_directory(
                       package_path.substr(0, separator)) +
                   "/" + package_path.substr(separator + 1U);
      }
      pattern_ = ReadPatternCsv(csv_path);
      const auto initial_scan = BuildScan(pattern_, 0, samples_, downsample_);

      raycast_entity_ = ecm.CreateEntity();
      ecm.CreateComponent(
          raycast_entity_,
          gz::sim::components::Pose(gz::sim::worldPose(entity_, ecm)));
      ecm.CreateComponent(raycast_entity_,
                          gz::sim::components::RaycastData());

      if (!rclcpp::ok()) {
        rclcpp::init(0, nullptr);
      }
      node_ = rclcpp::Node::make_shared("gz_livox_raycast_system");
      if (publish_custom_) {
        custom_publisher_ =
            node_->create_publisher<livox_ros_driver2::msg::CustomMsg>(
                topic_, rclcpp::SensorDataQoS());
      }
      cloud_publisher_ = node_->create_publisher<sensor_msgs::msg::PointCloud2>(
          topic_ + "/pointcloud", rclcpp::SensorDataQoS());
      configured_ = true;
      gzmsg << "Livox rotating raycast configured: rows=" << pattern_.size()
            << " rays/scan=" << initial_scan.size() << '\n';
    } catch (const std::exception &error) {
      gzerr << "Livox system configuration invalid: " << error.what() << '\n';
    }
  }

  void PreUpdate(const gz::sim::UpdateInfo &info,
                 gz::sim::EntityComponentManager &ecm) override {
    if (!configured_ || info.paused || request_pending_) {
      return;
    }

    auto *pose = ecm.Component<gz::sim::components::Pose>(raycast_entity_);
    auto *raycast =
        ecm.Component<gz::sim::components::RaycastData>(raycast_entity_);
    if (pose == nullptr || raycast == nullptr) {
      gzerr << "Livox raycast ECS query entity disappeared\n";
      configured_ = false;
      return;
    }
    if (info.simTime < next_scan_time_) {
      raycast->Data().rays.clear();
      raycast->Data().results.clear();
      return;
    }

    pose->Data() = gz::sim::worldPose(entity_, ecm);
    pending_scan_ =
        BuildScan(pattern_, current_start_index_, samples_, downsample_);
    auto &data = raycast->Data();
    data.rays.clear();
    data.results.clear();
    data.rays.reserve(pending_scan_.size());
    for (const auto &sample : pending_scan_) {
      double x{};
      double y{};
      double z{};
      AxisFromRotate(sample.rotate, x, y, z);
      const gz::math::Vector3d axis{x, y, z};
      data.rays.push_back({min_range_ * axis, max_range_ * axis});
    }

    pending_time_ns_ =
        std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime)
            .count();
    request_pending_ = true;
    next_scan_time_ =
        info.simTime + std::chrono::nanoseconds(kScanPeriodNanoseconds);
  }

  void PostUpdate(const gz::sim::UpdateInfo &,
                  const gz::sim::EntityComponentManager &ecm) override {
    if (!request_pending_) {
      return;
    }
    const auto *raycast =
        ecm.Component<gz::sim::components::RaycastData>(raycast_entity_);
    if (raycast == nullptr) {
      return;
    }
    const auto &results = raycast->Data().results;
    if (results.size() != pending_scan_.size()) {
      gzerr << "Livox physics returned " << results.size() << " results for "
            << pending_scan_.size() << " rays; scan dropped\n";
      request_pending_ = false;
      return;
    }

    livox_ros_driver2::msg::CustomMsg custom;
    custom.header.stamp.sec =
        static_cast<std::int32_t>(pending_time_ns_ / kNanosecondsPerSecond);
    custom.header.stamp.nanosec =
        static_cast<std::uint32_t>(pending_time_ns_ % kNanosecondsPerSecond);
    custom.header.frame_id = frame_id_;
    custom.timebase = static_cast<std::uint64_t>(pending_time_ns_);
    custom.points.reserve(results.size());

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = custom.header;
    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(results.size());
    sensor_msgs::PointCloud2Iterator<float> cloud_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> cloud_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> cloud_z(cloud, "z");

    for (std::size_t index = 0; index < results.size(); ++index) {
      double range = results[index].point.Length();
      if (!std::isfinite(range) || range <= min_range_ || range >= max_range_) {
        range = 0.0;
      }
      double axis_x{};
      double axis_y{};
      double axis_z{};
      AxisFromRotate(pending_scan_[index].rotate, axis_x, axis_y, axis_z);

      const gz::math::Vector3d mounted =
          install_rotation_ * gz::math::Vector3d(range * axis_x, range * axis_y,
                                                 range * axis_z);
      livox_ros_driver2::msg::CustomPoint point;
      point.offset_time = pending_scan_[index].offset_time_ns;
      point.x = static_cast<float>(mounted.X());
      point.y = static_cast<float>(mounted.Y());
      point.z = static_cast<float>(mounted.Z());
      point.reflectivity = 0U;
      point.tag = 0U;
      point.line = 0U;
      custom.points.push_back(point);

      *cloud_x = point.x;
      *cloud_y = point.y;
      *cloud_z = point.z;
      ++cloud_x;
      ++cloud_y;
      ++cloud_z;
    }

    custom.point_num = static_cast<std::uint32_t>(custom.points.size());
    if (custom_publisher_) {
      custom_publisher_->publish(custom);
    }
    cloud_publisher_->publish(cloud);
    current_start_index_ += samples_;
    request_pending_ = false;
  }

private:
  gz::sim::Entity entity_{gz::sim::kNullEntity};
  gz::sim::Entity raycast_entity_{gz::sim::kNullEntity};
  std::vector<RotateInfo> pattern_;
  std::vector<RaySample> pending_scan_;
  std::int64_t samples_{0};
  std::int64_t downsample_{1};
  std::int64_t current_start_index_{0};
  std::int64_t pending_time_ns_{0};
  gz::math::Quaterniond install_rotation_{gz::math::Quaterniond::Identity};
  double min_range_{0.1};
  double max_range_{200.0};
  std::string frame_id_;
  std::string topic_;
  std::chrono::steady_clock::duration next_scan_time_{0};
  bool configured_{false};
  bool request_pending_{false};
  rclcpp::Node::SharedPtr node_;
  bool publish_custom_{true};
  rclcpp::Publisher<livox_ros_driver2::msg::CustomMsg>::SharedPtr
      custom_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_publisher_;
};
} // namespace gz_livox_simulation

GZ_ADD_PLUGIN(gz_livox_simulation::LivoxRaycastSystem, gz::sim::System,
              gz_livox_simulation::LivoxRaycastSystem::ISystemConfigure,
              gz_livox_simulation::LivoxRaycastSystem::ISystemPreUpdate,
              gz_livox_simulation::LivoxRaycastSystem::ISystemPostUpdate)
GZ_ADD_PLUGIN_ALIAS(gz_livox_simulation::LivoxRaycastSystem,
                    "gz_livox_simulation::LivoxRaycastSystem")
