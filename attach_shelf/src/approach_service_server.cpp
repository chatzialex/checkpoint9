#include "attach_shelf/srv/go_to_loading.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

using LaserScan = sensor_msgs::msg::LaserScan;
using GoToLoading = attach_shelf::srv::GoToLoading;
using Twist = geometry_msgs::msg::Twist;
using Empty = std_msgs::msg::Empty;

class ApproachServiceServer : public rclcpp::Node {
public:
  ApproachServiceServer();

private:
  constexpr static char kNodeName[]{"approach_service_server"};
  constexpr static char kServiceName[]{"approach_shelf"};
  constexpr static char kScanTopicName[]{"/scan"};
  constexpr static char kCmdTopicName[]{"robot/cmd_vel"};
  constexpr static char kElevatorUpTopicName[]{"elevator_up"};

  constexpr static char kOdomFrame[]{"robot_odom"};
  constexpr static char kBaseFootprintFrame[]{"robot_base_footprint"};
  constexpr static char kLaserFrame[]{"robot_front_laser_link"};
  constexpr static char kCartFrame[]{"cart_frame"};

  constexpr static auto kTimerPeriod{100ms};
  constexpr static double kIntensityThreshold{8000};
  constexpr static double kMinLegsAngularDistance{0.5};
  constexpr static double kKpYaw{1.0};
  constexpr static double kKpDdistance{0.5};
  constexpr static double kGoalPosTol{5e-2};
  constexpr static auto kMotionTimeout{10s};

  void subscription_cb(const std::shared_ptr<const LaserScan> msg);
  void service_cb(const std::shared_ptr<GoToLoading::Request> req,
                  const std::shared_ptr<GoToLoading::Response> res);

  std::optional<Eigen::Isometry3d> compCenter();
  std::optional<Eigen::Isometry3d> getTransform(const std::string &source_frame,
                                                const std::string &dest_frame);
  bool move_to_goal(
      const std::string &dest_frame,
      const Eigen::Isometry3d &dest_to_goal = Eigen::Isometry3d::Identity());

  rclcpp::Subscription<LaserScan>::SharedPtr subscription_{};
  rclcpp::Publisher<Twist>::SharedPtr publisher_{};
  rclcpp::Publisher<Empty>::SharedPtr elevator_up_publisher_{};
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_{};
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{};
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::Service<GoToLoading>::SharedPtr service_{};

  std::optional<LaserScan> scan_{std::nullopt};
};

ApproachServiceServer::ApproachServiceServer()
    : Node{kNodeName}, subscription_{this->create_subscription<LaserScan>(
                           kScanTopicName, 1,
                           std::bind(&ApproachServiceServer::subscription_cb,
                                     this, std::placeholders::_1))},
      publisher_{this->create_publisher<Twist>(kCmdTopicName, 1)},
      elevator_up_publisher_{
          this->create_publisher<Empty>(kElevatorUpTopicName, 1)},
      tf_buffer_{std::make_unique<tf2_ros::Buffer>(this->get_clock())},
      tf_listener_{
          std::make_shared<tf2_ros::TransformListener>(*this->tf_buffer_)},
      tf_static_broadcaster_{
          std::make_shared<tf2_ros::StaticTransformBroadcaster>(this)},
      service_{this->create_service<GoToLoading>(
          kServiceName,
          std::bind(&ApproachServiceServer::service_cb, this,
                    std::placeholders::_1, std::placeholders::_2))} {
  RCLCPP_INFO(this->get_logger(), "Started %s service server.", kServiceName);
}

void ApproachServiceServer::subscription_cb(
    const std::shared_ptr<const LaserScan> msg) {
  scan_ = *msg;
}

void ApproachServiceServer::service_cb(
    const std::shared_ptr<GoToLoading::Request> req,
    const std::shared_ptr<GoToLoading::Response> res) {
  RCLCPP_INFO(this->get_logger(), "%s service called", kServiceName);

  res->complete = false;

  // Publish center frame.

  if (!scan_) {
    RCLCPP_WARN(this->get_logger(), "Laser scan not received yet.");
    return;
  }
  std::optional<Eigen::Isometry3d> scan_to_center = compCenter();
  if (!scan_to_center) {
    return;
  }
  std::optional<Eigen::Isometry3d> odom_to_scan{
      getTransform(kOdomFrame, kLaserFrame)};
  if (!odom_to_scan) {
    return;
  }
  std::optional<Eigen::Isometry3d> odom_to_base_footprint{
      getTransform(kOdomFrame, kBaseFootprintFrame)};
  if (!odom_to_base_footprint) {
    return;
  }

  Eigen::Isometry3d odom_to_center_tf;
  odom_to_center_tf.translation() =
      (odom_to_scan.value() * scan_to_center.value()).translation();
  odom_to_center_tf.linear() = odom_to_base_footprint.value().linear();
  geometry_msgs::msg::TransformStamped odom_to_center{
      tf2::eigenToTransform(odom_to_center_tf)};
  odom_to_center.header.stamp = this->get_clock()->now();
  odom_to_center.header.frame_id = kOdomFrame;
  odom_to_center.child_frame_id = kCartFrame;
  tf_static_broadcaster_->sendTransform(odom_to_center);
  tf_buffer_->setTransform(odom_to_center, "default_authority", true);

  // Perform the movement if requested.

  Eigen::Isometry3d cart_to_goal = Eigen::Isometry3d::Identity();

  if (!req->attach_to_shelf) {
    goto end;
  }

  // Move to center frame

  RCLCPP_INFO(this->get_logger(), "Moving towards the center point...");
  if (!move_to_goal(kCartFrame)) {
    RCLCPP_WARN(this->get_logger(), "Moving towards the center point failed.");
    return;
  }

  // Move 30cm more

  RCLCPP_INFO(this->get_logger(), "Moving 30cm more...");

  cart_to_goal.translation()[0] = 0.3;
  if (!move_to_goal(kCartFrame, cart_to_goal)) {
    RCLCPP_WARN(this->get_logger(), "The 30cm forward movement failed.");
    return;
  }

  // Load shelf.

  RCLCPP_INFO(this->get_logger(), "Attaching shelf...");
  elevator_up_publisher_->publish(Empty{});

end:
  RCLCPP_INFO(this->get_logger(), "Done.");
  res->complete = true;
}

std::optional<Eigen::Isometry3d>
ApproachServiceServer::getTransform(const std::string &source_frame,
                                    const std::string &dest_frame) {
  geometry_msgs::msg::TransformStamped t{};
  try {
    t = tf_buffer_->lookupTransform(source_frame, dest_frame,
                                    tf2::TimePointZero);
  } catch (const tf2::TransformException &ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
                source_frame.c_str(), dest_frame.c_str(), ex.what());
    return std::nullopt;
  }
  return tf2::transformToEigen(t);
}

bool ApproachServiceServer::move_to_goal(
    const std::string &dest_frame, const Eigen::Isometry3d &dest_to_goal) {

  const auto t_start{std::chrono::steady_clock::now()};
  std::optional<Eigen::Isometry3d> footprint_to_dest{};
  Eigen::Isometry3d footprint_to_goal{};
  double error_distance{};
  double error_yaw{};

  while (true) {
    footprint_to_dest = getTransform(kBaseFootprintFrame, dest_frame);
    if (!footprint_to_dest) {
      return false;
    }

    footprint_to_goal = footprint_to_dest.value() * dest_to_goal;

    error_distance = std::sqrt(std::pow(footprint_to_goal.translation()[0], 2) +
                               std::pow(footprint_to_goal.translation()[1], 2));
    error_yaw = std::atan2(footprint_to_goal.translation()[1],
                           footprint_to_goal.translation()[0]);

    Twist msg{};
    if (error_distance < kGoalPosTol) {
      publisher_->publish(msg);
      return true;
    }
    if (std::chrono::steady_clock::now() - t_start > kMotionTimeout) {
      publisher_->publish(msg);
      return false;
    }
    msg.angular.z = kKpYaw * error_yaw;
    msg.linear.x = kKpDdistance * error_distance;
    publisher_->publish(msg);
    std::this_thread::sleep_for(kTimerPeriod);
  }
}

std::optional<Eigen::Isometry3d> ApproachServiceServer::compCenter() {
  double center_x{};
  double center_y{};
  int num_points{0};
  double angle_min{};
  double angle_max{};
  double angle{};

  for (size_t i{0}; i < scan_.value().ranges.size(); ++i) {
    // the average of two averages is equal to the average of
    // all points (because of the linearity of the average)
    if (scan_.value().range_min <= scan_.value().ranges[i] &&
        scan_.value().ranges[i] <= scan_.value().range_max &&
        scan_.value().intensities[i] >= kIntensityThreshold) {
      angle = scan_.value().angle_min + i * scan_.value().angle_increment;
      center_x += scan_.value().ranges[i] * std::cos(angle);
      center_y += scan_.value().ranges[i] * std::sin(angle);
      // the angles are strictly increasing with i
      if (!num_points) {
        angle_min = angle;
      }
      angle_max = angle;
      ++num_points;
    }
  }

  if (num_points && angle_max - angle_min >= kMinLegsAngularDistance) {
    Eigen::Isometry3d to_center = Eigen::Isometry3d::Identity();
    to_center.translation() =
        Eigen::Vector3d{center_x / num_points, center_y / num_points, 0};
    return to_center;
  } else {
    return std::nullopt;
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node{std::make_shared<ApproachServiceServer>()};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
}