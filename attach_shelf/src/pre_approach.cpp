#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <cmath>
#include <iostream>
#include <memory>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace std::chrono_literals;

enum class MotionState { Premotion, MovingForward, Rotating, Finished };

class PreApproach : public rclcpp::Node {
public:
  PreApproach();

private:
  static constexpr char kNodeName[]{"pre_approach"};
  static constexpr char kScanTopicName[]{"/scan"};
  static constexpr char kVelCmdTopicName[]{"robot/cmd_vel"};
  static constexpr char kOdomTopicName[]{"/odom"};

  static constexpr char kObstacleParamName[]{"obstacle"};
  static constexpr char kDegreesParamName[]{"degrees"};

  static constexpr auto kTimerPeriod{100ms};

  static constexpr double kForwardVel{0.2}; // [m/s]
  static constexpr double kAngularVel{0.5}; // [rad/s]

  static constexpr double kPi{3.1416};
  static constexpr double kDegreesToRad{kPi / 180};

  void scan_cb(const std::shared_ptr<const sensor_msgs::msg::LaserScan> msg);
  void odom_cb(const std::shared_ptr<const nav_msgs::msg::Odometry> msg);
  bool get_params();
  void declare_params();
  void timer_cb();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{};
  rclcpp::TimerBase::SharedPtr timer_{};

  double dist_to_obstacle_current_{}; // [m]
  double dist_to_obstacle_final_{};   // [m]
  double rotation_final_{};           // [rad]
  double rotation_last_{};            // [rad]
  double rotation_total_{};           // [rad]
  double rotation_current_{0.0};      // [rad]
  MotionState motion_state_{MotionState::Premotion};
  bool received_odom_{false};
  bool received_scan_{false};
};

PreApproach::PreApproach()
    : Node{kNodeName},
      scan_subscription_{this->create_subscription<sensor_msgs::msg::LaserScan>(
          kScanTopicName, 1,
          [this](const std::shared_ptr<const sensor_msgs::msg::LaserScan> msg) {
            return scan_cb(msg);
          })},
      odom_subscription_{this->create_subscription<nav_msgs::msg::Odometry>(
          kOdomTopicName, 1,
          [this](const std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
            return odom_cb(msg);
          })},
      twist_publisher_{this->create_publisher<geometry_msgs::msg::Twist>(
          kVelCmdTopicName, 1)},
      timer_{this->create_wall_timer(kTimerPeriod,
                                     [this]() { return timer_cb(); })} {
  declare_params();
}

void PreApproach::scan_cb(
    const std::shared_ptr<const sensor_msgs::msg::LaserScan> msg) {
  const auto dist_front{msg->ranges[msg->ranges.size() / 2]};
  const auto range_min{msg->range_min};
  const auto range_max{msg->range_max};

  if (dist_front < range_min || dist_front > range_max) {
    RCLCPP_WARN(this->get_logger(),
                "Ignoring out-of-range laser measurement: %f (range_min:%f, "
                "range_max:%f)",
                dist_front, range_min, range_max);
  };

  dist_to_obstacle_current_ = dist_front;
  received_scan_ = true;
}

void PreApproach::odom_cb(
    const std::shared_ptr<const nav_msgs::msg::Odometry> msg) {
  tf2::Quaternion q{msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w};

  double pitch{};
  double roll{};
  double yaw{};

  tf2::Matrix3x3(q).getEulerYPR(yaw, pitch, roll);

  received_odom_ = true;
  rotation_current_ = yaw;
}

bool PreApproach::get_params() {
  double dist_to_obstacle_final{};
  int rotation_final_degrees{};
  this->get_parameter(kObstacleParamName, dist_to_obstacle_final);
  this->get_parameter(kDegreesParamName, rotation_final_degrees);

  if (dist_to_obstacle_final <= 0) {
    RCLCPP_WARN_ONCE(this->get_logger(),
                     "Parameter %s must be positive, got: %f",
                     kObstacleParamName, dist_to_obstacle_final);
    return false;
  }

  dist_to_obstacle_final_ = dist_to_obstacle_final;
  rotation_final_ = rotation_final_degrees * kDegreesToRad;
  return true;
}

void PreApproach::declare_params() {
  rcl_interfaces::msg::ParameterDescriptor obstacle_param_description{};
  obstacle_param_description.description =
      "Distance from the obstacle (in m) to stop the robot.";
  this->declare_parameter<std::double_t>(kObstacleParamName, 0.5,
                                         obstacle_param_description);

  rcl_interfaces::msg::ParameterDescriptor degrees_param_description{};
  degrees_param_description.description =
      "Degrees to rotate the robot after stopping.";
  this->declare_parameter<int>(kDegreesParamName, 0.0,
                               degrees_param_description);
}

void PreApproach::timer_cb() {
  geometry_msgs::msg::Twist twist{};

  switch (motion_state_) {
  case MotionState::Premotion:
    RCLCPP_INFO_ONCE(this->get_logger(), "Preparing for motion.");
    if (!received_odom_) {
      RCLCPP_WARN_ONCE(this->get_logger(), "Odometry not available.");
      break;
    }
    if (!received_scan_) {
      RCLCPP_WARN(this->get_logger(), "Laser scan info not available.");
      break;
    }
    if (!get_params()) {
      RCLCPP_ERROR(this->get_logger(), "Received invalid parameter values.");
      break;
    }
    motion_state_ = MotionState::MovingForward;
    [[fallthrough]];
  case MotionState::MovingForward:
    RCLCPP_INFO_ONCE(this->get_logger(), "Heading towards wall...");
    if (dist_to_obstacle_current_ > dist_to_obstacle_final_) {
      twist.linear.x = kForwardVel;
      break;
    }
    rotation_total_ = 0.0;
    rotation_last_ = rotation_current_;
    motion_state_ = MotionState::Rotating;
    [[fallthrough]];
  case MotionState::Rotating: {
    RCLCPP_INFO_ONCE(this->get_logger(), "Initiating %f degrees rotation.",
                     rotation_final_ / kDegreesToRad);
    const int sign{rotation_final_ >= 0 ? 1 : -1};
    rotation_total_ += std::atan2(std::sin(rotation_current_ - rotation_last_),
                                  std::cos(rotation_current_ - rotation_last_));
    if (sign * (rotation_total_ - rotation_final_) < 0) {
      twist.angular.z = sign * kAngularVel;
      rotation_last_ = rotation_current_;
      break;
    }
    motion_state_ = MotionState::Finished;
    [[fallthrough]];
  }
  case MotionState::Finished:
    RCLCPP_INFO_ONCE(this->get_logger(), "Motion finished.");
    timer_->cancel();
  };

  twist_publisher_->publish(twist);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PreApproach>());

  rclcpp::shutdown();
}