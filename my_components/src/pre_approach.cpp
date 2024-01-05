#include "my_components/pre_approach.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <cmath>
#include <memory>

namespace my_components {

PreApproach::PreApproach(const rclcpp::NodeOptions &options)
    : Node{kNodeName, options},
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
                                     [this]() { return timer_cb(); })} {}

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

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::PreApproach)