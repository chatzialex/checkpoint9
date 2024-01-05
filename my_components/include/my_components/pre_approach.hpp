#ifndef MY_COMPONENTS__PRE_APPROACH_HPP_
#define MY_COMPONENTS__PRE_APPROACH_HPP_

#include "my_components/visibility_control.h"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"

namespace my_components {

using namespace std::chrono_literals;

enum class MotionState { Premotion, MovingForward, Rotating, Finished };

class PreApproach : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC explicit PreApproach(const rclcpp::NodeOptions &options);

private:
  static constexpr char kNodeName[]{"pre_approach"};
  static constexpr char kScanTopicName[]{"/scan"};
  static constexpr char kVelCmdTopicName[]{
      "diffbot_base_controller/cmd_vel_unstamped"};
  static constexpr char kOdomTopicName[]{"/diffbot_base_controller/odom"};

  static constexpr auto kTimerPeriod{100ms};

  static constexpr double kForwardVel{0.2}; // [m/s]
  static constexpr double kAngularVel{0.5}; // [rad/s]

  static constexpr double kPi{3.1416};
  static constexpr double kDegreesToRad{kPi / 180};

  void scan_cb(const std::shared_ptr<const sensor_msgs::msg::LaserScan> msg);
  void odom_cb(const std::shared_ptr<const nav_msgs::msg::Odometry> msg);
  void timer_cb();

  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr
      scan_subscription_{};
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_{};
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_{};
  rclcpp::TimerBase::SharedPtr timer_{};

  double dist_to_obstacle_current_{};  // [m]
  double dist_to_obstacle_final_{0.3}; // [m]
  double rotation_final_{-1.5708};     // [rad]
  double rotation_last_{};             // [rad]
  double rotation_total_{};            // [rad]
  double rotation_current_{0.0};       // [rad]
  MotionState motion_state_{MotionState::Premotion};
  bool received_odom_{false};
  bool received_scan_{false};
};

} // namespace my_components

#endif // MY_COMPONENTS__PRE_APPROACH_HPP_