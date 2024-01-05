#ifndef MY_COMPONENTS__ATTACH_CLIENT_HPP_
#define MY_COMPONENTS__ATTACH_CLIENT_HPP_

#include "my_components/visibility_control.h"

#include "my_components/srv/go_to_loading.hpp"

#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

namespace my_components {

using namespace std::chrono_literals;

class AttachClient : public rclcpp::Node {
public:
  COMPOSITION_PUBLIC explicit AttachClient(const rclcpp::NodeOptions &options);

private:
  constexpr static char kNodeName[]{"attach_client"};
  constexpr static char kServiceName[]{"approach_shelf"};
  constexpr static auto kTimerPeriod{1s};
  constexpr static auto kServiceWaitTime{1s};
  constexpr static auto kFutureWaitTime{1s};

  void timer_cb();

  rclcpp::Client<my_components::srv::GoToLoading>::SharedPtr client_{};
  rclcpp::TimerBase::SharedPtr timer_{};
};

} // namespace my_components

#endif // MY_COMPONENTS__ATTACH_CLIENT_HPP_