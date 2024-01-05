#include "my_components/attach_client.hpp"
#include <memory>

namespace my_components {

AttachClient::AttachClient(const rclcpp::NodeOptions &options)
    : Node{kNodeName, options},
      client_{
          this->create_client<my_components::srv::GoToLoading>(kServiceName)},
      timer_{this->create_wall_timer(kTimerPeriod,
                                     [this] { return timer_cb(); })} {}

void AttachClient::timer_cb() {
  while (!client_->wait_for_service(kServiceWaitTime)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
          this->get_logger(),
          "Client interrupted while waiting for service. Terminating...");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service Unavailable.");
    return;
  }

  timer_->cancel();

  auto request{std::make_shared<my_components::srv::GoToLoading::Request>()};
  request->attach_to_shelf = true;

  auto response_received_callback =
      [this](rclcpp::Client<my_components::srv::GoToLoading>::SharedFuture
                 future) {
        auto status = future.wait_for(1s);
        if (status == std::future_status::ready) {
          if (future.get()->complete) {
            RCLCPP_INFO(this->get_logger(), "%s succeeded.", kServiceName);
          } else {
            RCLCPP_INFO(this->get_logger(), "%s failed.", kServiceName);
          }
        } else {
          RCLCPP_INFO(this->get_logger(), "%s in progress...", kServiceName);
        }
      };
  auto future_result =
      client_->async_send_request(request, response_received_callback);
}

} // namespace my_components

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(my_components::AttachClient)