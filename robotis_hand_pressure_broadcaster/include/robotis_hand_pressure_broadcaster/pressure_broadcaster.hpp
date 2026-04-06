#ifndef ROBOTIS_HAND_PRESSURE_BROADCASTER__PRESSURE_BROADCASTER_HPP_
#define ROBOTIS_HAND_PRESSURE_BROADCASTER__PRESSURE_BROADCASTER_HPP_

#include <memory>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace robotis_hand_pressure_broadcaster
{

class PressureBroadcaster : public controller_interface::ControllerInterface
{
public:
  using CallbackReturn =
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  PressureBroadcaster() = default;

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  CallbackReturn on_init() override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  bool refresh_parameters();
  void configure_message_layout();

  std::vector<std::string> sensor_names_;
  std::vector<std::string> interface_names_;
  std::string topic_name_;

  std::shared_ptr<realtime_tools::RealtimePublisher<std_msgs::msg::Float64MultiArray>>
    publisher_;
};

}  // namespace robotis_hand_pressure_broadcaster

#endif  // ROBOTIS_HAND_PRESSURE_BROADCASTER__PRESSURE_BROADCASTER_HPP_
