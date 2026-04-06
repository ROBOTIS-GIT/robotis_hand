#include "robotis_hand_pressure_broadcaster/pressure_broadcaster.hpp"

#include <algorithm>
#include <string_view>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace robotis_hand_pressure_broadcaster
{

namespace
{

std::string infer_hand_name(const std::vector<std::string> & sensor_names)
{
  for (const auto & sensor_name : sensor_names) {
    if (sensor_name.find("finger_l_") != std::string::npos) {
      return "left";
    }

    if (sensor_name.find("finger_r_") != std::string::npos) {
      return "right";
    }
  }

  return "";
}

}  // namespace

controller_interface::InterfaceConfiguration
PressureBroadcaster::command_interface_configuration() const
{
  return {controller_interface::interface_configuration_type::NONE, {}};
}

controller_interface::InterfaceConfiguration
PressureBroadcaster::state_interface_configuration() const
{
  const_cast<PressureBroadcaster *>(this)->refresh_parameters();

  std::vector<std::string> requested_interfaces;
  requested_interfaces.reserve(sensor_names_.size() * interface_names_.size());

  for (const auto & sensor_name : sensor_names_) {
    for (const auto & interface_name : interface_names_) {
      requested_interfaces.emplace_back(sensor_name + "/" + interface_name);
    }
  }

  return {controller_interface::interface_configuration_type::INDIVIDUAL, requested_interfaces};
}

PressureBroadcaster::CallbackReturn PressureBroadcaster::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("sensor_names", {});
    auto_declare<std::vector<std::string>>("interface_names", {});
    auto_declare<std::string>("hand_name", "");
    auto_declare<std::string>("frame_id", "");
    auto_declare<std::string>("topic_name", "~/pressures");
  } catch (const std::exception & exception) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to declare parameters: %s", exception.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

PressureBroadcaster::CallbackReturn PressureBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!refresh_parameters() || sensor_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'sensor_names' parameter must not be empty");
    return CallbackReturn::ERROR;
  }

  if (interface_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "'interface_names' parameter must not be empty");
    return CallbackReturn::ERROR;
  }

  auto publisher = get_node()->create_publisher<robotis_interfaces::msg::HandPressures>(topic_name_, 10);
  publisher_ =
    std::make_shared<realtime_tools::RealtimePublisher<robotis_interfaces::msg::HandPressures>>(publisher);

  configure_message();

  return CallbackReturn::SUCCESS;
}

PressureBroadcaster::CallbackReturn PressureBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!publisher_) {
    RCLCPP_ERROR(get_node()->get_logger(), "Publisher is not configured");
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

PressureBroadcaster::CallbackReturn PressureBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return CallbackReturn::SUCCESS;
}

controller_interface::return_type PressureBroadcaster::update(
  const rclcpp::Time & time,
  const rclcpp::Duration & /*period*/)
{
  if (!publisher_ || !publisher_->trylock()) {
    return controller_interface::return_type::OK;
  }

  auto & message = publisher_->msg_;
  std::transform(
    state_interfaces_.cbegin(),
    state_interfaces_.cend(),
    message.pressures.begin(),
    [](const auto & state_interface) {
      return state_interface.get_optional().value_or(0.0);
    });
  const auto time_ns = time.nanoseconds();
  message.header.stamp.sec = static_cast<int32_t>(time_ns / 1000000000LL);
  message.header.stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000LL);

  publisher_->unlockAndPublish();
  return controller_interface::return_type::OK;
}

void PressureBroadcaster::configure_message()
{
  auto & message = publisher_->msg_;
  message.header.frame_id = frame_id_;
  message.hand_name = hand_name_;
  message.sensor_names = sensor_names_;
  message.channel_names = interface_names_;
  message.pressures.assign(sensor_names_.size() * interface_names_.size(), 0.0);
}

bool PressureBroadcaster::refresh_parameters()
{
  if (!get_node()->has_parameter("sensor_names") ||
    !get_node()->has_parameter("interface_names") ||
    !get_node()->has_parameter("hand_name") ||
    !get_node()->has_parameter("frame_id") ||
    !get_node()->has_parameter("topic_name"))
  {
    return false;
  }

  sensor_names_ = get_node()->get_parameter("sensor_names").as_string_array();
  interface_names_ = get_node()->get_parameter("interface_names").as_string_array();
  hand_name_ = get_node()->get_parameter("hand_name").as_string();
  frame_id_ = get_node()->get_parameter("frame_id").as_string();
  topic_name_ = get_node()->get_parameter("topic_name").as_string();

  if (hand_name_.empty()) {
    hand_name_ = infer_hand_name(sensor_names_);
  }

  return true;
}

}  // namespace robotis_hand_pressure_broadcaster

PLUGINLIB_EXPORT_CLASS(
  robotis_hand_pressure_broadcaster::PressureBroadcaster,
  controller_interface::ControllerInterface)
