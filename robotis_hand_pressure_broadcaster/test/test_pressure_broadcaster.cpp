#include <gtest/gtest.h>
#include <chrono>
#include <thread>
#include "robotis_hand_pressure_broadcaster/pressure_broadcaster.hpp"

using robotis_hand_pressure_broadcaster::PressureBroadcaster;
using Message = robotis_interfaces::msg::HandPressures;
using CallbackReturn = PressureBroadcaster::CallbackReturn;

class BroadcasterTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void configure(PressureBroadcaster & controller, int count)
  {
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "pressure_test";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    ASSERT_EQ(controller.init(params), controller_interface::return_type::OK);
    std::vector<std::string> names;
    for (int i = 1; i <= count; ++i) {
      names.push_back("Present Pressure " + std::to_string(i));
    }
    controller.get_node()->set_parameter(rclcpp::Parameter(
      "sensor_names", std::vector<std::string>{"finger_r_sensor1", "finger_r_sensor2"}));
    controller.get_node()->set_parameter(rclcpp::Parameter("interface_names", names));
    controller.get_node()->set_parameter(rclcpp::Parameter("topic_name", "/test_pressures"));
  }
};

TEST_F(BroadcasterTest, PublishesEveryChannelWithReversedHandles)
{
  for (int count : {9, 10}) {
    PressureBroadcaster controller;
    configure(controller, count);
    ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    const auto requested = controller.state_interface_configuration().names;
    ASSERT_EQ(requested.size(), static_cast<size_t>(count * 2));
    std::vector<double> values(count * 2);
    std::vector<hardware_interface::StateInterface> states;
    states.reserve(values.size());
    for (int i = 0; i < count * 2; ++i) {
      values[i] = i + 1;
      states.emplace_back("finger_r_sensor" + std::to_string(i / count + 1),
        "Present Pressure " + std::to_string(i % count + 1), &values[i]);
    }
    std::vector<hardware_interface::LoanedStateInterface> loans;
    for (auto it = states.rbegin(); it != states.rend(); ++it) {
      loans.emplace_back(*it);
    }
    controller.assign_interfaces({}, std::move(loans));
    ASSERT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
    auto node = std::make_shared<rclcpp::Node>("pressure_listener");
    Message::SharedPtr received;
    auto subscription = node->create_subscription<Message>("/test_pressures", 10,
        [&received](Message::SharedPtr msg) {received = msg;});
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    while (!received && std::chrono::steady_clock::now() < deadline) {
      controller.update(node->now(), rclcpp::Duration::from_seconds(0.01));
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    ASSERT_TRUE(received);
    ASSERT_EQ(received->sensors.size(), 2u);
    for (int f = 0; f < 2; ++f) {
      ASSERT_EQ(received->sensors[f].pressure_values.size(), static_cast<size_t>(count));
      ASSERT_EQ(received->sensors[f].pressure_names.size(), static_cast<size_t>(count));
      for (int t = 0; t < count; ++t) {
        EXPECT_EQ(received->sensors[f].pressure_values[t], f * count + t + 1);
        EXPECT_EQ(received->sensors[f].pressure_names[t],
          "Present Pressure " + std::to_string(t + 1));
      }
    }
    controller.release_interfaces();
  }
}

TEST_F(BroadcasterTest, RejectsInvalidConfigurationAndMissingHandles)
{
  PressureBroadcaster controller;
  configure(controller, 11);
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  std::vector<std::string> names;
  for (int i = 1; i <= 10; ++i) {
    names.push_back("Present Pressure " + std::to_string(i));
  }
  controller.get_node()->set_parameter(rclcpp::Parameter("interface_names", names));
  ASSERT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
  EXPECT_EQ(controller.on_activate(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
  std::swap(names[0], names[1]);
  controller.get_node()->set_parameter(rclcpp::Parameter("interface_names", names));
  EXPECT_EQ(controller.on_configure(rclcpp_lifecycle::State()), CallbackReturn::ERROR);
}
