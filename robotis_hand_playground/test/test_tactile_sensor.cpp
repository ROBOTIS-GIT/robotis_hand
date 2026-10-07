#include <gtest/gtest.h>
#include "robotis_hand_playground/tactile_sensor.hpp"

using namespace robotis_hand_playground;

static TactileSensor::HandPressuresPtr message(int count)
{
  auto msg = std::make_shared<TactileSensor::HandPressuresMsg>();
  for (int f = 1; f <= 5; ++f) {
    robotis_interfaces::msg::TactileSensor sensor;
    sensor.sensor_name = "finger_r_sensor" + std::to_string(f);
    for (int t = 1; t <= count; ++t) {
      sensor.pressure_names.push_back("Present Pressure " + std::to_string(t));
      sensor.pressure_values.push_back(20);
    }
    msg->sensors.push_back(sensor);
  }
  return msg;
}

TEST(TactileSensor, BothLayoutsPreserveLastChannelThroughBaselineAndFiltering)
{
  auto clock = std::make_shared<rclcpp::Clock>();
  for (int count : {9, 10}) {
    TactileSensor processor(rclcpp::get_logger("test"), clock, count);
    FingerArray fingers{};
    processor.initialize(fingers);
    bool baseline = false;
    auto msg = message(count);
    ASSERT_TRUE(processor.check_msg(msg));
    for (int sample = 0; sample < 30; ++sample) {
      processor.update_pressure(fingers, baseline, processor.parse_sensors(msg));
    }
    ASSERT_TRUE(baseline);
    EXPECT_DOUBLE_EQ(fingers[0].baseline_tactiles.back(), 20.0);
    msg->sensors[0].pressure_values.back() = 120;
    for (int sample = 0; sample < 100; ++sample) {
      processor.update_pressure(fingers, baseline, processor.parse_sensors(msg));
    }
    EXPECT_NEAR(fingers[0].filtered_force, 100.0, 0.001);
    EXPECT_DOUBLE_EQ(fingers[1].filtered_force, 0.0);
  }
}

TEST(TactileSensor, RejectsWrongLengthNamesOrderAndHand)
{
  auto clock = std::make_shared<rclcpp::Clock>();
  TactileSensor processor(rclcpp::get_logger("test"), clock, 10);
  EXPECT_FALSE(processor.check_msg(nullptr));
  EXPECT_FALSE(processor.check_msg(message(9)));
  auto msg = message(10);
  msg->sensors[0].pressure_names.pop_back();
  EXPECT_FALSE(processor.check_msg(msg));
  msg = message(10);
  msg->sensors[0].pressure_values.pop_back();
  EXPECT_FALSE(processor.check_msg(msg));
  msg = message(10);
  std::swap(msg->sensors[0].pressure_names[0], msg->sensors[0].pressure_names[1]);
  EXPECT_FALSE(processor.check_msg(msg));
  msg = message(10);
  msg->sensors[0].sensor_name = "finger_l_sensor1";
  EXPECT_FALSE(processor.check_msg(msg));
  msg = message(10);
  std::swap(msg->sensors[0], msg->sensors[1]);
  EXPECT_FALSE(processor.check_msg(msg));
  EXPECT_THROW(TactileSensor(rclcpp::get_logger("test"), clock, 11), std::invalid_argument);
}
