#ifndef SEGWAYRMP_BASE_ROS_HPP
#define SEGWAYRMP_BASE_ROS_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "comm_ctrl_navigation.hpp"
#include "odometry.hpp"

namespace westonrobot {

class Segwayrmp : public rclcpp::Node {
public:
  Segwayrmp(std::string node_name);

  bool Initialize(void);
  void Run(void);
  void Stop(void);
  void PublishBatteryState(void);

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;

  tf2::Quaternion quat_;
  sensor_msgs::msg::Imu ros_imu_;
  nav_msgs::msg::Odometry ros_odom_;
  geometry_msgs::msg::Quaternion odom_quat_;

  rclcpp::Time Timestamp(int64_t timestamp);

  bool ReadParameters(void);
  double GetOrientationX(void);
  double GetOrientationY(void);
  double GetOrientationZ(void);
  double GetOrientationW(void);

  void PublishImuState(void);
  void PublishImuOdomState(void);
  static void OdomImuData(StampedBasicFrame *frame);
  void CommandVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  bool keep_running_;
  float rad_to_deg_ = 57.2958;

  s_aprctrl_datastamped_t timestamp_data_;
};

} // namespace westonrobot

#endif
