#ifndef SEGWAYRMP_BASE_ROS_HPP
#define SEGWAYRMP_BASE_ROS_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
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
  bool keep_running_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  // rclcpp::Publisher<tf2_ros::TransformBroadcaster>::SharedPtr
  // odom_broadcaster_;

  sensor_msgs::msg::Imu ros_imu_;
  nav_msgs::msg::Odometry ros_odom_;
  geometry_msgs::msg::TransformStamped odom_transform_;

  rclcpp::Time TimeStamp(int64_t timestamp);

  void PublishImuState(void);
  static void PublishOdomImuData(StampedBasicFrame *frame);
  // void PubOdomToRosOdom(Odometry odom_data);
  void CommandVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  s_aprctrl_datastamped_t timestamp_data;
  // odometry odometry_;
};

} // namespace westonrobot

#endif
