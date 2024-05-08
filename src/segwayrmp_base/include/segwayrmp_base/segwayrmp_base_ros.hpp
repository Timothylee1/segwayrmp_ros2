#ifndef _ROBOT_HPP
#define _ROBOT_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include "segwayrmp_msg/srv/enable_chassis_rotation.hpp"
#include "segwayrmp_msg/srv/set_chassis_control.hpp"
// #include "odometry.hpp"
#include "comm_ctrl_navigation.hpp"

#define IAP_STATE_COMPLETE 3
#define IAP_STATE_FAIL 4
#define IAP_STATE_ABORT 5

namespace westonrobot {

class Chassis : public rclcpp::Node {
public:
  Chassis(const rclcpp::Node::SharedPtr &nh_);

  bool Initialize(void);
  void Run(void);
  void Stop(void);

private:
  void CommandVelocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;

  bool ChassisRotationCallback(
      segwayrmp_msg::srv::EnableChassisRotation::Request const &req,
      segwayrmp_msg::srv::EnableChassisRotation::Response &res);

  rclcpp::Service<segwayrmp_msg::srv::EnableChassisRotation>::SharedPtr
      enable_chassis_rotation_srv;

  bool keep_running_;
};

} // namespace westonrobot

#endif
