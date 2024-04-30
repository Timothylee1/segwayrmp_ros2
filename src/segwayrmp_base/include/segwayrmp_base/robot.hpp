#ifndef _ROBOT_HPP
#define _ROBOT_HPP

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

// #include "Ge_encoder_odometry.hpp"
#include "comm_ctrl_navigation.hpp"

#define IAP_STATE_COMPLETE 3
#define IAP_STATE_FAIL 4
#define IAP_STATE_ABORT 5

namespace robot {

class Chassis : public rclcpp::Node {
public:
  Chassis(const rclcpp::Node::SharedPtr &nh);

private:
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

  rclcpp::Node::SharedPtr nh_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr
      velocity_subscription_;
};

} // namespace robot

#endif
