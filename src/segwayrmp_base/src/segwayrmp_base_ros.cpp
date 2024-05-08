#include "segwayrmp_base/segwayrmp_base_ros.hpp"

namespace westonrobot {

Chassis::Chassis(const rclcpp::Node::SharedPtr &nh_)
    : Node("chassis_node"), keep_running_(false) {
  cmd_vel_sub_ = nh_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&Chassis::CommandVelocityCallback, this,
                std::placeholders::_1));

  // enable_chassis_rotation_srv =
  //     nh_->create_service<segwayrmp_msg::srv::EnableChassisRotation>(
  //         "enable_chassis_rotation_srv",
  //         Chassis::ChassisRotationCallback);
}

bool Chassis::Initialize(void) {
  // initialize segwayrmp
  // ros messenger
  // feedback
  return true;
}

void Chassis::Stop(void) { keep_running_ = false; }

void Chassis::Run(void) {
  keep_running_ = true;
  // allows segwayrmp to turn left and right on the stop
  enable_rotate_switch(1);
  // timer for control
  while (keep_running_) {
    // might not be here
    set_enable_ctrl(1); // allows segwayrmp to be controlled via /cmd_vel
  }
}

/* Callbacks */
void Chassis::CommandVelocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  double angular_vel = msg->angular.z;
  double linear_vel = msg->linear.x;
  RCLCPP_DEBUG(this->get_logger(),
               "Received Twist message: linear.x= %f, angular.z= %f",
               linear_vel, angular_vel);

  set_cmd_vel(linear_vel, angular_vel);
}

bool Chassis::ChassisRotationCallback(
    segwayrmp_msg::srv::EnableChassisRotation::Request const &req,
    segwayrmp_msg::srv::EnableChassisRotation::Response &res) {
  if (req.enable_chassis_rotation == false) {
    res.enable_chassis_rotation_result = enable_rotate_switch(0);
    RCLCPP_INFO(this->get_logger(),
                "enable_chassis_rotation_result disable: %d",
                res.enable_chassis_rotation_result);
  } else if (req.enable_chassis_rotation == true) {
    res.enable_chassis_rotation_result = enable_rotate_switch(1);
    RCLCPP_INFO(this->get_logger(), "enable_chassis_rotation_result enable: %d",
                res.enable_chassis_rotation_result);
  } else {
    return false;
  }

  return true;
}

} // namespace westonrobot
