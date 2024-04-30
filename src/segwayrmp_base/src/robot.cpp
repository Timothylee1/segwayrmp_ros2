#include "segwayrmp_base/robot.hpp"
// #include "Ge_encoder_odometry.hpp"

namespace robot {

Chassis::Chassis(const rclcpp::Node::SharedPtr &nh) : Node("chassis_node") {
  velocity_subscription_ = nh->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&Chassis::cmd_vel_callback, this, std::placeholders::_1));
}

void Chassis::cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
  // Extract linear and angular velocities
  double angular_vel = msg->angular.z;
  double linear_vel = msg->linear.x;

  // Call your set_cmd_vel function to configure the chassis (implementation
  // assumed)
  set_cmd_vel(linear_vel, angular_vel);

  RCLCPP_INFO(this->get_logger(),
              "Received Twist message: linear.x=%f, angular.z=%f", linear_vel,
              angular_vel);
}

} // namespace robot
