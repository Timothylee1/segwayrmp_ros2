#include "rclcpp/rclcpp.hpp"
#include "segwayrmp_base/segwayrmp_base_ros.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("segwayrmp_base_node");
  auto node_params =
      node->declare_parameter<std::string>("segwayrmp_base_node", "ttyUSB0");

  if (node_params != "ttyUSB0") {
    printf("segwayrmp_base_node: %s\n", node_params.c_str());
    set_smart_car_serial(node_params.c_str());
  }

  // Before calling init_control_ctrl, need to call this function to set whether
  // the communication port is a serial port or a CAN port,
  // "comu)serial":serial; "comu_can":CAN.;Others: Illegal
  set_comu_interface(comu_can);
  if (init_control_ctrl() == -1) {
    printf("init_control failed\n");
  } else {
    printf("init success!\n");
  }

  set_enable_ctrl(1);
  westonrobot::Chassis sbv(node);

  enable_rotate_switch(1);
  // sbv.Run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
