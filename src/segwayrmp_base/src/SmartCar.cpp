#include "rclcpp/rclcpp.hpp"
#include "segwayrmp_base/robot.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("SmartCar");
  auto node_params =
      node->declare_parameter<std::string>("RMPSmartCarSerial", "ttyUSB0");

  if (node_params != "ttyUSB0") {
    printf("RMPSmartCarSerial: %s\n", node_params.c_str());
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

  robot::Chassis sbv(node);

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
