#include "rclcpp/rclcpp.hpp"
#include "segwayrmp_base/segwayrmp_base_ros.hpp"

std::shared_ptr<westonrobot::Segwayrmp> robot;

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  robot = std::make_shared<westonrobot::Segwayrmp>("segwayrmp_base_node");

  if (robot->Initialize()) {
    printf("Segwayrmp initialized, start running...\n");
    robot->Run();
  }

  return 0;
}
