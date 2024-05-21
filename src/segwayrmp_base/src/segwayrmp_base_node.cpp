#include "rclcpp/rclcpp.hpp"
#include "segwayrmp_base/segwayrmp_base_ros.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto segwayrmp = std::make_shared<westonrobot::Segwayrmp>("segwayrmp_base_node");

  if (segwayrmp->Initialize()) {
    printf("Segwayrmp initialized, start running...\n");
    segwayrmp->Run();
  }
  
  return 0;
}
