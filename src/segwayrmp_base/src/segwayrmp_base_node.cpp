#include "rclcpp/rclcpp.hpp"
#include "segwayrmp_base/segwayrmp_base_ros.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::Node::SharedPtr node = std::make_shared<rclcpp::Node>("segwayrmp_base_node");
  westonrobot::Segwayrmp segwayrmp(node.get());

  if (segwayrmp.Initialize()) {
    printf("Segwayrmp initialized, start running...\n");
    segwayrmp.Run();
  }


  return 0;
}
