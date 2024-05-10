#include "segwayrmp_base/segwayrmp_base_ros.hpp"

#define imu_angular_vel_ 0.0009288
#define imu_linear_vel_ 0.0023943
#define rad_to_deg_ 57.2958
chassis_motors_speed_data_t motorsSpeedData;
chassis_car_speed_data_t carSpeedData;
uint64_t Speed_TimeStamp;
uint8_t Speed_update;

front_motors_ticks_t frontTicksData;
rear_motors_ticks_t rearTicksData;
uint64_t Ticks_TimeStamp;
uint8_t Ticks_update;

imu_gyr_original_data_t ImuGyrData;
uint64_t ImuGyr_TimeStamp;
uint8_t ImuGyr_update;

imu_acc_original_data_t ImuAccData;
uint64_t ImuAcc_TimeStamp;
uint8_t ImuAcc_update;

odom_pos_xy_t OdomPoseXy;
odom_euler_xy_t OdomEulerXy;
odom_euler_z_t OdomEulerZ;
odom_vel_line_xy_t OdomVelLineXy;
uint64_t Odom_TimeStamp;
uint8_t Odom_update;

float FrontAxleAngle;
uint64_t FrontAxle_TimeStamp;
uint8_t FrontAxle_update;

namespace westonrobot {

Segwayrmp::Segwayrmp(std::string node_name)
    : Node(node_name), keep_running_(false) {

  timestamp_data.on_new_data = PublishOdomImuData;
  aprctrl_datastamped_jni_register(&timestamp_data);

  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&Segwayrmp::CommandVelocityCallback, this,
                std::placeholders::_1));
  batt_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  /* yet to do*/
  odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
}

bool Segwayrmp::Initialize(void) {
  set_comu_interface(comu_can); // Set communication via can0
  if (init_control_ctrl()) {
    printf("Initialization failed\n");
    return false;
  }
  enable_rotate_switch(1); // Enable the Segwayrmp to rotate in place
  return true;
}

void Segwayrmp::Stop(void) { keep_running_ = false; }

void Segwayrmp::Run(void) {
  rclcpp::Rate rate(3);
  keep_running_ = true;
  while (keep_running_) {
    PublishBatteryState();
    PublishImuState(); // Increase publishing rate
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}

void Segwayrmp::PublishBatteryState(void) {
  sensor_msgs::msg::BatteryState batt_msg;
  batt_msg.present = get_bat_soc() ? true : false;
  batt_msg.percentage = static_cast<float>(get_bat_soc());
  batt_msg.temperature = static_cast<float>(get_bat_temp());
  batt_msg.voltage = static_cast<float>(get_bat_mvol()) / 1000;

  batt_pub_->publish(batt_msg);
}

void Segwayrmp::PublishImuState(void) {
  uint64_t imu_stamp =
      ImuGyr_TimeStamp > ImuAcc_TimeStamp ? ImuGyr_TimeStamp : ImuAcc_TimeStamp;
  ros_imu_.header.stamp = TimeStamp(imu_stamp);
  ros_imu_.header.frame_id = "robot_imu";
  ros_imu_.angular_velocity.x =
      (double)ImuGyrData.gyr[0] / 900.0; //* imu_angular_vel_;
  ros_imu_.angular_velocity.y =
      (double)ImuGyrData.gyr[1] / 900.0; //* imu_angular_vel_;
  ros_imu_.angular_velocity.z =
      (double)ImuGyrData.gyr[2] / 900.0; //* imu_angular_vel_;
  ros_imu_.linear_acceleration.x =
      (double)ImuAccData.acc[0] / 4000.0 * 9.8; // * imu_linear_vel_;
  ros_imu_.linear_acceleration.y =
      (double)ImuAccData.acc[1] / 4000.0 * 9.8; // * imu_linear_vel_;
  ros_imu_.linear_acceleration.z =
      (double)ImuAccData.acc[2] / 4000.0 * 9.8; // * imu_linear_vel_;
  imu_pub_->publish(ros_imu_);
}

rclcpp::Time Segwayrmp::TimeStamp(int64_t timestamp) {
  uint32_t sec_ = timestamp / 1000000;
  uint32_t nsec_ = (timestamp % 1000000) * 1000;
  return rclcpp::Time(sec_, nsec_);
}

void Segwayrmp::PublishOdomImuData(StampedBasicFrame *frame) {
  if (frame->type_id == Chassis_Data_Motors_Speed) {
    memcpy(&motorsSpeedData, frame->data,
           sizeof(motorsSpeedData));
    Speed_TimeStamp = frame->timestamp;
    Speed_update |= 1;
  } else if (frame->type_id == Chassis_Data_Car_Speed) {
    memcpy(&carSpeedData, frame->data,
           sizeof(carSpeedData));
    Speed_TimeStamp = frame->timestamp;
    Speed_update |= 2;
  } else if (frame->type_id == Chassis_Data_Front_Ticks) {
    memcpy(&frontTicksData, frame->data,
           sizeof(frontTicksData));
    Ticks_TimeStamp = frame->timestamp;
    Ticks_update |= 1;
  } else if (frame->type_id == Chassis_Data_Rear_Ticks) {
    memcpy(&rearTicksData, frame->data,
           sizeof(rearTicksData));
    Ticks_TimeStamp = frame->timestamp;
    Ticks_update |= 2;
  } else if (frame->type_id == Chassis_Data_Imu_Gyr) {
    memcpy(&ImuGyrData, frame->data,
           sizeof(ImuGyrData));
    ImuGyr_TimeStamp = frame->timestamp;
    ImuGyr_update = 1;
  } else if (frame->type_id == Chassis_Data_Imu_Acc) {
    memcpy(&ImuAccData, frame->data,
           sizeof(ImuAccData));
    ImuAcc_TimeStamp = frame->timestamp;
    ImuAcc_update = 1;
  } else if (frame->type_id == Chassis_Data_Odom_Pose_xy) {
    memcpy(&OdomPoseXy, frame->data,
           sizeof(OdomPoseXy));
    Odom_TimeStamp = frame->timestamp;
    Odom_update |= 1;
    // ROS_INFO("odomPosX:%f, odomPosY:%f", OdomPoseXy.pos_x, OdomPoseXy.pos_y);
  } else if (frame->type_id == Chassis_Data_Odom_Euler_xy) {
    memcpy(&OdomEulerXy, frame->data,
           sizeof(OdomEulerXy));
    Odom_TimeStamp = frame->timestamp;
    Odom_update |= 2;
  } else if (frame->type_id == Chassis_Data_Odom_Euler_z) {
    memcpy(&OdomEulerZ, frame->data,
           sizeof(OdomEulerZ));
    Odom_TimeStamp = frame->timestamp;
    Odom_update |= 4;
  } else if (frame->type_id == Chassis_Data_Odom_Linevel_xy) {
    memcpy(&OdomVelLineXy, frame->data,
           sizeof(OdomVelLineXy));
    Odom_TimeStamp = frame->timestamp;
    Odom_update |= 8;
  } else if (frame->type_id == Chassis_Data_Front_Encoder_Angle) {
    memcpy(&FrontAxleAngle, frame->data, sizeof(FrontAxleAngle));
    FrontAxle_TimeStamp = frame->timestamp;
    FrontAxle_update = 1;
  }
}

/* Callbacks */
void Segwayrmp::CommandVelocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  double angular_vel = msg->angular.z;
  double linear_vel = msg->linear.x;
  RCLCPP_DEBUG(this->get_logger(),
               "Received Twist message: linear.x= %f, angular.z= %f",
               linear_vel, angular_vel);

  // Enables control of the system via /cmd_vel
  set_enable_ctrl(1);
  set_cmd_vel(linear_vel, angular_vel);
}

} // namespace westonrobot
