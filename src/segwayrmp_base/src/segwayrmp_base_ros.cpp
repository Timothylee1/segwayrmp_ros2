#include "segwayrmp_base/segwayrmp_base_ros.hpp"
/* 
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
 */
imu_gyr_original_data_t ImuGyrData;
uint64_t ImuGyr_TimeStamp;
uint8_t ImuGyr_update;

imu_acc_original_data_t ImuAccData;
uint64_t ImuAcc_TimeStamp;
uint8_t ImuAcc_update;

/* odom_pos_xy_t OdomPoseXy;
odom_euler_xy_t OdomEulerXy;
odom_euler_z_t OdomEulerZ;
odom_vel_line_xy_t OdomVelLineXy;
uint64_t Odom_TimeStamp;
uint8_t Odom_update;

float FrontAxleAngle;
uint64_t FrontAxle_TimeStamp;
uint8_t FrontAxle_update; */

namespace westonrobot {

rclcpp::Time timestamp2rostime(int64_t timestamp) {
  uint32_t sec_ = timestamp / 1000000;
  uint32_t nsec_ = (timestamp % 1000000) * 1000;
  return rclcpp::Time(sec_, nsec_);
}

/*
static void PubData(StampedBasicFrame *frame) {
  if (frame->type_id == Chassis_Data_Motors_Speed) {
    memcpy(&motorsSpeedData, frame->data,
           sizeof(motorsSpeedData)); // Speed data from chassis
    Speed_TimeStamp = frame->timestamp;
    Speed_update |= 1;
  } else if (frame->type_id == Chassis_Data_Car_Speed) {
    memcpy(&carSpeedData, frame->data,
           sizeof(carSpeedData)); // Speed data from chassis
    Speed_TimeStamp = frame->timestamp;
    Speed_update |= 2;
  } else if (frame->type_id == Chassis_Data_Front_Ticks) {
    memcpy(&frontTicksData, frame->data,
           sizeof(frontTicksData)); // Ticks data from chassis
    Ticks_TimeStamp = frame->timestamp;
    Ticks_update |= 1;
    // ROS_INFO("fLTICKS:%d, frticks:%d", frontTicksData.fl_ticks,
    // frontTicksData.fr_ticks);
  } else if (frame->type_id == Chassis_Data_Rear_Ticks) {
    memcpy(&rearTicksData, frame->data,
           sizeof(rearTicksData)); // Ticks data from chassis
    Ticks_TimeStamp = frame->timestamp;
    Ticks_update |= 2;
    // ROS_INFO("rLTICKS:%d, rrticks:%d", rearTicksData.rl_ticks,
    // rearTicksData.rr_ticks);
  } else if (frame->type_id == Chassis_Data_Imu_Gyr) {
    memcpy(&ImuGyrData, frame->data,
           sizeof(ImuGyrData)); // Ticks data from chassis
    ImuGyr_TimeStamp = frame->timestamp;
    ImuGyr_update = 1;
    // ROS_INFO("GYR0:%d, gyr1:%d, gyr2:%d", ImuGyrData.gyr[0],
    // ImuGyrData.gyr[1], ImuGyrData.gyr[2]);
  } else if (frame->type_id == Chassis_Data_Imu_Acc) {
    memcpy(&ImuAccData, frame->data,
           sizeof(ImuAccData)); // Ticks data from chassis
    ImuAcc_TimeStamp = frame->timestamp;
    ImuAcc_update = 1;
    // ROS_INFO("ACC0:%d, acc1:%d, acc2:%d", ImuAccData.acc[0],
    // ImuAccData.acc[1], ImuAccData.acc[2]);
  } else if (frame->type_id == Chassis_Data_Odom_Pose_xy) {
    memcpy(&OdomPoseXy, frame->data,
           sizeof(OdomPoseXy)); // Ticks data from chassis
    Odom_TimeStamp = frame->timestamp;
    Odom_update |= 1;
    // ROS_INFO("odomPosX:%f, odomPosY:%f", OdomPoseXy.pos_x, OdomPoseXy.pos_y);
  } else if (frame->type_id == Chassis_Data_Odom_Euler_xy) {
    memcpy(&OdomEulerXy, frame->data,
           sizeof(OdomEulerXy)); // Ticks data from chassis
    Odom_TimeStamp = frame->timestamp;
    Odom_update |= 2;
    // ROS_INFO("OdomEulerX:%f, OdomEulerY:%f", OdomEulerXy.euler_x,
    // OdomEulerXy.euler_y);
  } else if (frame->type_id == Chassis_Data_Odom_Euler_z) {
    memcpy(&OdomEulerZ, frame->data,
           sizeof(OdomEulerZ)); // Ticks data from chassis
    Odom_TimeStamp = frame->timestamp;
    Odom_update |= 4;
    // ROS_INFO("OdomEulerZ:%f", OdomEulerZ.euler_z);
  } else if (frame->type_id == Chassis_Data_Odom_Linevel_xy) {
    memcpy(&OdomVelLineXy, frame->data,
           sizeof(OdomVelLineXy)); // Ticks data from chassis
    Odom_TimeStamp = frame->timestamp;
    Odom_update |= 8;
    // ROS_INFO("OdomVelLineX:%f, OdomVelLineY:%f", OdomVelLineXy.vel_line_x,
    // OdomVelLineXy.vel_line_y);
  } else if (frame->type_id == Chassis_Data_Front_Encoder_Angle) {
    memcpy(&FrontAxleAngle, frame->data, sizeof(FrontAxleAngle));
    FrontAxle_TimeStamp = frame->timestamp;
    FrontAxle_update = 1;
  }
}


double getOrientationX() {
  float x = OdomEulerXy.euler_x / rad_to_deg_ / 2.0;
  float y = OdomEulerXy.euler_y / rad_to_deg_ / 2.0;
  float z = OdomEulerZ.euler_z / rad_to_deg_ / 2.0;

  float fCosHRoll = cos(x);
  float fSinHRoll = sin(x);
  float fCosHPitch = cos(y);
  float fSinHPitch = sin(y);
  float fCosHYaw = cos(z);
  float fSinHYaw = sin(z);

  return (fSinHRoll * fCosHPitch * fCosHYaw -
          fCosHRoll * fSinHPitch * fSinHYaw);
}

double getOrientationY() {
  float x = OdomEulerXy.euler_x / rad_to_deg_ / 2.0;
  float y = OdomEulerXy.euler_y / rad_to_deg_ / 2.0;
  float z = OdomEulerZ.euler_z / rad_to_deg_ / 2.0;

  float fCosHRoll = cos(x);
  float fSinHRoll = sin(x);
  float fCosHPitch = cos(y);
  float fSinHPitch = sin(y);
  float fCosHYaw = cos(z);
  float fSinHYaw = sin(z);

  return (fCosHRoll * fSinHPitch * fCosHYaw +
          fSinHRoll * fCosHPitch * fSinHYaw);
}

double getOrientationZ() {
  float x = OdomEulerXy.euler_x / rad_to_deg_ / 2.0;
  float y = OdomEulerXy.euler_y / rad_to_deg_ / 2.0;
  float z = OdomEulerZ.euler_z / rad_to_deg_ / 2.0;

  float fCosHRoll = cos(x);
  float fSinHRoll = sin(x);
  float fCosHPitch = cos(y);
  float fSinHPitch = sin(y);
  float fCosHYaw = cos(z);
  float fSinHYaw = sin(z);

  return (fCosHRoll * fCosHPitch * fSinHYaw -
          fSinHRoll * fSinHPitch * fCosHYaw);
}

double getOrientationW() {
  float x = OdomEulerXy.euler_x / rad_to_deg_ / 2.0;
  float y = OdomEulerXy.euler_y / rad_to_deg_ / 2.0;
  float z = OdomEulerZ.euler_z / rad_to_deg_ / 2.0;

  float fCosHRoll = cos(x);
  float fSinHRoll = sin(x);
  float fCosHPitch = cos(y);
  float fSinHPitch = sin(y);
  float fCosHYaw = cos(z);
  float fSinHYaw = sin(z);

  return (fCosHRoll * fCosHPitch * fCosHYaw +
          fSinHRoll * fSinHPitch * fSinHYaw);
}

void Segwayrmp::PubOdomToRosOdom(Odometry odom_data) {
  ros_odom_.header.stamp = timestamp2rostime(odom_data.TimeStamp);
  ros_odom_.header.frame_id = "odom";
  ros_odom_.pose.pose.position.x = odom_data.pose_.x;
  ros_odom_.pose.pose.position.y = odom_data.pose_.y;
  ros_odom_.pose.pose.position.z = 0;
  ros_odom_.pose.pose.orientation.x = 0;
  ros_odom_.pose.pose.orientation.y = 0;
  ros_odom_.pose.pose.orientation.z = sin(odom_data.pose_.orientation / 2);
  ros_odom_.pose.pose.orientation.w = cos(odom_data.pose_.orientation / 2);
  ros_odom_.twist.twist.linear.x = odom_data.twist_.v_x;
  ros_odom_.twist.twist.linear.y = odom_data.twist_.v_y;
  ros_odom_.twist.twist.linear.z = 0;
  ros_odom_.twist.twist.angular.x = 0;
  ros_odom_.twist.twist.angular.y = 0;
  ros_odom_.twist.twist.angular.z = odom_data.twist_.w_z;
  odom_pub_->publish(ros_odom_);
} */

void Segwayrmp::PublishImuState(void) {
  uint64_t imu_stamp =
      ImuGyr_TimeStamp > ImuAcc_TimeStamp ? ImuGyr_TimeStamp : ImuAcc_TimeStamp;
  ros_imu_.header.stamp = timestamp2rostime(imu_stamp);
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
/*
void Segwayrmp::TimeUpdate(void) {

  if (ImuGyr_update == 1 && ImuAcc_update == 1) {

    PublishImuState();

    ImuGyr_update = 0;
    ImuAcc_update = 0;
  }

  static uint64_t time_pre = 0;
  if (Odom_update == 15) {
    Odom_update = 0;

    geometry_msgs::Quaternion odom_quat =
        tf::createQuaternionMsgFromYaw(OdomEulerZ.euler_z / rad_to_deg_);

    ROS_odom.header.stamp = timestamp2rostime(Odom_TimeStamp);
    ROS_odom.header.frame_id = "odom";
    ROS_odom.pose.pose.position.x = OdomPoseXy.pos_x;
    ROS_odom.pose.pose.position.y = OdomPoseXy.pos_y;
    ROS_odom.pose.pose.position.z = 0;
    ROS_odom.pose.pose.orientation.x = getOrientationX();
    ROS_odom.pose.pose.orientation.y = getOrientationY();
    ROS_odom.pose.pose.orientation.z = getOrientationZ();
    ROS_odom.pose.pose.orientation.w = getOrientationW();
    ROS_odom.child_frame_id = "base_link";
    ROS_odom.twist.twist.linear.x =
        (double)carSpeedData.car_speed / LINE_SPEED_TRANS_GAIN_MPS;
    ROS_odom.twist.twist.linear.y = 0;
    ROS_odom.twist.twist.linear.z = 0;
    ROS_odom.twist.twist.angular.x = (double)ImuGyrData.gyr[0] / 900.0;
    ROS_odom.twist.twist.angular.y = (double)ImuGyrData.gyr[1] / 900.0;
    ROS_odom.twist.twist.angular.z =
        (double)ImuGyrData.gyr[2] / 900.0; //* imu_angular_vel_;

    odom_trans.header.stamp = timestamp2rostime(Odom_TimeStamp);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = OdomPoseXy.pos_x;
    odom_trans.transform.translation.y = OdomPoseXy.pos_y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = odom_quat;
    odom_broadcaster.sendTransform(odom_trans);

    if ((Odom_TimeStamp - time_pre) > 100000) {
      static uint8_t first = 1;
      if (first) {
        first = 0;
      } else {
        // uint64_t timegap = (Odom_TimeStamp - time_pre);
        // printf("!!!!!!!!!! timeout !!!timegap: %lu, curtime:%lu,
        // pretime:%lu\n", timegap, Odom_TimeStamp, time_pre);
      }
    }
    time_pre = Odom_TimeStamp;
    Odom_pub.publish(ROS_odom);
  }

  if (FrontAxle_update == 1) {
    FrontAxle_update = 0;
    front_axle_angle_fb.chassis_front_axle_angle = FrontAxleAngle;
    front_axle_angle_fb.chassis_front_axle_angle_timestamp =
        FrontAxle_TimeStamp;
    front_axle_angle_fb_pub.publish(front_axle_angle_fb);
  }
  ros::spinOnce();
} */

Segwayrmp::Segwayrmp(std::string node_name)
    : Node(node_name), keep_running_(false) {
  cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&Segwayrmp::CommandVelocityCallback, this,
                std::placeholders::_1));

  batt_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10);
  imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
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
    PublishImuState();
    rclcpp::spin_some(shared_from_this());
    rate.sleep();
  }
}

void Segwayrmp::PublishBatteryState(void) {

  sensor_msgs::msg::BatteryState batt_msg;
  batt_msg.percentage = static_cast<float>(get_bat_soc());
  batt_msg.temperature = static_cast<float>(get_bat_temp());
  batt_msg.voltage = static_cast<float>(get_bat_mvol()) / 1000;
  batt_msg.current = static_cast<float>(get_bat_mcurrent()) / 1000;
  batt_pub_->publish(batt_msg);
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
