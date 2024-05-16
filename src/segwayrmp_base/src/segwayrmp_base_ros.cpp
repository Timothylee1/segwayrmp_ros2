#include "segwayrmp_base/segwayrmp_base_ros.hpp"

uint8_t imu_gyroscope_update_;
uint64_t imu_gyroscope_timestamp_;
chassis_car_speed_data_t speed_data_;
imu_gyr_original_data_t imu_gyroscope_data_;

uint8_t imu_acceleration_update_;
uint64_t imu_acceleration_timestamp_;
imu_acc_original_data_t imu_acceleration_data_;

uint8_t odom_update_;
uint64_t odom_timestamp_;
odom_pos_xy_t odom_pose_xy_;
odom_euler_z_t odom_euler_z_;
odom_euler_xy_t odom_euler_xy_;
odom_vel_line_xy_t odom_velocity_xy_;
namespace westonrobot {

Segwayrmp::Segwayrmp(rclcpp::Node *node) : node_(node), keep_running_(false) {
  timestamp_data_.on_new_data = OdomImuData;
  aprctrl_datastamped_jni_register(&timestamp_data_);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(node_);

  cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10,
      std::bind(&Segwayrmp::CommandVelocityCallback, this,
                std::placeholders::_1));
  batt_pub_ = node_->create_publisher<sensor_msgs::msg::BatteryState>(
      "/battery_state", 10);
  imu_pub_ = node_->create_publisher<sensor_msgs::msg::Imu>("/imu", 10);
  odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
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
  while (keep_running_ && rclcpp::ok()) {
    PublishBatteryState();
    PublishImuOdomState();
    // rclcpp::spin_some();
    rate.sleep();
  }
  rclcpp::shutdown();
}

rclcpp::Time Segwayrmp::Timestamp(int64_t timestamp) {
  uint32_t sec_ = timestamp / 1000000;
  uint32_t nsec_ = (timestamp % 1000000) * 1000;
  return rclcpp::Time(sec_, nsec_);
}

void Segwayrmp::PublishBatteryState(void) {
  sensor_msgs::msg::BatteryState batt_msg;
  batt_msg.present = get_bat_soc() ? true : false;
  batt_msg.percentage = static_cast<float>(get_bat_soc());
  batt_msg.temperature = static_cast<float>(get_bat_temp());
  batt_msg.voltage = static_cast<float>(get_bat_mvol()) / 1000;
  batt_msg.current = static_cast<float>(get_bat_mcurrent()) / 1000;
  batt_pub_->publish(batt_msg);
}

void Segwayrmp::PublishImuState(void) {
  uint64_t imu_stamp = imu_gyroscope_timestamp_ > imu_acceleration_timestamp_
                           ? imu_gyroscope_timestamp_
                           : imu_acceleration_timestamp_;
  ros_imu_.header.stamp = Timestamp(imu_stamp);
  ros_imu_.header.frame_id = "imu";
  ros_imu_.angular_velocity.x = (double)imu_gyroscope_data_.gyr[0] / 900.0;
  ros_imu_.angular_velocity.y = (double)imu_gyroscope_data_.gyr[1] / 900.0;
  ros_imu_.angular_velocity.z = (double)imu_gyroscope_data_.gyr[2] / 900.0;
  ros_imu_.linear_acceleration.x =
      (double)imu_acceleration_data_.acc[0] / 4000.0 * 9.81;
  ros_imu_.linear_acceleration.y =
      (double)imu_acceleration_data_.acc[1] / 4000.0 * 9.81;
  ros_imu_.linear_acceleration.z =
      (double)imu_acceleration_data_.acc[2] / 4000.0 * 9.81;

  // Populate angular velocity covariance
  for (size_t i = 0; i < 9; ++i) {
    // Set diagonal elements to non-zero values, and others to zero
    ros_imu_.angular_velocity_covariance[i] = (i % 4 == 0) ? 0.1 : 0.0;
  }

  // Populate linear velocity covariance
  for (size_t i = 0; i < 9; ++i) {
    // Set diagonal elements to non-zero values, and others to zero
    ros_imu_.linear_acceleration_covariance[i] = (i % 4 == 0) ? 0.15 : 0.0;
  }

  imu_pub_->publish(ros_imu_);
}

void Segwayrmp::PublishImuOdomState(void) {
  if (imu_gyroscope_update_ == 1 && imu_acceleration_update_ == 1) {
    PublishImuState();
    imu_gyroscope_update_ = 0;
    imu_acceleration_update_ = 0;
  }
  if (odom_update_ == 15) {
    odom_update_ = 0;

    quat_.setRPY(0, 0, odom_euler_z_.euler_z / rad_to_deg_);
    // odom_quat_ = tf2::toMsg(quat_);
    tf2::convert(quat_, odom_quat_);

    ros_odom_.header.stamp = Timestamp(odom_timestamp_);
    ros_odom_.header.frame_id = "odom";
    ros_odom_.pose.pose.position.x = odom_pose_xy_.pos_x;
    ros_odom_.pose.pose.position.y = odom_pose_xy_.pos_y;
    ros_odom_.pose.pose.position.z = 0.0;
    ros_odom_.pose.pose.orientation.x = GetOrientationX();
    ros_odom_.pose.pose.orientation.y = GetOrientationY();
    ros_odom_.pose.pose.orientation.z = GetOrientationZ();
    ros_odom_.pose.pose.orientation.w = GetOrientationW();
    ros_odom_.child_frame_id = "base_link";
    ros_odom_.twist.twist.linear.x =
        (double)speed_data_.car_speed / LINE_SPEED_TRANS_GAIN_MPS;
    ros_odom_.twist.twist.linear.y = 0.0;
    ros_odom_.twist.twist.linear.z = 0.0;
    ros_odom_.twist.twist.angular.x =
        (double)imu_gyroscope_data_.gyr[0] / 900.0;
    ros_odom_.twist.twist.angular.y =
        (double)imu_gyroscope_data_.gyr[1] / 900.0;
    ros_odom_.twist.twist.angular.z =
        (double)imu_gyroscope_data_.gyr[2] / 900.0;

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = Timestamp(odom_timestamp_);
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";
    tf_msg.transform.translation.x = odom_pose_xy_.pos_x;
    tf_msg.transform.translation.y = odom_pose_xy_.pos_y;
    tf_msg.transform.translation.z = 0.0;
    tf_msg.transform.rotation = odom_quat_;
    tf_broadcaster_->sendTransform(tf_msg);

    odom_pub_->publish(ros_odom_);
  }
}

void Segwayrmp::OdomImuData(StampedBasicFrame *frame) {
  if (frame->type_id == Chassis_Data_Car_Speed) {
    memcpy(&speed_data_, frame->data, sizeof(speed_data_));
  } else if (frame->type_id == Chassis_Data_Imu_Gyr) {
    memcpy(&imu_gyroscope_data_, frame->data, sizeof(imu_gyroscope_data_));
    imu_gyroscope_timestamp_ = frame->timestamp;
    imu_gyroscope_update_ = 1;
  } else if (frame->type_id == Chassis_Data_Imu_Acc) {
    memcpy(&imu_acceleration_data_, frame->data,
           sizeof(imu_acceleration_data_));
    imu_acceleration_timestamp_ = frame->timestamp;
    imu_acceleration_update_ = 1;
  } else if (frame->type_id == Chassis_Data_Odom_Pose_xy) {
    memcpy(&odom_pose_xy_, frame->data, sizeof(odom_pose_xy_));
    odom_timestamp_ = frame->timestamp;
    odom_update_ |= 1;
  } else if (frame->type_id == Chassis_Data_Odom_Euler_xy) {
    memcpy(&odom_euler_xy_, frame->data, sizeof(odom_euler_xy_));
    odom_timestamp_ = frame->timestamp;
    odom_update_ |= 2;
  } else if (frame->type_id == Chassis_Data_Odom_Euler_z) {
    memcpy(&odom_euler_z_, frame->data, sizeof(odom_euler_z_));
    odom_timestamp_ = frame->timestamp;
    odom_update_ |= 4;
  } else if (frame->type_id == Chassis_Data_Odom_Linevel_xy) {
    memcpy(&odom_velocity_xy_, frame->data, sizeof(odom_velocity_xy_));
    odom_timestamp_ = frame->timestamp;
    odom_update_ |= 8;
  }
}

double Segwayrmp::GetOrientationX(void) {
  float x = odom_euler_xy_.euler_x / rad_to_deg_ / 2.0;
  float y = odom_euler_xy_.euler_y / rad_to_deg_ / 2.0;
  float z = odom_euler_z_.euler_z / rad_to_deg_ / 2.0;

  float fCosHRoll = cos(x);
  float fSinHRoll = sin(x);
  float fCosHPitch = cos(y);
  float fSinHPitch = sin(y);
  float fCosHYaw = cos(z);
  float fSinHYaw = sin(z);

  return (fSinHRoll * fCosHPitch * fCosHYaw -
          fCosHRoll * fSinHPitch * fSinHYaw);
}

double Segwayrmp::GetOrientationY(void) {
  float x = odom_euler_xy_.euler_x / rad_to_deg_ / 2.0;
  float y = odom_euler_xy_.euler_y / rad_to_deg_ / 2.0;
  float z = odom_euler_z_.euler_z / rad_to_deg_ / 2.0;

  float fCosHRoll = cos(x);
  float fSinHRoll = sin(x);
  float fCosHPitch = cos(y);
  float fSinHPitch = sin(y);
  float fCosHYaw = cos(z);
  float fSinHYaw = sin(z);

  return (fCosHRoll * fSinHPitch * fCosHYaw +
          fSinHRoll * fCosHPitch * fSinHYaw);
}

double Segwayrmp::GetOrientationZ(void) {
  float x = odom_euler_xy_.euler_x / rad_to_deg_ / 2.0;
  float y = odom_euler_xy_.euler_y / rad_to_deg_ / 2.0;
  float z = odom_euler_z_.euler_z / rad_to_deg_ / 2.0;

  float fCosHRoll = cos(x);
  float fSinHRoll = sin(x);
  float fCosHPitch = cos(y);
  float fSinHPitch = sin(y);
  float fCosHYaw = cos(z);
  float fSinHYaw = sin(z);

  return (fCosHRoll * fCosHPitch * fSinHYaw -
          fSinHRoll * fSinHPitch * fCosHYaw);
}

double Segwayrmp::GetOrientationW(void) {
  float x = odom_euler_xy_.euler_x / rad_to_deg_ / 2.0;
  float y = odom_euler_xy_.euler_y / rad_to_deg_ / 2.0;
  float z = odom_euler_z_.euler_z / rad_to_deg_ / 2.0;

  float fCosHRoll = cos(x);
  float fSinHRoll = sin(x);
  float fCosHPitch = cos(y);
  float fSinHPitch = sin(y);
  float fCosHYaw = cos(z);
  float fSinHYaw = sin(z);

  return (fCosHRoll * fCosHPitch * fCosHYaw +
          fSinHRoll * fSinHPitch * fSinHYaw);
}

/* Callbacks */
void Segwayrmp::CommandVelocityCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  double angular_vel = msg->angular.z;
  double linear_vel = msg->linear.x;
  RCLCPP_DEBUG(node_->get_logger(),
               "Received Twist message: linear.x= %f, angular.z= %f",
               linear_vel, angular_vel);

  // Enables control of the system via /cmd_vel
  set_enable_ctrl(1);
  set_cmd_vel(linear_vel, angular_vel);
}

} // namespace westonrobot
