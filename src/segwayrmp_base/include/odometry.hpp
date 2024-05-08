//
// Created by efan on 3/19/20.
//

#ifndef ODOMETRY_HPP
#define ODOMETRY_HPP

#include <fstream>
#include <queue>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "odometry_data.hpp"

class ImuRecord {
 public:
  std::vector<OdometryData::BaseImu> imu_buffer_;
  bool first = true;
  OdometryData::BaseImu getAverageImu(int64_t ts) {
    std::vector<OdometryData::BaseImu>::reverse_iterator i = imu_buffer_.rbegin();
    while (i != imu_buffer_.rend() && i->TimeStamp > ts) {
      ++i;
    }
    double yaw_sum_ = 0;
    int64_t time_gap_ = 0;
    OdometryData::BaseImu imu_after = *i;
    if (i != imu_buffer_.rend()) ++i;
    for (; i != imu_buffer_.rend(); ++i) {
      yaw_sum_ += (i->baseYaw + imu_after.baseYaw) / 2 *
                  (imu_after.TimeStamp - i->TimeStamp);
      time_gap_ += imu_after.TimeStamp - i->TimeStamp;
    }
    if (i != imu_buffer_.rend() && ts != i->TimeStamp) {
      std::vector<OdometryData::BaseImu>::reverse_iterator j = i;
      j--;
      OdometryData::BaseImu imu_suanz = i->Interpolation(ts, *i, *j);
      yaw_sum_ += (i->baseYaw + imu_suanz.baseYaw) / 2 *
                  (imu_suanz.TimeStamp - i->TimeStamp);
      time_gap_ += imu_suanz.TimeStamp - i->TimeStamp;
    }
    if (time_gap_ == 0) return imu_buffer_.back();
    OdometryData::BaseImu res_imu(ts, yaw_sum_ / time_gap_);
    imu_buffer_.clear();
    imu_buffer_.push_back(res_imu);
    return res_imu;
  }
};

class odometry {
 public:
  odometry();
  ~odometry() {}
  /**
   * Author: efan
   * \brief: interface function
   * This function is called when imu data is obtained
   * If the function returns true, call function GetOdometry()
   * */
  bool add_imubase(OdometryData::BaseImu &imudata);
  /**
   * Author: efan
   * \brief: interface function
   * This function is called when ticks data is obtained
   * If the function returns true, call function GetOdometry()
   * */
  bool add_ticks(OdometryData::Ticks &ticksdata);
  /**
   * Author: efan
   * \brief: interface function
   * This function return the GetOdometry of the robot
   * */
  Odometry GetOdometry();

  OdometryData *created(OdometryData::BaseImu &imudata,
                      OdometryData::Ticks &ticksdata);

  int m_num_out = 0;

 private:
  void estimate(OdometryData *rawdata);
  void init();

  double m_omegaImu_bias = -0.00016859;
  int m_num_static = 0;
  const int m_num_ignore_static = 5;
  double m_imu_static_sum = 0;
  int64_t m_current_ts;
  bool m_isInit = false;
  bool m_initial = true;
  int64_t m_prev_t_read;
  int64_t m_current_t_read;
  int64_t m_prev_right_count;
  int64_t m_prev_left_count;

  double m_count_to_right_rot_;
  double m_count_to_left_rot_;
  double m_distance_per_count_;
  double m_wheels_distance;
  Odometry m_odometry_;

  bool m_if_record = false;
  std::ofstream m_record_file;
  std::string m_record_file_path;

  ImuRecord m_imu_record_;
  std::queue<OdometryData::Ticks> m_futureTicks_;
};

#endif  // ODOMETRY_HPP
