/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-07-02
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */
#pragma once

#include <cmath>
#include <mpc_msgs/msg/detail/hybrid_trajectory__struct.hpp>
#include <string>
#include <fstream>
#include "coor_tools.h"
#include "mpc_msgs/msg/hybrid_trajectory.hpp"
#include "mpc_msgs/msg/trajectory.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rs_path.h"

namespace utils_tool {
using mpc_msgs::msg::Trajectory;

class PathGenerator : public rclcpp::Node {
 private:
  /* data */

  rclcpp::Publisher<Trajectory>::SharedPtr trajectory_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
  rclcpp::Publisher<mpc_msgs::msg::HybridTrajectory>::SharedPtr
      ref_traj_publisher_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  Trajectory traj_;
  nav_msgs::msg::Path path_;
  mpc_msgs::msg::HybridTrajectory hybrid_traj_;
  std::string traj_type_;
  std::shared_ptr<RSPath> rs_path_ptr_;
  double resulation_;
  double velocity_;
  double wheel_base_;
  double max_steer_;
  double cycle_;
  double amplitude_;
  bool generate_from_file_{false};
  std::string path_folder_;
  std::string path_filename_;
  int generate_mode_;
  bool is_published_{false};
  double path_length_;
  /*路径方向，仅对直线有用*/
  double path_heading_;
  std::stringstream ss_;
  std::ofstream save_file_;
  /*运动方向 1-->前进  -1-->后退*/
  int direction_;

 public:
  PathGenerator(/* args */ std::string name);
  ~PathGenerator();

  std::string get_cur_time_str();

  /**
   * @description  : 完善路径信息（转角、速度等）
   * @return        {*}
   */
  // todo 目前只考虑了前进，后续需要考虑倒车
  void perfectPath(Trajectory& traj, double velocity, double wheel_base,
                   int direction);

  /**
   * @description  : 生成sin曲线路径
   * @param         {double} length: 沿x轴的长度
   * @param         {double} heading: x轴方向
   * @param         {double} cycle: 周期
   * @param         {double} amplitude: 幅度
   * @return        {*}
   */
  void generateSinCurve(double length, double heading, double cycle,
                        double amplitude, int direction);

  /**
   * @description  : 生成直线路径
   * @param         {double} length: 路径长度
   * @param         {double} heading:路径方向
   * @return        {*}
   */
  void generateLineCurve(double length, double heading, int direction);

  void generateBezierCurve(const geometry_msgs::msg::PoseStamped& begin_pose,
                           const geometry_msgs::msg::PoseStamped& end_pose,
                           const double& step);

  /**
   * @description  : 生成圆形路径
   * @return        {*}
   */
  void generateCircleCurve();

  /**
   * @description  : 生成RS曲线
   * @return        {*}
   */
  void generateRSPahtFromFile(const std::string& file_name,
                              nav_msgs::msg::Path& path);

  /**
   * @description  : 定时器回调
   * @return        {*}
   */
  void timerCallback();

  /**
   * @description  : 开启定时器
   * @return        {*}
   */
  void startTimer();

  void generatePathFromFile(const std::string& file_name);

  void loadPathFromFile(const std::string& file_name,
                        nav_msgs::msg::Path& path);

  void loadTrajFromFile(const std::string& file_name,
                        mpc_msgs::msg::HybridTrajectory& hybrid_traj);

  void declareParameter();
};
}  // namespace utils_tool
