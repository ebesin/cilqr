/*
 * @Author       : dwayne
 * @Date         : 2023-06-27
 * @LastEditTime : 2023-06-30
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */
#pragma once

#include <string>

#include "coor_tools.h"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

namespace utils_tool {
class VisualizationTools : public rclcpp::Node {
 private:
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_publisher_;
  /*odom marker and subscriber*/
  visualization_msgs::msg::MarkerArray markers_;
  visualization_msgs::msg::Marker odom_robot_marker_;
  visualization_msgs::msg::Marker odom_text_marker_;
  // visualization_msgs::msg::Marker                          path_marker_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
  // rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr     path_subscriber_;

  /*parameters*/
  bool isVisualizationOdom_;
  std::string odom_topic_name_;

  double vehicle_length_;
  double vehicle_width_;
  double vehicle_height_;
  double vehicle_a_;
  double vehicle_r_;
  double vehicle_g_;
  double vehicle_b_;

  double text_length_;
  double text_width_;
  double text_height_;
  double text_a_;
  double text_r_;
  double text_g_;
  double text_b_;

  double obs_length;
  double obs_width;
  double obs_height;
  double first_obs_pose_x;
  double first_obs_pose_y;
  double first_obs_pose_theta;
  double second_obs_pose_x;
  double second_obs_pose_y;
  double second_obs_pose_theta;
  double third_obs_pose_x;
  double third_obs_pose_y;
  double third_obs_pose_theta;
  double forth_obs_pose_x;
  double forth_obs_pose_y;
  double forth_obs_pose_theta;

  /*callback functions*/
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path);

  /*declare and get parameters*/
  void declareAndGetParameters();

  /*init markers*/
  void initMarkers();

  void declareParameter();

 public:
  VisualizationTools(std::string name);
  ~VisualizationTools();
};

}  // namespace utils_tool
