/*
 * @Author       : dwayne
 * @Date         : 2023-06-27
 * @LastEditTime : 2023-06-30
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */
#include "visualization_tool.hpp"

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>

#include "coor_tools.h"

namespace utils_tool {

VisualizationTools::VisualizationTools(std::string name) : Node(name) {
  declareParameter();
  initMarkers();
  odom_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_name_, rclcpp::SystemDefaultsQoS(),
      std::bind(&VisualizationTools::odomCallback, this,
                std::placeholders::_1));
  // path_subscriber_ =
  marker_publisher_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "visualization_marker", rclcpp::SystemDefaultsQoS());
}

VisualizationTools::~VisualizationTools() {}

void VisualizationTools::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr odom) {
  geometry_msgs::msg::Pose trans;
  trans.position.x = 1.0 / 2 - 0.2;
  markers_.markers[0].pose = utils_tool::absoluteSum(odom->pose.pose, trans);
  // markers_.markers[0].pose = odom->pose.pose;
  std::string marker_text(
      "linear_velocity:" + std::to_string(odom->twist.twist.linear.x) + "\n" +
      "angular_velocity:" + std::to_string(odom->twist.twist.angular.z) + "\n" +
      "heading:" +
      std::to_string(
          rad2Deg(getYawFromQuaternion(odom->pose.pose.orientation))));
  markers_.markers[1].text = marker_text;
  markers_.markers[1].pose.position = odom->pose.pose.position;
  markers_.markers[1].pose.position.x += 3;
  marker_publisher_->publish(markers_);
}

void VisualizationTools::pathCallback(
    const nav_msgs::msg::Path::SharedPtr path) {}

void VisualizationTools::initMarkers() {
  /*odom marker*/
  odom_robot_marker_.header.frame_id = "map";
  odom_robot_marker_.type = visualization_msgs::msg::Marker::CUBE;
  odom_robot_marker_.action = visualization_msgs::msg::Marker::MODIFY;
  odom_robot_marker_.ns = std::string("utils_tool");
  odom_robot_marker_.id = 0;

  odom_robot_marker_.scale.x = vehicle_length_;  // 长
  odom_robot_marker_.scale.y = vehicle_width_;   // 宽
  odom_robot_marker_.scale.z = vehicle_height_;  // 高

  odom_robot_marker_.color.a = vehicle_a_;
  odom_robot_marker_.color.r = vehicle_r_;
  odom_robot_marker_.color.g = vehicle_g_;
  odom_robot_marker_.color.b = vehicle_b_;

  /*odom text marker*/
  odom_text_marker_.header.frame_id = "map";
  odom_text_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  odom_text_marker_.action = visualization_msgs::msg::Marker::MODIFY;
  odom_text_marker_.ns = std::string("utils_tool");
  odom_robot_marker_.id = 1;

  odom_text_marker_.scale.x = text_length_;  // 长
  odom_text_marker_.scale.y = text_width_;   // 宽
  odom_text_marker_.scale.z = text_height_;  // 高

  odom_text_marker_.color.a = 0.0;
  odom_text_marker_.color.r = text_r_;
  odom_text_marker_.color.g = text_g_;
  odom_text_marker_.color.b = text_b_;

  markers_.markers.push_back(odom_robot_marker_);
  markers_.markers.push_back(odom_text_marker_);

  visualization_msgs::msg::Marker obs1;
  obs1.header.frame_id = "map";
  obs1.type = visualization_msgs::msg::Marker::CUBE;
  obs1.ns = std::string("utils_tool2");
  obs1.id = 1;

  obs1.scale.x = obs_length;  // 长
  obs1.scale.y = obs_width;   // 宽
  obs1.scale.z = obs_height;  // 高

  obs1.color.a = 1.0;
  obs1.color.r = 1.0;
  obs1.color.g = 0.0;
  obs1.color.b = 0.0;

  obs1.pose.position.x = first_obs_pose_x;
  obs1.pose.position.y = first_obs_pose_y;

  visualization_msgs::msg::Marker obs2;
  obs2.header.frame_id = "map";
  obs2.type = visualization_msgs::msg::Marker::CUBE;
  obs2.ns = std::string("utils_tool2");
  obs2.id = 2;

  obs2.scale.x = obs_length;  // 长
  obs2.scale.y = obs_width;   // 宽
  obs2.scale.z = obs_height;  // 高

  obs2.color.a = 1.0;
  obs2.color.r = 1.0;
  obs2.color.g = 0.0;
  obs2.color.b = 0.0;
  obs2.pose.position.x = second_obs_pose_x;
  obs2.pose.position.y = second_obs_pose_y;

  visualization_msgs::msg::Marker obs3;
  obs3.header.frame_id = "map";
  obs3.type = visualization_msgs::msg::Marker::CUBE;
  obs3.ns = std::string("utils_tool2");
  obs3.id = 3;

  obs3.scale.x = obs_length;  // 长
  obs3.scale.y = obs_width;   // 宽
  obs3.scale.z = obs_height;  // 高

  obs3.color.a = 1.0;
  obs3.color.r = 1.0;
  obs3.color.g = 0.0;
  obs3.color.b = 0.0;
  obs3.pose.position.x = third_obs_pose_x;
  obs3.pose.position.y = third_obs_pose_y;

  visualization_msgs::msg::Marker obs4;
  obs4.header.frame_id = "map";
  obs4.type = visualization_msgs::msg::Marker::CUBE;
  obs4.ns = std::string("utils_tool2");
  obs4.id = 4;

  obs4.scale.x = obs_length;  // 长
  obs4.scale.y = obs_width;   // 宽
  obs4.scale.z = obs_height;  // 高

  obs4.color.a = 1.0;
  obs4.color.r = 1.0;
  obs4.color.g = 0.0;
  obs4.color.b = 0.0;
  obs4.pose.position.x = forth_obs_pose_x;
  obs4.pose.position.y = forth_obs_pose_y;
  markers_.markers.push_back(obs1);
  markers_.markers.push_back(obs2);
  markers_.markers.push_back(obs3);
  markers_.markers.push_back(obs4);
}

void VisualizationTools::declareAndGetParameters() {}

void VisualizationTools::declareParameter() {
  get_parameter_or("is_visualization_odom", isVisualizationOdom_, true);
  get_parameter_or("odom_topic_name", odom_topic_name_, std::string("odom"));
  declare_parameter("vehicle_length", 0.978);
  declare_parameter("vehicle_width", 0.721);
  declare_parameter("vehicle_height", 0.330);
  declare_parameter("vehicle_a", 0.8);
  declare_parameter("vehicle_r", 0.286);
  declare_parameter("vehicle_g", 0.314);
  declare_parameter("vehicle_b", 0.341);

  declare_parameter("text_length", 0.2);
  declare_parameter("text_width", 0.2);
  declare_parameter("text_height", 0.2);
  declare_parameter("text_a", 0.8);
  declare_parameter("text_r", 0.0);
  declare_parameter("text_g", 0.0);
  declare_parameter("text_b", 0.0);

  declare_parameter("obs_length", 0.0);
  declare_parameter("obs_width", 0.0);
  declare_parameter("obs_height", 0.0);
  declare_parameter("first_obs_pose_x", 0.0);
  declare_parameter("first_obs_pose_y", 0.0);
  declare_parameter("first_obs_pose_theta", 0.0);
  declare_parameter("second_obs_pose_x", 0.0);
  declare_parameter("second_obs_pose_y", 0.0);
  declare_parameter("second_obs_pose_theta", 0.0);
  declare_parameter("third_obs_pose_y", 0.0);
  declare_parameter("third_obs_pose_x", 0.0);
  declare_parameter("third_obs_pose_theta", 0.0);
  declare_parameter("forth_obs_pose_x", 0.0);
  declare_parameter("forth_obs_pose_y", 0.0);
  declare_parameter("forth_obs_pose_theta", 0.0);

  /*get parameters*/
  vehicle_length_ = get_parameter("vehicle_length").as_double();
  vehicle_width_ = get_parameter("vehicle_width").as_double();
  vehicle_height_ = get_parameter("vehicle_height").as_double();
  vehicle_a_ = get_parameter("vehicle_a").as_double();
  vehicle_r_ = get_parameter("vehicle_r").as_double();
  vehicle_g_ = get_parameter("vehicle_g").as_double();
  vehicle_b_ = get_parameter("vehicle_b").as_double();

  text_length_ = get_parameter("text_length").as_double();
  text_width_ = get_parameter("text_width").as_double();
  text_height_ = get_parameter("text_height").as_double();
  text_a_ = get_parameter("text_a").as_double();
  text_r_ = get_parameter("text_r").as_double();
  text_g_ = get_parameter("text_g").as_double();
  text_b_ = get_parameter("text_b").as_double();

  obs_length = get_parameter("obs_length").as_double();
  obs_width = get_parameter("obs_width").as_double();
  obs_height = get_parameter("obs_height").as_double();
  first_obs_pose_x = get_parameter("first_obs_pose_x").as_double();
  first_obs_pose_y = get_parameter("first_obs_pose_y").as_double();
  first_obs_pose_theta = get_parameter("first_obs_pose_theta").as_double();
  second_obs_pose_x = get_parameter("second_obs_pose_x").as_double();
  second_obs_pose_y = get_parameter("second_obs_pose_y").as_double();
  second_obs_pose_theta = get_parameter("second_obs_pose_theta").as_double();
  third_obs_pose_x = get_parameter("third_obs_pose_x").as_double();
  third_obs_pose_y = get_parameter("third_obs_pose_y").as_double();
  third_obs_pose_theta = get_parameter("third_obs_pose_theta").as_double();
  forth_obs_pose_x = get_parameter("forth_obs_pose_x").as_double();
  forth_obs_pose_y = get_parameter("forth_obs_pose_y").as_double();
  forth_obs_pose_theta = get_parameter("forth_obs_pose_theta").as_double();
}

}  // namespace utils_tool
