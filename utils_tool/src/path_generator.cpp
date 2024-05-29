/*
 * @Author       : dwayne
 * @Date         : 2023-06-28
 * @LastEditTime : 2023-07-02
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#include "path_generator.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <mpc_msgs/msg/detail/hybrid_lane__struct.hpp>
#include <mpc_msgs/msg/detail/hybrid_trajectory__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/logging.hpp>
#include <string>
#include <utility>

#include "bezier_plan.h"
#include "coor_tools.h"
#include "type.hpp"

namespace utils_tool {
PathGenerator::PathGenerator(std::string name) : Node(name) {
  declareParameter();
  trajectory_publisher_ =
      create_publisher<Trajectory>("/mpc_controller/input/reference_trajectory",
                                   rclcpp::SystemDefaultsQoS());
  path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/generate_path", rclcpp::SystemDefaultsQoS());
  ref_traj_publisher_ = create_publisher<mpc_msgs::msg::HybridTrajectory>(
      "/hybrid_traj", rclcpp::SystemDefaultsQoS());
  this->rs_path_ptr_ = std::make_shared<RSPath>(wheel_base_ / tan(max_steer_));
  std::stringstream ss;
  ss << "/home/dwayne/workspace/navigation/nav2_ws/src/navigation/"
        "utils_tool/data/"
     << get_cur_time_str() << ".txt";
  save_file_.open(ss.str());
  switch (generate_mode_) {
    case 1: {
      generatePathFromFile(path_folder_ + "/" + path_filename_);
      break;
    }
    case 2: {
      if (traj_type_ == std::string("line")) {
        declare_parameter("path_length", 20.0);
        declare_parameter("path_heading", 0.0);
        path_length_ = get_parameter("path_length").as_double();
        path_heading_ =
            utils_tool::deg2Rad(get_parameter("path_heading").as_double());
        generateLineCurve(path_length_, path_heading_, direction_);
      } else if (traj_type_ == std::string("sin")) {
        declare_parameter("path_length", 20.0);
        declare_parameter("path_heading", 0.0);
        declare_parameter("sin_cycle", 4 * M_PI);
        declare_parameter("sin_amplitude", 4.0);
        path_length_ = get_parameter("path_length").as_double();
        path_heading_ =
            utils_tool::deg2Rad(get_parameter("path_heading").as_double());
        cycle_ = get_parameter("sin_cycle").as_double();
        amplitude_ = get_parameter("sin_amplitude").as_double();
        generateSinCurve(path_length_, path_heading_, cycle_, amplitude_,
                         direction_);
      } else {
        RCLCPP_ERROR(get_logger(), "no such type of curve!!!");
      }
      break;
    }
    case 3: {
      geometry_msgs::msg::PoseStamped begin_pose;
      geometry_msgs::msg::PoseStamped end_pose;
      end_pose.pose.position.x = 3;
      end_pose.pose.position.y = 3;
      generateBezierCurve(begin_pose, end_pose, 0.05);
      break;
    }
    case 4: {
      // mpc_msgs::msg::HybridTrajectory hybrid_traj;
      loadTrajFromFile(path_folder_ + "/" + path_filename_, hybrid_traj_);
      // ref_traj_publisher_->publish(hybrid_traj);
      // nav_msgs::msg::Path path;
      path_.header.frame_id = "map";
      path_.poses.clear();
      for (auto t : hybrid_traj_.trajectory) {
        path_.poses.insert(path_.poses.end(), t.path.poses.begin(),
                           t.path.poses.end());
      }
      // path_publisher_->publish(path_);
      startTimer();
      break;
    }
    case 5: {
      path_.header.frame_id = "map";
      nav_msgs::msg::Path temp_path;
      generateRSPahtFromFile(path_folder_ + "/" + path_filename_, path_);
      std::stringstream ss;
      ss << "/home/dwayne/workspace/navigation/nav2_ws/src/navigation/"
            "guided_hybrid_planner/data/exp_rs_path/"
         << "gen.txt";
      std::ofstream file(ss.str());
      for (auto p : path_.poses) {
        file << p.pose << std::endl;
      }
      // path_.poses.clear();
      // path_.poses.emplace_back(temp_path.poses.front());
      // path_.poses.emplace_back(temp_path.poses.back());
      startTimer();
      break;
    }
  }
}

PathGenerator::~PathGenerator() {}

void PathGenerator::perfectPath(Trajectory& traj, double velocity,
                                double wheel_base, int direction) {
  auto getDistance = [](geometry_msgs::msg::Point a,
                        geometry_msgs::msg::Point b) {
    return sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y));
  };

  auto sign = [](int val) {
    if (val > 0) {
      return 1;
    } else if (val < 0) {
      return -1;
    } else {
      return 0;
    }
  };

  rclcpp::Duration duration = rclcpp::Duration::from_seconds(0.0);
  for (int i = 0; i < traj.points.size() - 1; i++) {
    traj.points[i].longitudinal_velocity_mps = sign(direction) * velocity;
    traj.points[i].acceleration_mps2 = 0;
    traj.points[i].time_from_start = duration;
    duration = duration + rclcpp::Duration::from_seconds(
                              getDistance(traj.points[i].pose.position,
                                          traj.points[i + 1].pose.position) /
                              velocity);

    traj.points[i].rear_wheel_angle_rad = 0;
    // 前轮转角计算
    double delta_theta =
        sign(direction) *
        (getYawFromQuaternion(traj.points[i + 1].pose.orientation) -
         getYawFromQuaternion(traj.points[i].pose.orientation));
    if (delta_theta > M_PI)
      delta_theta = delta_theta - 2 * M_PI;
    else if (delta_theta < -M_PI)
      delta_theta = delta_theta + 2 * M_PI;
    double tan_theta = wheel_base * delta_theta / resulation_;
    traj.points[i].front_wheel_angle_rad = atan(tan_theta);
    traj.points[i].heading_rate_rps =
        sign(direction) * velocity * tan_theta / wheel_base;
  }
  traj.points[traj.points.size() - 1].longitudinal_velocity_mps = 0;
  traj.points[traj.points.size() - 1].acceleration_mps2 = 0;
  traj.points[traj.points.size() - 1].time_from_start = duration;
  traj.points[traj.points.size() - 1].rear_wheel_angle_rad = 0;
  traj.points[traj.points.size() - 1].front_wheel_angle_rad = 0;
  traj.points[traj.points.size() - 1].heading_rate_rps = 0;
}

void PathGenerator::generateSinCurve(double length, double heading,
                                     double cycle, double amplitude,
                                     int direction) {
  Trajectory traj;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  int size = static_cast<int>(round(length / resulation_));
  RCLCPP_INFO(get_logger(), "path point size:%d", size);
  auto sinCurve = [](double cycle, double amplitude, double x) {
    double omega = 2 * M_PI / cycle;
    return amplitude * sin(omega * x);
  };
  auto sinCurveDerivative = [](double cycle, double amplitude, double x) {
    double omega = 2 * M_PI / cycle;
    return amplitude * omega * cos(omega * x);
  };
  auto sign = [](int val) {
    if (val > 0) {
      return 1;
    } else if (val < 0) {
      return -1;
    } else {
      return 0;
    }
  };
  traj.points.resize(size + 1);
  path.poses.resize(size + 1);
  traj.points[0].pose.position.x = 0;
  traj.points[0].pose.position.y = 0;
  path.poses[0].pose = traj.points[0].pose;
  for (int i = 0; i <= size; i++) {
    double x_ori = i * resulation_;
    double y_ori = sinCurve(cycle, amplitude, i * resulation_);
    traj.points[i].pose.orientation = createQuaternionMsgFromYaw(
        atan(sinCurveDerivative(cycle, amplitude, i * resulation_)) + heading +
        (direction == 1 ? 0 : 3.14159));
    traj.points[i].pose.position.x =
        cos(heading) * x_ori - sin(heading) * y_ori;
    traj.points[i].pose.position.y =
        sin(heading) * x_ori + cos(heading) * y_ori;
    path.poses[i].pose = traj.points[i].pose;
  }
  traj.points[size].pose.orientation = traj.points[size - 1].pose.orientation;
  perfectPath(traj, velocity_, wheel_base_, direction);
  traj_ = traj;
  path_ = path;
  // trajectory_publisher_->publish(traj);
  // path_publisher_->publish(path);
  startTimer();
}

void PathGenerator::generateLineCurve(double length, double heading,
                                      int direction) {
  Trajectory traj;
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  int size = static_cast<int>(round(length / resulation_));
  RCLCPP_INFO(get_logger(), "path point size:%d", size);
  double vec_i = cos(heading);
  double vec_j = sin(heading);
  traj.points.resize(size + 1);
  path.poses.resize(size + 1);
  traj.points[0].pose.position.x = 0;
  traj.points[0].pose.position.y = 0;
  path.poses[0].pose = traj.points[0].pose;
  for (int i = 1; i <= size; i++) {
    traj.points[i].pose.position.x = i * resulation_ * vec_i;
    traj.points[i].pose.position.y = i * resulation_ * vec_j;
    traj.points[i - 1].pose.orientation =
        createQuaternionMsgFromYaw(getHeadingFromVector(
            traj.points[i].pose.position.x - traj.points[i - 1].pose.position.x,
            traj.points[i].pose.position.y -
                traj.points[i - 1].pose.position.y));
    path.poses[i].pose = traj.points[i].pose;
  }
  traj.points[size].pose.orientation = traj.points[size - 1].pose.orientation;
  perfectPath(traj, velocity_, wheel_base_, direction);
  RCLCPP_INFO_STREAM(
      get_logger(),
      "last_x:" << traj.points[size].pose.position.x
                << "  last_y:" << traj.points[size].pose.position.y);
  traj_ = traj;
  path_ = path;
  // trajectory_publisher_->publish(traj);
  // path_publisher_->publish(path);
  for (auto& p : path.poses) {
    save_file_ << p << std::endl;
  }
  startTimer();
}

void PathGenerator::generateCircleCurve() {}

void PathGenerator::generateRSPahtFromFile(const std::string& file_name,
                                           nav_msgs::msg::Path& path) {
  nav_msgs::msg::Path temp_path;
  loadPathFromFile(file_name, temp_path);
  std::stringstream ss;
  ss << "/home/dwayne/workspace/navigation/nav2_ws/src/navigation/"
        "guided_hybrid_planner/data/res.txt";
  std::ofstream file(ss.str());
  RCLCPP_INFO_STREAM(get_logger(), "path_size: " << temp_path.poses.size());
  VectorVec3d rs_path;
  for (int i = 0; i < temp_path.poses.size() - 1; i++) {
    Vec3d start;
    start << temp_path.poses.at(i).pose.position.x,
        temp_path.poses.at(i).pose.position.y,
        getYawFromQuaternion(temp_path.poses.at(i).pose.orientation);
    Vec3d goal;
    goal << temp_path.poses.at(i + 1).pose.position.x,
        temp_path.poses.at(i + 1).pose.position.y,
        getYawFromQuaternion(temp_path.poses.at(i + 1).pose.orientation);
    double distance;
    VectorVec3d temp_path =
        rs_path_ptr_->GetRSPath(start, goal, resulation_, distance);
    rs_path.insert(rs_path.end(), temp_path.begin(), temp_path.end());
  }
  for (const auto& p : rs_path) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = p.x();
    pose.pose.position.y = p.y();
    pose.pose.orientation = createQuaternionMsgFromYaw(p.z());
    file << p.x() << " " << p.y() << " " << p.z() << " " << std::endl;
    path.poses.emplace_back(std::move(pose));
  }
}

void PathGenerator::generateBezierCurve(
    const geometry_msgs::msg::PoseStamped& begin_pose,
    const geometry_msgs::msg::PoseStamped& end_pose, const double& step) {
  double dist = doBezierPlan(begin_pose, end_pose, path_, step);
  path_.header.frame_id = "map";
  startTimer();
}

void PathGenerator::timerCallback() {
  trajectory_publisher_->publish(traj_);
  path_publisher_->publish(path_);
  if (generate_mode_ == 4) {
    if (!is_published_) {
      ref_traj_publisher_->publish(hybrid_traj_);
      is_published_ = true;
    }
  }
  // ref_traj_publisher_-?
}

void PathGenerator::startTimer() {
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(0.1));
  pub_timer_ = create_wall_timer(
      period_ns, std::bind(&PathGenerator::timerCallback, this));
}

void PathGenerator::generatePathFromFile(const std::string& file_name) {
  path_.header.frame_id = "map";
  loadPathFromFile(file_name, path_);
  startTimer();
}

void PathGenerator::loadPathFromFile(const std::string& file_name,
                                     nav_msgs::msg::Path& path) {
  std::ifstream path_file(file_name);
  path.poses.clear();
  if (!path_file.is_open()) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "unable to open file named <" << file_name << ">");
    return;
  }
  std::string temp;
  bool first_line = true;
  while (std::getline(path_file, temp)) {
    if (first_line) {
      first_line = false;
      continue;
    }
    std::istringstream is(temp);
    std::string s;
    geometry_msgs::msg::PoseStamped pose;
    int col = 0;
    while (is >> s) {
      switch (col++) {
        case 0:
          pose.pose.position.x = atof(s.c_str());
          break;
        case 1:
          pose.pose.position.y = atof(s.c_str());
          break;
        case 2:
          pose.pose.orientation = createQuaternionMsgFromYaw(atof(s.c_str()));
          break;
      }
    }
    path.poses.push_back(pose);
  }
}

void PathGenerator::loadTrajFromFile(
    const std::string& file_name,
    mpc_msgs::msg::HybridTrajectory& hybrid_traj) {
  hybrid_traj.trajectory.clear();
  // nav_msgs::msg::Path temp_path;
  mpc_msgs::msg::HybridLane temp_lane;
  std::ifstream path_file(file_name);
  if (!path_file.is_open()) {
    RCLCPP_WARN_STREAM(get_logger(),
                       "unable to open file named <" << file_name << ">");
    return;
  }
  std::string temp;
  bool first_line = true;
  int pre_direction;
  while (std::getline(path_file, temp)) {
    std::istringstream is(temp);
    std::string s;
    geometry_msgs::msg::PoseStamped pose;
    int cur_direction{-1};
    int col = 0;
    while (is >> s) {
      switch (col++) {
        case 0:
          pose.pose.position.x = atof(s.c_str());
          break;
        case 1:
          pose.pose.position.y = atof(s.c_str());
          break;
        case 2:
          pose.pose.orientation = createQuaternionMsgFromYaw(atof(s.c_str()));
          break;
        case 3:
          cur_direction = atoi(s.c_str());
          break;
      }
    }
    if (first_line) {
      first_line = false;
      temp_lane.direction.direction = cur_direction;
      temp_lane.path.poses.push_back(pose);
    } else {
      if (cur_direction == pre_direction || cur_direction == -1) {
        temp_lane.path.poses.push_back(pose);
      } else {
        hybrid_traj.trajectory.emplace_back(temp_lane);
        temp_lane.path.poses.clear();
        temp_lane.direction.direction = cur_direction;
        temp_lane.path.poses.push_back(pose);
      }
    }
    if (cur_direction != -1) {
      pre_direction = cur_direction;
    }
  }
  hybrid_traj.trajectory.emplace_back(temp_lane);
}

std::string PathGenerator::get_cur_time_str() {
  auto now = std::chrono::system_clock::now();
  auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                now.time_since_epoch()) %
            1000;
  std::time_t currentTime = std::chrono::system_clock::to_time_t(now);
  std::tm* timeInfo = std::localtime(&currentTime);
  std::stringstream ss;
  ss << std::put_time(timeInfo, "%Y-%m-%d-%H-%M-%S") << "-" << ms.count();
  return ss.str();
}

void PathGenerator::declareParameter() {
  declare_parameter("traj_type", "line");
  declare_parameter("resulation", 0.1);
  declare_parameter("velocity", 0.5);
  declare_parameter("wheel_base", 0.65);
  declare_parameter("max_steer", 25.0);
  declare_parameter("direction", 1);
  declare_parameter("generate_from_file", false);
  declare_parameter("generate_mode", 1);
  declare_parameter("path_folder", std::string(""));
  declare_parameter("path_filename", std::string(""));

  get_parameter_or("traj_type", traj_type_, std::string("line"));
  get_parameter_or("resulation", resulation_, 0.1);
  get_parameter_or("velocity", velocity_, 0.5);
  get_parameter_or("wheel_base", wheel_base_, 0.65);
  get_parameter_or("max_steer", max_steer_, 25.0);
  max_steer_ = deg2Rad(max_steer_);
  get_parameter_or("direction", direction_, 1);
  get_parameter_or("generate_from_file", generate_from_file_, false);
  get_parameter_or("generate_mode", generate_mode_, 1);
  get_parameter_or("path_folder", path_folder_, std::string(""));
  get_parameter_or("path_filename", path_filename_, std::string(""));
}

}  // namespace utils_tool
