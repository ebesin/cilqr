#include "optimizer_utils.h"

#include <Eigen/src/Core/Matrix.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <utility>

#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_state_interface.h"

namespace OptimizerUtils {

void getNearPose(
    const std::shared_ptr<VehicleModelInterface>& vehicle_model_ptr,
    const nav_msgs::msg::Path& path, geometry_msgs::msg::Pose& near_pose,
    size_t& idx, double& dist) {
  // geometry_msgs::msg::Pose cur_pose =
  //     vehicle_model_ptr->getCurState().base_state_.pose.pose;
  geometry_msgs::msg::Pose cur_pose;
  cur_pose.position.x = vehicle_model_ptr->getCurStateVec()(0);
  cur_pose.position.y = vehicle_model_ptr->getCurStateVec()(1);
  if (path.poses.empty()) {
    near_pose = cur_pose;
    idx = 0;
    dist = 0.0;
    return;
  }
  size_t cur_idx{0};
  double min_dist{1000};
  for (size_t i = 0; i < path.poses.size(); ++i) {
    double dist =
        std::hypot(path.poses.at(i).pose.position.x - cur_pose.position.x,
                   path.poses.at(i).pose.position.y - cur_pose.position.y);
    if (dist < min_dist) {
      min_dist = dist;
      cur_idx = i;
    }
  }
  near_pose = path.poses.at(cur_idx).pose;
  idx = cur_idx;
  dist = min_dist;
}

int calApproximateTime(
    const std::shared_ptr<VehicleModelInterface>& vehicle_model_ptr,
    const double total_dist, double& approximate_time) {
  VehicleStateInterface start_state = vehicle_model_ptr->getCurState();
  VehicleStateInterface end_state = vehicle_model_ptr->getEndState();
  VehicleStateInterface max_state = vehicle_model_ptr->getMaxState();
  VehicleStateInterface min_state = vehicle_model_ptr->getMinState();
  approximate_time = 0;
  if (start_state.base_state_.twist.twist.linear.x >
          max_state.base_state_.twist.twist.linear.x ||
      end_state.base_state_.twist.twist.linear.x >
          max_state.base_state_.twist.twist.linear.x ||
      total_dist < 0) {
    return -1;
  }

  double acc_time = (max_state.base_state_.twist.twist.linear.x -
                     start_state.base_state_.twist.twist.linear.x) /
                    max_state.acc_vx_;

  double dacc_time = (max_state.base_state_.twist.twist.linear.x -
                      end_state.base_state_.twist.twist.linear.x) /
                     max_state.acc_vx_;
  double acc_dist = start_state.base_state_.twist.twist.linear.x * acc_time +
                    (max_state.acc_vx_ * acc_time * acc_time) / 2;
  double dacc_dist = end_state.base_state_.twist.twist.linear.x * dacc_time +
                     (max_state.acc_vx_ * dacc_time * dacc_time) / 2;
  if (acc_dist + dacc_dist < total_dist) {
    approximate_time = acc_time + dacc_time +
                       (total_dist - acc_dist - dacc_dist) / max_state.acc_vx_;
    return 0;
  } else {
    double act_max_vx =
        sqrt((2 * max_state.acc_vx_ * max_state.acc_vx_ +
              max_state.acc_vx_ * end_state.base_state_.twist.twist.linear.x *
                  end_state.base_state_.twist.twist.linear.x +
              max_state.acc_vx_ * start_state.base_state_.twist.twist.linear.x *
                  start_state.base_state_.twist.twist.linear.x) /
             (max_state.acc_vx_ + max_state.acc_vx_));
    approximate_time =
        (act_max_vx - start_state.base_state_.twist.twist.linear.x) /
            max_state.acc_vx_ +
        (act_max_vx - end_state.base_state_.twist.twist.linear.x) /
            max_state.acc_vx_;
    return 0;
  }

  return -1;
}

int reSampleTrajByTime(
    const std::shared_ptr<VehicleModelInterface>& vehicle_model_ptr,
    const nav_msgs::msg::Path& ref_path, nav_msgs::msg::Path& resampled_path) {}

int reCalculateTimeInterval(
    const std::shared_ptr<VehicleModelInterface>& vehicle_model_ptr,
    const nav_msgs::msg::Path& ref_path, bool, size_t& point_index,
    double& time_interval) {
  geometry_msgs::msg::Pose near_pose;
  double dist2point{0};
  getNearPose(vehicle_model_ptr, ref_path, near_pose, point_index, dist2point);
  std::cout << "point_index: " << point_index
            << " ref_path.poses.size(): " << ref_path.poses.size() << std::endl;
  if (point_index == ref_path.poses.size() - 1) {
    time_interval = 0;
    return 0;
  }
  double total_dist{0};
  for (size_t i = point_index; i < ref_path.poses.size() - 1; i++) {
    total_dist += std::hypot(ref_path.poses.at(i).pose.position.x -
                                 ref_path.poses.at(i + 1).pose.position.x,
                             ref_path.poses.at(i).pose.position.y -
                                 ref_path.poses.at(i + 1).pose.position.y);
  }
  double approximate_time;
  std::cout << "calApproximateTime..." << std::endl;
  if (calApproximateTime(vehicle_model_ptr, total_dist, approximate_time) < 0) {
    return -2;
  }
  std::cout << "approximate_time: " << approximate_time
            << "  total_dist: " << total_dist << std::endl;

  time_interval = approximate_time / (ref_path.poses.size() - point_index - 1);
  return 0;
}

void convertToMsg(const std::vector<Eigen::VectorXd>& state,
                  nav_msgs::msg::Path& path) {
  if (state.empty()) {
    path.poses.clear();
    return;
  }
  path.poses.clear();
  path.poses.reserve(state.size());
  for (auto s : state) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = s(0);
    pose.pose.position.y = s(1);
    if (s.size() > 2) {
      tf2::Quaternion q;
      q.setRPY(0, 0, s(2));
      pose.pose.orientation = tf2::toMsg(q);
    }
    path.poses.emplace_back(std::move(pose));
  }
}

void convertToStateVec(const nav_msgs::msg::Path& path,
                       std::vector<Eigen::VectorXd>& state_vec) {
  if (path.poses.empty()) {
    state_vec.clear();
    return;
  }
  state_vec.clear();
  state_vec.reserve(path.poses.size());
  for (auto p : path.poses) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(3);
    state << p.pose.position.x, p.pose.position.y,
        tf2::getYaw(p.pose.orientation);
    state_vec.emplace_back(std::move(state));
  }
}

void pathSegmentation(const nav_msgs::msg::Path& path, int unit_size,
                      std::vector<nav_msgs::msg::Path>& paths) {
  int segment_size = path.poses.size() / unit_size;
  int left = path.poses.size() % unit_size;
  if (left < 20 && segment_size > 0) {
    segment_size -= 1;
  }

  for (int i = 0; i < segment_size; i++) {
    nav_msgs::msg::Path single_path;
    single_path.poses.reserve(unit_size);
    single_path.poses.insert(single_path.poses.end(),
                             path.poses.begin() + i * unit_size,
                             path.poses.begin() + (i + 1) * unit_size);
    paths.emplace_back(std::move(single_path));
  }
  nav_msgs::msg::Path single_path;
  single_path.poses.insert(single_path.poses.end(),
                           path.poses.begin() + segment_size * unit_size,
                           path.poses.end());
  paths.emplace_back(std::move(single_path));
}

}  // namespace OptimizerUtils
