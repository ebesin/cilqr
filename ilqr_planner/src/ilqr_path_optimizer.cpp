#include "ilqr_path_optimizer.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <cstddef>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <iostream>
#include <utility>

#include "optimizer_utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace Optimizer {
IlqrPathOptimizer::IlqrPathOptimizer(
    const std::shared_ptr<VehicleModel::VehicleModelInterface>&
        vehicle_model_ptr,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Q_end,
    const Eigen::MatrixXd& R)
    : IlqrOptimizer(vehicle_model_ptr, Q, Q_end, R) {}

int IlqrPathOptimizer::optimize(const nav_msgs::msg::Path& ref_path,
                                nav_msgs::msg::Path& opt_path) {
  size_t waypoint_idx{0};
  double time_interval{0};
  OptimizerUtils::reCalculateTimeInterval(m_vehicle_model_ptr, ref_path, false,
                                          waypoint_idx, time_interval);
  std::cout << "opt_time_interval: " << time_interval << std::endl;
  // setTimeInterval(time_interval * 1.5);
  setTimeInterval(0.1);
  std::vector<Eigen::VectorXd> ref_state;
  ref_state.reserve(ref_path.poses.size() - waypoint_idx);
  for (size_t i = waypoint_idx; i < ref_path.poses.size(); i++) {
    Eigen::VectorXd state = Eigen::VectorXd::Zero(3);
    state << ref_path.poses.at(i).pose.position.x,
        ref_path.poses.at(i).pose.position.y,
        tf2::getYaw(ref_path.poses.at(i).pose.orientation);
    ref_state.push_back(state);
  }
  std::vector<Eigen::VectorXd> opt_state;
  std::vector<Eigen::VectorXd> control;
  IlqrOptimizer::optimize(ref_state, opt_state, control);
  opt_path.poses.clear();
  opt_path.poses.reserve(opt_state.size());
  for (size_t i = 0; i < opt_state.size(); i++) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = opt_state.at(i)(0);
    pose.pose.position.y = opt_state.at(i)(1);
    tf2::Quaternion q;
    q.setRPY(0, 0, opt_state.at(i)(2));
    pose.pose.orientation = tf2::toMsg(q);
    opt_path.poses.emplace_back(std::move(pose));
  }
  return 0;
}
}  // namespace Optimizer
