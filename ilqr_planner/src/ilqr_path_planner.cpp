#include "ilqr_path_planner.h"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <memory>
#include <nav_msgs/msg/path.hpp>

#include "coor_tools.h"
#include "ilqr_optimizer.h"
#include "ilqr_path_optimizer.h"
#include "optimizer_utils.h"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace Optimizer {

IlqrPathPlanner::IlqrPathPlanner(
    const std::shared_ptr<VehicleModel::VehicleModelInterface>
        &vehicle_model_ptr,
    const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Q_end,
    const Eigen::MatrixXd &R)
    : vehicle_model_ptr_(vehicle_model_ptr), Q_(Q), Q_end_(Q_end), R_(R) {
  planner_ptr_ =
      std::make_shared<IlqrOptimizer>(vehicle_model_ptr_, Q_, Q_end, R);
}

int IlqrPathPlanner::doPlan(const geometry_msgs::msg::Pose &begin_pose,
                            const geometry_msgs::msg::Pose &end_pose,
                            nav_msgs::msg::Path &path) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                                            vehicle_model_ptr_->getDimX());
  planner_ptr_->setWightMatrix(Q, Q_end_, R_);
  Eigen::VectorXd cur_state =
      Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimX());
  cur_state << begin_pose.position.x, begin_pose.position.y,
      tf2::getYaw(begin_pose.orientation);
  vehicle_model_ptr_->setCurStateVec(cur_state);

  nav_msgs::msg::Path ref_path;
  ref_path.poses.resize(50);
  //   ref_path.poses.front().pose = begin_pose;
  ref_path.poses.back().pose = end_pose;
  std::cout << "ref_path: " << std::endl << ref_path << std::endl;
  std::vector<Eigen::VectorXd> ref_state;
  std::vector<Eigen::VectorXd> optimized_state;
  std::vector<Eigen::VectorXd> control;
  OptimizerUtils::convertToStateVec(ref_path, ref_state);
  planner_ptr_->setTimeInterval(0.1);
  planner_ptr_->optimize(ref_state, optimized_state, control);

  OptimizerUtils::convertToMsg(optimized_state, path);
  return 0;
}

int IlqrPathPlanner::doPlan(const nav_msgs::msg::Path &ref_path,
                            nav_msgs::msg::Path &opt_path) {
  planner_ptr_->setWightMatrix(Q_, Q_end_, R_);
  Eigen::VectorXd cur_state =
      Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimX());
  cur_state << ref_path.poses.front().pose.position.x,
      ref_path.poses.front().pose.position.y,
      tf2::getYaw(ref_path.poses.front().pose.orientation);
  vehicle_model_ptr_->setCurStateVec(cur_state);

  std::vector<Eigen::VectorXd> ref_state;
  std::vector<Eigen::VectorXd> optimized_state;
  std::vector<Eigen::VectorXd> control;
  OptimizerUtils::convertToStateVec(ref_path, ref_state);
  planner_ptr_->setTimeInterval(0.1);
  planner_ptr_->optimize(ref_state, optimized_state, control);

  OptimizerUtils::convertToMsg(optimized_state, opt_path);
  return 0;
}
}  // namespace Optimizer
