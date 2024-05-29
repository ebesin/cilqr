#pragma once

#include <vector>

#include "nav_msgs/msg/path.hpp"
#include "vehicle_model_interface.h"
#include "vehicle_state_ackermann.h"
#include "vehicle_state_diff.h"
#include "vehicle_state_interface.h"

using namespace VehicleState;
using namespace VehicleModel;

namespace OptimizerUtils {

void getNearPose(const std::shared_ptr<VehicleModelInterface>&,
                 const nav_msgs::msg::Path&,
                 geometry_msgs::msg::Pose& near_pose, size_t& idx,
                 double& dist);

/**
 * @brief 计算使用匀加速及匀减速过程行驶一段路径的大致时间
 */
int calApproximateTime(std::shared_ptr<VehicleModelInterface>&,
                       const double& total_dist, double& approximate_time);

int reSampleTrajByTime(std::shared_ptr<VehicleModelInterface>&,
                       const nav_msgs::msg::Path&, nav_msgs::msg::Path&);

int reCalculateTimeInterval(
    const std::shared_ptr<VehicleModelInterface>& vehicle_model_ptr,
    const nav_msgs::msg::Path& ref_path, bool, size_t& point_index,
    double& time_interval);

void convertToMsg(const std::vector<Eigen::VectorXd>& state,
                  nav_msgs::msg::Path& path);

void convertToStateVec(const nav_msgs::msg::Path& path,
                       std::vector<Eigen::VectorXd>& state);

void pathSegmentation(const nav_msgs::msg::Path& path, int unit_size,
                      std::vector<nav_msgs::msg::Path>& paths);
}  // namespace OptimizerUtils
