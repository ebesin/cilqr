#pragma once
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <nav_msgs/msg/detail/path__struct.hpp>

using geometry_msgs::msg::Point;
using geometry_msgs::msg::Pose;
using geometry_msgs::msg::PoseStamped;
using nav_msgs::msg::Path;

void kMatch(const PoseStamped& begin_pose, const PoseStamped& end_pose,
            const double& step, double& k1, double& k2);

bool fixedModePlan(const PoseStamped& begin_pose, const PoseStamped& end_pose,
                   std::vector<PoseStamped>& ctrl_list);

double doBezierPlan(const PoseStamped& begin_pose, const PoseStamped& end_pose,
                    Path& path, const double& step);

double doBezierThreeOrder(const std::vector<PoseStamped>& ctrl_list,
                          const double& step, Path& path);

template <typename T>
void oneBezierFiveOrder(const std::vector<PoseStamped>& ctrl_list, const T& t,
                        PoseStamped& cur_pose, T& cur_curvature);

double doBezierFiveOrder(const std::vector<PoseStamped>& ctrl_list,
                         const double& step, Path& path);
