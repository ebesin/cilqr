#pragma once

#include <fstream>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <mpc_msgs/msg/detail/hybrid_lane__struct.hpp>
#include <mpc_msgs/msg/detail/hybrid_trajectory__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <string>
#include <vector>

#include "cilqr_solver.hpp"
#include "ilqr_optimizer.h"
#include "ilqr_path_optimizer.h"
#include "mpc_msgs/msg/hybrid_trajectory.hpp"
#include "mpc_msgs/msg/vehicle_state.hpp"
#include "optimizer_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_model_interface.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using namespace CIlqrSolver;

class CIlqrPlanner : public rclcpp::Node {
 public:
  CIlqrPlanner(std::string name);
  ~CIlqrPlanner() = default;

 private:
  bool SolveProblem(const geometry_msgs::msg::Pose& cur_pose,
                    const nav_msgs::msg::Path& ref_path,
                    const std::vector<Obstacle>& obstacles,
                    nav_msgs::msg::Path& opt_path);

  void timerCallback();
  void declareParameter();
  std::string get_cur_time_str();

  void publishVehiclePath(const nav_msgs::msg::Path& path, double width,
                          double length, int interval);

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ref_path_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_publisher_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      vehicle_path_publisher_;
  std::ofstream file_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  geometry_msgs::msg::Pose back_center_, front_center_;
  double radius_;
  double cur_speed_;

  double scaleFactor{1.0};
  double obstacleFactor{1.0};
  double rotCost{0.4};
  double w_Q{50.0};
  double w_R{1.0};
  double wheel_base{0.5};

  double begin_pose_x;
  double begin_pose_y;
  double begin_pose_theta;
  double end_pose_x;
  double end_pose_y;
  double end_pose_theta;
  double obs_radius;
  double first_obs_pose_x;
  double first_obs_pose_y;
  double second_obs_pose_x;
  double second_obs_pose_y;
  double third_obs_pose_x;
  double third_obs_pose_y;

  bool has_exec{false};
};
