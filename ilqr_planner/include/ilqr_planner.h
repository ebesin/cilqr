#pragma once

#include <fstream>
#include <memory>
#include <mpc_msgs/msg/detail/hybrid_lane__struct.hpp>
#include <mpc_msgs/msg/detail/hybrid_trajectory__struct.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/time.hpp>
#include <vector>

#include "ilqr_optimizer.h"
#include "ilqr_path_optimizer.h"
#include "mpc_msgs/msg/hybrid_trajectory.hpp"
#include "mpc_msgs/msg/vehicle_state.hpp"
#include "optimizer_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_model_interface.h"

using namespace Optimizer;
using namespace VehicleModel;

namespace IlqrPlanner {
class IlqrPlanner : public rclcpp::Node {
 public:
  IlqrPlanner(std::string name);
  ~IlqrPlanner() = default;

 private:
  void setWeightMatrix(int direction, bool first = true,
                       bool close_to_end = false);

  void refPathCallback(const nav_msgs::msg::Path::SharedPtr ref_path);
  void refHybridTrajCallback(
      const mpc_msgs::msg::HybridTrajectory::SharedPtr ref_hybrid_traj);
  void curStateCallback(const nav_msgs::msg::Odometry::SharedPtr cur_state);
  void vehicleStateCallback(const mpc_msgs::msg::VehicleState& cur_state);
  void timerCallback();
  void control(const mpc_msgs::msg::HybridLane& ref_lane);
  void declareParameter();
  int getOptStep(nav_msgs::msg::Path ref_path, int cur_idx, double length,
                 double& act_length);

  std::string get_cur_time_str();

  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr ref_path_subscriber_;
  rclcpp::Subscription<mpc_msgs::msg::HybridTrajectory>::SharedPtr
      ref_hybrid_traj_subscriber_;
  rclcpp::Subscription<mpc_msgs::msg::VehicleState>::SharedPtr
      vehicle_state_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr opt_path_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;
  rclcpp::Publisher<mpc_msgs::msg::VehicleState>::SharedPtr
      vehicle_control_publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr
      cur_state_subscriber_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  std::shared_ptr<OptimizerInterface> optimizer_ptr_;
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;
  nav_msgs::msg::Path ref_path_;
  nav_msgs::msg::Path ref_traj_;
  nav_msgs::msg::Path opt_path_;
  mpc_msgs::msg::VehicleState state_;
  mpc_msgs::msg::HybridTrajectory ref_hybrid_traj_;
  mpc_msgs::msg::HybridLane ref_hybrid_lane_;
  bool get_vehicle_state_flag_{false};
  double weight_end_vel_{10.0};
  double forward_first_weight_end_vel_{10.0};
  double forward_second_weight_end_vel_{10.0};
  double backward_first_weight_end_vel_{10.0};
  double backward_second_weight_end_vel_{10.0};
  double arrive_dist_{0.03};

  nav_msgs::msg::Odometry::SharedPtr cur_state_;
  bool get_cur_state_flag_{false};
  bool finish_cur_lane_{true};
  bool close_to_end_{false};
  bool finish_cur_traj_{false};

  int max_opt_points_{40};
  double speed_{0.4};
  double opt_length_{1.6};
  double opt_time_interval_{0.1};
  bool auto_cal_time_interval_{false};
  double auto_time_interval_coefficient_{1.0};
  double wheel_base_{0.65};

  std::ofstream file_;

  std::vector<double> forward_first_weight_intermediate_state_ = {1e0, 1e0, 1e0,
                                                                  1e0, 1e0};
  std::vector<double> forward_first_weight_end_state_{1e3, 1e3, 1e3, 1e3, 1e3};
  std::vector<double> forward_first_weight_control_{1e3, 1e3};

  std::vector<double> forward_second_weight_intermediate_state_ = {
      1e0, 1e0, 1e0, 1e0, 1e0};
  std::vector<double> forward_second_weight_end_state_{1e3, 1e3, 1e3, 1e3, 1e3};
  std::vector<double> forward_second_weight_control_{1e3, 1e3};

  std::vector<double> backward_first_weight_intermediate_state_ = {
      1e2, 1e2, 1.0, 1e0, 1e0};
  std::vector<double> backward_first_weight_end_state_{1e3, 1e3, 1e1, 0.0, 0.0};
  std::vector<double> backward_first_weight_control_{1e1, 1e3};

  std::vector<double> backward_second_weight_intermediate_state_ = {
      1e2, 1e2, 1.0, 1e0, 1e0};
  std::vector<double> backward_second_weight_end_state_{1e3, 1e3, 1e1, 0.0,
                                                        0.0};
  std::vector<double> backward_second_weight_control_{1e1, 1e3};
};
}  // namespace IlqrPlanner
