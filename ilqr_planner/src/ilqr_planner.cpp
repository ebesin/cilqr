#include "ilqr_planner.h"

#include <cmath>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <mpc_msgs/msg/detail/hybrid_lane__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <utility>

#include "coor_tools.h"
#include "ilqr_optimizer.h"
#include "ilqr_path_optimizer.h"
#include "nav_msgs/msg/path.hpp"
#include "optimizer_utils.h"
#include "vehicle_model_bicycle_rear_drive_five_state.h"
#include "vehicle_model_bicycle_rear_drive_three_state.h"
#include "vehicle_state_ackermann.h"
#include "vehicle_state_interface.h"

namespace IlqrPlanner {
IlqrPlanner::IlqrPlanner(std::string name) : rclcpp::Node(name) {
  RCLCPP_INFO_STREAM(get_logger(), "start ilqrplanner.... name: " << name);
  declareParameter();
  vehicle_model_ptr_ =
      std::make_shared<VehicleModelBicycleRearDriveFiveState>(wheel_base_);

  Eigen::MatrixXd Q, Q_end, R;
  optimizer_ptr_ =
      std::make_shared<IlqrOptimizer>(vehicle_model_ptr_, Q, Q_end, R);
  RCLCPP_INFO(get_logger(), "create IlqrOptimizer finished");

  ref_path_subscriber_ = create_subscription<nav_msgs::msg::Path>(
      "/generate_path", rclcpp::SystemDefaultsQoS(),
      std::bind(&IlqrPlanner::refPathCallback, this, std::placeholders::_1));
  ref_hybrid_traj_subscriber_ =
      create_subscription<mpc_msgs::msg::HybridTrajectory>(
          "/hybrid_traj", rclcpp::SystemDefaultsQoS(),
          std::bind(&IlqrPlanner::refHybridTrajCallback, this,
                    std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "create ref_path_subscriber_ finished");
  opt_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/opt_path", rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(get_logger(), "create opt_path_publisher_ finished");
  twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(get_logger(), "create twist_publisher_ finished");
  vehicle_control_publisher_ = create_publisher<mpc_msgs::msg::VehicleState>(
      "/vehicle_control", rclcpp::SystemDefaultsQoS());
  RCLCPP_INFO(get_logger(), "create vehicle_control_publisher_ finished");
  cur_state_subscriber_ = create_subscription<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SystemDefaultsQoS(),
      std::bind(&IlqrPlanner::curStateCallback, this, std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "create cur_state_subscriber_ finished");
  vehicle_state_subscriber_ = create_subscription<mpc_msgs::msg::VehicleState>(
      "/vehicle_state", rclcpp::SystemDefaultsQoS(),
      std::bind(&IlqrPlanner::vehicleStateCallback, this,
                std::placeholders::_1));
  RCLCPP_INFO(get_logger(), "create vehicle_state_subscriber_ finished");

  RCLCPP_INFO(get_logger(), "before create timer");
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(0.05));
  control_timer_ = create_wall_timer(
      period_ns, std::bind(&IlqrPlanner::timerCallback, this));

  RCLCPP_INFO(get_logger(), "start ilqrplanner successfully....");
  ref_traj_.poses.clear();
  std::stringstream ss;
  ss << "/home/dwayne/workspace/navigation/nav2_ws/src/navigation/"
        "ilqr_planner/data/"
     << get_cur_time_str() << ".txt";
  file_.open(ss.str());
}

void IlqrPlanner::control(const mpc_msgs::msg::HybridLane& ref_lane) {
  if (close_to_end_) {
    mpc_msgs::msg::VehicleState vehicle_control;
    vehicle_control.direct_speed_control = true;
    vehicle_control.vel = 0.0;
    vehicle_control.steer = 0.0;
    vehicle_control.header.stamp = get_clock()->now();
    vehicle_control.time = opt_time_interval_;
    vehicle_control_publisher_->publish(vehicle_control);
    RCLCPP_INFO(get_logger(), "control finish, skip..");
    finish_cur_lane_ = true;
    return;
  }
  ref_path_.poses.clear();
  if (ref_lane.path.poses.empty()) {
    RCLCPP_WARN(get_logger(), "ref_path is empty!");
    return;
  }
  if (!get_vehicle_state_flag_) {
    RCLCPP_WARN(get_logger(), "haven't get cur state, skip optimize! ");
    return;
  }
  Eigen::VectorXd cur_state =
      Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimX());
  cur_state << state_.x, state_.y, state_.theta, state_.vel, state_.steer;
  vehicle_model_ptr_->setCurStateVec(cur_state);
  geometry_msgs::msg::Pose near_pose;
  size_t point_idx;
  double dist;
  OptimizerUtils::getNearPose(vehicle_model_ptr_, ref_lane.path, near_pose,
                              point_idx, dist);
  geometry_msgs::msg::PoseStamped cur_pose;
  cur_pose.pose.position.x = cur_state.x();
  cur_pose.pose.position.y = cur_state.y();
  cur_pose.pose.orientation =
      utils_tool::createQuaternionMsgFromYaw(cur_state.z());
  geometry_msgs::msg::PoseStamped ref_pose;
  ref_pose.pose = near_pose;
  auto diff = utils_tool::absoluteDifference(cur_pose, ref_pose);
  file_ << ref_pose << " " << cur_pose << " " << diff << std::endl;
  double length;
  auto opt_points_num =
      getOptStep(ref_lane.path, point_idx, opt_length_, length);
  double dt = length / speed_ / opt_points_num;
  for (size_t i = 0;
       i < opt_points_num && i + point_idx < ref_lane.path.poses.size(); i++) {
    ref_path_.poses.emplace_back(
        std::move(ref_lane.path.poses.at(i + point_idx)));
  }

  bool close_to_end =
      static_cast<int>(ref_path_.poses.size()) < max_opt_points_;
  RCLCPP_INFO_STREAM(get_logger(),
                     "total_ref_traj_size: " << ref_lane.path.poses.size()
                                             << " point_idx: " << point_idx
                                             << "  use_ref_path_.poses.size(): "
                                             << ref_path_.poses.size());

  std::shared_ptr<VehicleStateInterface> end_state =
      std::make_shared<VehicleStateAckermann>();
  end_state->base_state_.pose.pose = ref_path_.poses.back().pose;
  end_state->base_state_.twist.twist.linear.x = 0.0;
  vehicle_model_ptr_->setEndState(end_state);
  std::shared_ptr<VehicleStateAckermann> max_state_ptr =
      std::make_shared<VehicleStateAckermann>();
  max_state_ptr->base_state_.twist.twist.linear.x = 2.0;
  max_state_ptr->acc_vx_ = 4.0;
  vehicle_model_ptr_->setMaxState(max_state_ptr);
  std::dynamic_pointer_cast<IlqrOptimizer>(optimizer_ptr_)->setTimeInterval(dt);
  // first opt
  std::vector<Eigen::VectorXd> ref_state;
  std::vector<Eigen::VectorXd> first_optimized_state;
  std::vector<Eigen::VectorXd> first_control;
  OptimizerUtils::convertToStateVec(ref_path_, ref_state);
  if (ref_path_.poses.size() < 3) {
    close_to_end_ = true;
    return;
  }
  setWeightMatrix(ref_lane.direction.direction, true, close_to_end);
  optimizer_ptr_->optimize(ref_state, first_optimized_state, first_control);
  // second opt
  if (close_to_end) {
    for (auto& s : first_optimized_state) {
      if (s.size() > 3) {
        s(3) = 0;
      }
      if (s.size() > 4) {
        s(4) = 0;
      }
    }
  }
  // std::vector<Eigen::VectorXd> optimized_state;
  // std::vector<Eigen::VectorXd> control;
  // setWeightMatrix(ref_lane.direction.direction, false, close_to_end);
  // optimizer_ptr_->optimize(first_optimized_state, optimized_state, control);
  opt_path_.header.stamp = get_clock()->now();
  opt_path_.header.frame_id = "map";
  OptimizerUtils::convertToMsg(first_optimized_state, opt_path_);
  mpc_msgs::msg::VehicleState vehicle_control;
  vehicle_control.direct_speed_control = false;
  vehicle_control.acc = first_control.front()(0);
  vehicle_control.dsteer = first_control.front()(1);
  vehicle_control.header.stamp = get_clock()->now();
  vehicle_control.time = opt_time_interval_;
  vehicle_control_publisher_->publish(vehicle_control);
  opt_path_publisher_->publish(opt_path_);
}

int IlqrPlanner::getOptStep(nav_msgs::msg::Path ref_path, int cur_idx,
                            double exp_length, double& act_length) {
  double sum_dist = 0;
  int i = cur_idx;
  for (i = cur_idx; i < ref_path.poses.size() - 1; i++) {
    sum_dist += utils_tool::mod(ref_path.poses.at(cur_idx) -
                                ref_path.poses.at(cur_idx + 1));
    if (sum_dist > exp_length) {
      break;
    }
  }
  act_length = sum_dist;
  return i - cur_idx;
}

void IlqrPlanner::timerCallback() {
  if (finish_cur_lane_ && state_.vel == 0) {
    if (!ref_hybrid_traj_.trajectory.empty()) {
      ref_hybrid_lane_ = ref_hybrid_traj_.trajectory.front();
      finish_cur_lane_ = false;
      close_to_end_ = false;
      ref_hybrid_traj_.trajectory.erase(ref_hybrid_traj_.trajectory.begin());
      RCLCPP_INFO(get_logger(), "update to new lane");
    } else {
      finish_cur_traj_ = true;
      RCLCPP_INFO(get_logger(), "traj follow finished");
      return;
    }
  }
  control(ref_hybrid_lane_);
}

void IlqrPlanner::refPathCallback(
    const nav_msgs::msg::Path::SharedPtr ref_path) {
  ref_traj_.poses.clear();
  ref_traj_.poses.insert(ref_traj_.poses.end(), ref_path->poses.begin(),
                         ref_path->poses.end());
  // ref_traj_.poses.insert(ref_traj_.poses.end(), ref_path->poses.begin(),
  //                        ref_path->poses.begin() + 400);
  // finish_cur_lane_ = false;
  // ref_traj_.poses = ref_path->poses;
}

void IlqrPlanner::refHybridTrajCallback(
    const mpc_msgs::msg::HybridTrajectory::SharedPtr ref_hybrid_traj) {
  if (!finish_cur_traj_ && !ref_hybrid_traj->trajectory.empty()) {
    RCLCPP_WARN(get_logger(),
                "cur traj haven't finish, do not follow new traj");
    return;
  }
  ref_hybrid_traj_.trajectory = ref_hybrid_traj->trajectory;
  std::cout << "traj size: " << ref_hybrid_traj_.trajectory.size() << std::endl;
  finish_cur_traj_ = false;

  RCLCPP_INFO(get_logger(), "get new traj");
}

void IlqrPlanner::curStateCallback(
    const nav_msgs::msg::Odometry::SharedPtr cur_state) {
  if (!get_cur_state_flag_) {
    get_cur_state_flag_ = true;
  }

  cur_state_ = cur_state;
  if (!ref_hybrid_lane_.path.poses.empty()) {
    double dis_2_end = std::hypot(
        state_.x - ref_hybrid_lane_.path.poses.back().pose.position.x,
        state_.y - ref_hybrid_lane_.path.poses.back().pose.position.y);
    std::cout << "dis_2_end: " << dis_2_end << std::endl;
    if (dis_2_end < arrive_dist_) {
      close_to_end_ = true;
    }
  }
}

std::string IlqrPlanner::get_cur_time_str() {
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

void IlqrPlanner::vehicleStateCallback(
    const mpc_msgs::msg::VehicleState& cur_state) {
  if (!get_vehicle_state_flag_) {
    get_vehicle_state_flag_ = true;
  }
  state_ = cur_state;
}

void IlqrPlanner::setWeightMatrix(int direction, bool first,
                                  bool close_to_end) {
  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                                            vehicle_model_ptr_->getDimX());

  Eigen::MatrixXd Q_end = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                                                vehicle_model_ptr_->getDimX());

  Eigen::MatrixXd R = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimU(),
                                            vehicle_model_ptr_->getDimU());
  if (direction == mpc_msgs::msg::Direction::BACKWARD && first) {
    for (int i = 0; i < vehicle_model_ptr_->getDimX(); i++) {
      Q(i, i) = backward_first_weight_intermediate_state_.at(i);
      Q_end(i, i) = backward_first_weight_end_state_.at(i);
    }
    for (int i = 0; i < vehicle_model_ptr_->getDimU(); i++) {
      R(i, i) = backward_first_weight_control_.at(i);
    }
    if (close_to_end) {
      Q_end(3, 3) = backward_first_weight_end_vel_;
    }
  } else if (direction == mpc_msgs::msg::Direction::BACKWARD && !first) {
    for (int i = 0; i < vehicle_model_ptr_->getDimX(); i++) {
      Q(i, i) = backward_second_weight_intermediate_state_.at(i);
      Q_end(i, i) = backward_second_weight_end_state_.at(i);
    }
    for (int i = 0; i < vehicle_model_ptr_->getDimU(); i++) {
      R(i, i) = backward_second_weight_control_.at(i);
    }
    if (close_to_end) {
      Q_end(3, 3) = backward_second_weight_end_vel_;
    }
  } else if (direction == mpc_msgs::msg::Direction::FORWARD && !first) {
    for (int i = 0; i < vehicle_model_ptr_->getDimX(); i++) {
      Q(i, i) = forward_second_weight_intermediate_state_.at(i);
      Q_end(i, i) = forward_second_weight_end_state_.at(i);
    }
    for (int i = 0; i < vehicle_model_ptr_->getDimU(); i++) {
      R(i, i) = forward_second_weight_control_.at(i);
    }
    if (close_to_end) {
      Q_end(3, 3) = forward_second_weight_end_vel_;
    }
  } else {
    for (int i = 0; i < vehicle_model_ptr_->getDimX(); i++) {
      Q(i, i) = forward_first_weight_intermediate_state_.at(i);
      Q_end(i, i) = forward_first_weight_end_state_.at(i);
    }
    for (int i = 0; i < vehicle_model_ptr_->getDimU(); i++) {
      R(i, i) = forward_first_weight_control_.at(i);
    }
    if (close_to_end) {
      Q_end(3, 3) = forward_first_weight_end_vel_;
    }
  }
  std::cout << "vel weight: " << Q_end(3, 3) << std::endl;
  std::dynamic_pointer_cast<IlqrOptimizer>(optimizer_ptr_)
      ->setWightMatrix(Q, Q_end, R);
}

void IlqrPlanner::declareParameter() {
  declare_parameter("max_opt_points", 40);
  declare_parameter("speed", 0.4);
  declare_parameter("opt_length", 1.6);
  declare_parameter("opt_time_interval", 0.1);
  declare_parameter("auto_cal_time_interval", false);
  declare_parameter("auto_time_interval_coefficient", 1.0);
  declare_parameter("arrive_dist", 0.03);

  declare_parameter("forward_first_weight_intermediate_state",
                    forward_first_weight_intermediate_state_);
  declare_parameter("forward_first_weight_end_state",
                    forward_first_weight_end_state_);
  declare_parameter("forward_first_weight_control",
                    forward_first_weight_control_);

  declare_parameter("forward_second_weight_intermediate_state",
                    forward_second_weight_intermediate_state_);
  declare_parameter("forward_second_weight_end_state",
                    forward_second_weight_end_state_);
  declare_parameter("forward_second_weight_control",
                    forward_second_weight_control_);

  declare_parameter("backward_first_weight_intermediate_state",
                    backward_first_weight_intermediate_state_);
  declare_parameter("backward_first_weight_end_state",
                    backward_first_weight_end_state_);
  declare_parameter("backward_first_weight_control",
                    backward_first_weight_control_);

  declare_parameter("backward_second_weight_intermediate_state",
                    backward_second_weight_intermediate_state_);
  declare_parameter("backward_second_weight_end_state",
                    backward_second_weight_end_state_);
  declare_parameter("backward_second_weight_control",
                    backward_second_weight_control_);

  declare_parameter("wheel_base", 0.65);
  declare_parameter("forward_first_weight_end_vel", 10.0);
  declare_parameter("forward_second_weight_end_vel", 10.0);
  declare_parameter("backward_first_weight_end_vel", 10.0);
  declare_parameter("backward_second_weight_end_vel", 10.0);

  speed_ = get_parameter("speed").as_double();
  opt_length_ = get_parameter("opt_length").as_double();
  max_opt_points_ = get_parameter("max_opt_points").as_int();
  opt_time_interval_ = get_parameter("opt_time_interval").as_double();
  auto_cal_time_interval_ = get_parameter("auto_cal_time_interval").as_bool();
  auto_time_interval_coefficient_ =
      get_parameter("auto_time_interval_coefficient").as_double();
  arrive_dist_ = get_parameter("arrive_dist").as_double();

  forward_first_weight_intermediate_state_ =
      get_parameter("forward_first_weight_intermediate_state")
          .as_double_array();
  forward_first_weight_end_state_ =
      get_parameter("forward_first_weight_end_state").as_double_array();
  forward_first_weight_control_ =
      get_parameter("forward_first_weight_control").as_double_array();

  forward_second_weight_intermediate_state_ =
      get_parameter("forward_second_weight_intermediate_state")
          .as_double_array();
  forward_second_weight_end_state_ =
      get_parameter("forward_second_weight_end_state").as_double_array();
  forward_second_weight_control_ =
      get_parameter("forward_second_weight_control").as_double_array();

  backward_first_weight_intermediate_state_ =
      get_parameter("backward_first_weight_intermediate_state")
          .as_double_array();
  backward_first_weight_end_state_ =
      get_parameter("backward_first_weight_end_state").as_double_array();
  backward_first_weight_control_ =
      get_parameter("backward_first_weight_control").as_double_array();

  backward_second_weight_intermediate_state_ =
      get_parameter("backward_second_weight_intermediate_state")
          .as_double_array();
  backward_second_weight_end_state_ =
      get_parameter("backward_second_weight_end_state").as_double_array();
  backward_second_weight_control_ =
      get_parameter("backward_second_weight_control").as_double_array();

  forward_first_weight_end_vel_ =
      get_parameter("forward_first_weight_end_vel").as_double();
  forward_second_weight_end_vel_ =
      get_parameter("forward_second_weight_end_vel").as_double();

  backward_first_weight_end_vel_ =
      get_parameter("backward_first_weight_end_vel").as_double();
  backward_second_weight_end_vel_ =
      get_parameter("backward_second_weight_end_vel").as_double();

  wheel_base_ = get_parameter("wheel_base").as_double();

  RCLCPP_INFO(get_logger(), "declareParameter finished");
}
}  // namespace IlqrPlanner
