/*
 * @Author       : dwayne
 * @Date         : 2023-06-26
 * @LastEditTime : 2023-07-02
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#include "sim_ackermann.hpp"

#include <Eigen/src/Core/Matrix.h>

#include <memory>

#include "vehicle_model_bicycle_rear_drive_five_state.h"

namespace sim_robot {

SimAckermann::SimAckermann(std::string name) : Node(name) {
  declareParameter();
  /*get parameters*/
  origin_x_ = get_parameter("origin_x").as_double();
  origin_y_ = get_parameter("origin_y").as_double();
  origin_phi_ = get_parameter("origin_phi").as_double();
  pub_period_ = get_parameter("pub_period").as_double();
  min_sim_time_ = get_parameter("min_sim_time").as_double();
  wheel_base_ = get_parameter("wheel_base").as_double();
  cmd_sub_topic_ = get_parameter("cmd_sub_topic").as_string();
  odom_pub_toipc_ = get_parameter("odom_pub_toipc").as_string();
  state_num_ = get_parameter("state_num").as_int();

  /*initialize robot current status and simulator*/
  current_state_ptr_ = std::make_shared<sim_robot::BicycleKinematics::State>(
      origin_x_, origin_y_, origin_phi_);
  if (state_num_ == 3) {
    vehicle_model_ptr_ =
        std::make_shared<VehicleModelBicycleRearDriveThreeState>(wheel_base_);
  } else if (state_num_ == 5) {
    vehicle_model_ptr_ =
        std::make_shared<VehicleModelBicycleRearDriveFiveState>(wheel_base_);
  }
  Eigen::VectorXd state = Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimX());
  state << origin_x_, origin_y_, origin_phi_;
  sim_robot_ptr_ = std::make_shared<SimRobot>(vehicle_model_ptr_, state);
  current_cmd_ptr_ = std::make_shared<geometry_msgs::msg::Twist>();
  simulator_ptr_ = std::make_shared<sim_robot::BicycleKinematics>(
      wheel_base_, rclcpp::Duration::from_seconds(min_sim_time_));

  /*initialize subscriber and publisher*/
  cmd_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&SimAckermann::cmdCallback, this, std::placeholders::_1));
  acc_cmd_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "acc_cmd_vel", rclcpp::SystemDefaultsQoS(),
      std::bind(&SimAckermann::accCmdCallback, this, std::placeholders::_1));
  lateral_control_subscriber_ =
      create_subscription<mpc_msgs::msg::AckermannLateralCommand>(
          "/mpc_controller/output/control_cmd", rclcpp::SystemDefaultsQoS(),
          std::bind(&SimAckermann::lateralControlCallback, this,
                    std::placeholders::_1));
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "odom", rclcpp::SystemDefaultsQoS());

  current_odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/mpc_controller/input/current_odometry", rclcpp::SystemDefaultsQoS());
  current_accel_publisher_ =
      create_publisher<geometry_msgs::msg::AccelWithCovarianceStamped>(
          "/mpc_controller/input/current_accel", rclcpp::SystemDefaultsQoS());
  current_operation_mode_publisher_ =
      create_publisher<mpc_msgs::msg::OperationModeState>(
          "/mpc_controller/input/current_operation_mode",
          rclcpp::SystemDefaultsQoS());
  current_steering_publisher_ = create_publisher<mpc_msgs::msg::SteeringReport>(
      "/mpc_controller/input/current_steering", rclcpp::SystemDefaultsQoS());

  std::stringstream ss;
  ss << "/home/dwayne/workspace/navigation/nav2_ws/src/navigation/"
        "sim_robot/data/"
     << get_cur_time_str() << ".txt";
  file_.open(ss.str());

  /*initialize timer*/
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(pub_period_));
  pub_timer_ = create_wall_timer(period_ns,
                                 std::bind(&SimAckermann::timerCallback, this));
}

SimAckermann::~SimAckermann() {}

void SimAckermann::accCmdCallback(
    const geometry_msgs::msg::Twist::SharedPtr acc_cmd) {
  if (state_num_ != 5) {
    RCLCPP_WARN(get_logger(), "the number of state is not 5 !");
    return;
  }
}

void SimAckermann::cmdCallback(const geometry_msgs::msg::Twist::SharedPtr cmd) {
  if (state_num_ != 3) {
    RCLCPP_WARN(get_logger(), "the number of state is not 3 !");
    return;
  }
  if (std::abs(cmd->angular.z) > 0.4363)
    cmd->angular.z = cmd->angular.z > 0 ? 0.4363 : -0.4363;

  current_cmd_ptr_ = cmd;
  if (!is_init_) {
    last_time_ = get_clock()->now();
    is_init_ = true;
    return;
  }
  current_time_ = get_clock()->now();
  rclcpp::Duration time_interval = current_time_ - last_time_;
  if (time_interval > rclcpp::Duration::from_seconds(0.2)) {
    RCLCPP_WARN(get_logger(),
                "%fs has passed since the last command was received",
                time_interval.seconds());
    last_time_ = get_clock()->now();
    return;
  }

  Eigen::VectorXd control =
      Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimU());
  control << cmd->linear.x, cmd->angular.z;
  rclcpp::Time before_cal_ = get_clock()->now();
  sim_robot_ptr_->toNextState(control, time_interval.seconds(), 0.01);
  rclcpp::Time after_cal_ = get_clock()->now();
  last_time_ = current_time_;
}

void SimAckermann::lateralControlCallback(
    const mpc_msgs::msg::AckermannLateralCommand::SharedPtr lateral_command) {
  geometry_msgs::msg::Twist::SharedPtr cmd =
      std::make_shared<geometry_msgs::msg::Twist>();
  cmd->linear.x = 0.5;
  cmd->angular.z = lateral_command->steering_tire_angle;
  cmdCallback(cmd);
}

void SimAckermann::timerCallback() {
  // RCLCPP_INFO(get_logger(), "timer callback");
  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.child_frame_id = "odom";
  odom.header.stamp = get_clock()->now();
  odom.pose.pose.position.x = sim_robot_ptr_->getCurState()(0);
  odom.pose.pose.position.y = sim_robot_ptr_->getCurState()(1);
  odom.pose.pose.orientation =
      createQuaternionMsgFromYaw(sim_robot_ptr_->getCurState()(2));
  odom.twist.twist = *current_cmd_ptr_;
  // odom.twist.twist.angular.z = current_cmd_ptr_->linear.x / wheel_base_ *
  //                              tan(current_cmd_ptr_->angular.z);
  odom.twist.twist.angular.z = current_cmd_ptr_->angular.z;
  // RCLCPP_INFO_STREAM(get_logger(),
  //                    "x: " << current_state_ptr_->x_ << " y:" <<
  //                    current_state_ptr_->y_ << " phi:" <<
  //                    current_state_ptr_->phi_);
  odom_publisher_->publish(odom);
  current_odom_publisher_->publish(odom);
  geometry_msgs::msg::AccelWithCovarianceStamped accel;
  current_accel_publisher_->publish(accel);
  mpc_msgs::msg::OperationModeState oms;
  oms.is_autonomous_mode_available = true;
  current_operation_mode_publisher_->publish(oms);
  mpc_msgs::msg::SteeringReport sr;
  sr.steering_tire_angle = current_cmd_ptr_->angular.z;
  current_steering_publisher_->publish(sr);
}

std::string SimAckermann::get_cur_time_str() {
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

geometry_msgs::msg::Quaternion SimAckermann::createQuaternionMsgFromYaw(
    double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void SimAckermann::declareParameter() {
  declare_parameter("origin_x", 0.0);
  declare_parameter("origin_y", 0.0);
  declare_parameter("origin_phi", 0.0);
  declare_parameter("pub_period", 0.001);
  declare_parameter("min_sim_time", 0.001);
  declare_parameter("wheel_base", 0.65);
  declare_parameter("cmd_sub_topic", "cmd_vel");
  declare_parameter("odom_pub_toipc", "odom");
  declare_parameter("stata_num", 5);
}

}  // namespace sim_robot
