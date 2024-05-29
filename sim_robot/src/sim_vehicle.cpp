#include "sim_vehicle.h"

#include <eigen3/Eigen/src/Core/Matrix.h>

#include <chrono>
#include <geometry_msgs/msg/detail/twist__struct.hpp>
#include <memory>
#include <mpc_msgs/msg/detail/vehicle_state__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/time.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coor_tools.h"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vehicle_model_bicycle_rear_drive_five_state.h"
#include "vehicle_model_bicycle_rear_drive_four_state.h"
#include "vehicle_model_bicycle_rear_drive_three_state.h"

SimVehicle::SimVehicle(std::string name) : Node(name) {
  declareParameter();
  if (state_num_ == 3) {
    vehicle_model_ptr_ =
        std::make_shared<VehicleModelBicycleRearDriveThreeState>(wheel_base_);
  } else if (state_num_ == 4) {
    vehicle_model_ptr_ =
        std::make_shared<VehicleModelBicycleRearDriveFourState>(wheel_base_);
  } else if (state_num_ == 5) {
    vehicle_model_ptr_ =
        std::make_shared<VehicleModelBicycleRearDriveFiveState>(wheel_base_);
  } else {
    RCLCPP_ERROR(get_logger(), "the state number(which is %d)", state_num_);
    return;
  }
  Eigen::VectorXd state = Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimX());
  state << origin_x_, origin_y_, origin_phi_;
  sim_robot_ptr_ = std::make_shared<SimRobot>(vehicle_model_ptr_, state);
  use_control_ = Eigen::VectorXd::Zero(2);
  cmd_subscriber_ = create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", rclcpp::SystemDefaultsQoS(),
      std ::bind(&SimVehicle::cmdCallback, this, std::placeholders ::_1));
  ctrl_subscriber_ = create_subscription<mpc_msgs::msg::VehicleState>(
      "/vehicle_control", rclcpp::SystemDefaultsQoS(),
      std ::bind(&SimVehicle::ctrlCallback, this, std::placeholders ::_1));
  vehicle_state_publisher_ = create_publisher<mpc_msgs::msg::VehicleState>(
      "/vehicle_state", rclcpp::SystemDefaultsQoS());
  odom_publisher_ = create_publisher<nav_msgs::msg::Odometry>(
      "/odom", rclcpp::SystemDefaultsQoS());
  std::stringstream ss;
  ss << "/home/dwayne/workspace/navigation/nav2_ws/src/navigation/"
        "sim_robot/data/"
     << get_cur_time_str() << ".txt";
  file_.open(ss.str());
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(pub_period_));
  sim_timer_ = create_wall_timer(std::chrono::nanoseconds(period_ns),
                                 std::bind(&SimVehicle::timerCallback, this));
}

void SimVehicle::cmdCallback(const geometry_msgs::msg::Twist& cmd_vel) {
  use_control_ = Eigen::VectorXd::Zero(2);
  Eigen::VectorXd cur_state = sim_robot_ptr_->getCurState();
  cur_state(3) = cmd_vel.linear.x;
  cur_state(4) = cmd_vel.angular.z;
  sim_robot_ptr_->setState(cur_state);
  use_control_ << 0.0, 0.0;
}

void SimVehicle::ctrlCallback(
    const mpc_msgs::msg::VehicleState& state_control) {
  RCLCPP_INFO_STREAM(get_logger(),
                     " acc: " << state_control.acc
                              << " dsteer: " << state_control.dsteer
                              << " direct_speed_control: "
                              << state_control.direct_speed_control);
  reveive_control_ = state_control;
  Eigen::VectorXd u = getUFromControl(state_control);
  Eigen::VectorXd state = sim_robot_ptr_->calNextState(u, state_control.time);
  use_control_ = Eigen::VectorXd::Zero(2);
  if (vehicle_model_ptr_->getDimX() == 3) {
    use_control_ << state_control.vel, state_control.steer;
  } else if (vehicle_model_ptr_->getDimX() == 4) {
    if (reveive_control_.direct_speed_control) {
      Eigen::VectorXd cur_state = sim_robot_ptr_->getCurState();
      cur_state(3) = state_control.vel;
      sim_robot_ptr_->setState(cur_state);
      use_control_ << 0.0, state_control.steer;
    } else {
      Eigen::VectorXd cur_state = sim_robot_ptr_->getCurState();
      cur_state(3) = state(3);
      sim_robot_ptr_->setState(cur_state);
      use_control_ << 0.0, state_control.steer;
    }
  } else if (vehicle_model_ptr_->getDimX() == 5) {
    if (reveive_control_.direct_speed_control) {
      RCLCPP_INFO(get_logger(), "receive direct speed control");
      Eigen::VectorXd cur_state = sim_robot_ptr_->getCurState();
      cur_state(3) = state_control.vel;
      cur_state(4) = state_control.steer;
      sim_robot_ptr_->setState(cur_state);
      use_control_ << 0.0, 0.0;
    } else {
      Eigen::VectorXd cur_state = sim_robot_ptr_->getCurState();
      cur_state(3) = state(3);
      cur_state(4) = state(4);
      sim_robot_ptr_->setState(cur_state);
      use_control_ << 0.0, 0.0;
    }
    file_ << std::fixed << std::setprecision(5) << state[0] << " " << std::fixed
          << std::setprecision(5) << state[1] << " " << std::fixed
          << std::setprecision(5) << state[2] << " " << std::fixed
          << std::setprecision(5) << state[3] << " " << std::fixed
          << std::setprecision(5) << state[4] << " " << std::fixed
          << std::setprecision(5) << u[0] << " " << std::fixed
          << std::setprecision(5) << u[1] << std::endl;
  }
}

void SimVehicle::timerCallback() {
  if (!is_init_) {
    last_sim_time_ = get_clock()->now();
    is_init_ = true;
    return;
  }
  rclcpp::Time cur_time = get_clock()->now();
  rclcpp::Time last_ctrl_time = reveive_control_.header.stamp;
  double since_last_ctrl_time = (cur_time - last_ctrl_time).seconds();

  double time_diff = (cur_time - last_sim_time_).seconds();
  if (since_last_ctrl_time > 0.2) {
    RCLCPP_WARN(get_logger(),
                "%f has passed since last control,skip simulation",
                since_last_ctrl_time);
  } else {
    sim_robot_ptr_->toNextState(use_control_, time_diff, 0.001);
  }
  mpc_msgs::msg::VehicleState state =
      getMsgFromState(sim_robot_ptr_->getCurState());
  last_sim_time_ = cur_time;
  state.header.stamp = get_clock()->now();
  vehicle_state_publisher_->publish(state);

  nav_msgs::msg::Odometry odom;
  odom.header.frame_id = "map";
  odom.child_frame_id = "odom";
  odom.header.stamp = get_clock()->now();
  odom.pose.pose.position.x = sim_robot_ptr_->getCurState()(0);
  odom.pose.pose.position.y = sim_robot_ptr_->getCurState()(1);
  odom.pose.pose.orientation =
      createQuaternionMsgFromYaw(sim_robot_ptr_->getCurState()(2));
  if (vehicle_model_ptr_->getDimX() == 3) {
    odom.twist.twist.linear.x = use_control_(0);
    odom.twist.twist.angular.z = use_control_(1);
  } else if (vehicle_model_ptr_->getDimX() == 4) {
    odom.twist.twist.linear.x = sim_robot_ptr_->getCurState()(3);
    odom.twist.twist.angular.z = use_control_(1);
  } else if (vehicle_model_ptr_->getDimX() == 5) {
    odom.twist.twist.linear.x = sim_robot_ptr_->getCurState()(3);
    odom.twist.twist.angular.z = sim_robot_ptr_->getCurState()(4);
  }
  odom_publisher_->publish(odom);
}

Eigen::VectorXd SimVehicle::getUFromControl(
    const mpc_msgs::msg::VehicleState& control) {
  Eigen::VectorXd u = Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimU());
  if (vehicle_model_ptr_->getDimX() == 3) {
    u << control.vel, control.steer;
  } else if (vehicle_model_ptr_->getDimX() == 4) {
    u << control.acc, control.steer;
  } else if (vehicle_model_ptr_->getDimX() == 5) {
    u << control.acc, control.dsteer;
  }
  return u;
}

Eigen::VectorXd SimVehicle::getStateFromMsg(
    const mpc_msgs::msg::VehicleState& msg) {
  Eigen::VectorXd state = Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimX());
  if (vehicle_model_ptr_->getDimX() == 3) {
    state << msg.x, msg.y, msg.theta;
  } else if (vehicle_model_ptr_->getDimX() == 4) {
    state << msg.x, msg.y, msg.theta, msg.vel;
  } else if (vehicle_model_ptr_->getDimX() == 5) {
    state << msg.x, msg.y, msg.theta, msg.acc, msg.dsteer;
  }
  return state;
}

std::string SimVehicle::get_cur_time_str() {
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

mpc_msgs::msg::VehicleState SimVehicle::getMsgFromState(
    const Eigen::VectorXd& state) {
  mpc_msgs::msg::VehicleState state_msg;
  if (state.size() < 3) {
    RCLCPP_ERROR(get_logger(),
                 "the size of the state vector(which is %td) is less than 3",
                 state.size());
  }
  if (state.size() != vehicle_model_ptr_->getDimX()) {
    RCLCPP_ERROR(get_logger(),
                 "the size of the state vector(which is %td) is not equal to "
                 "the number of "
                 "the vehicle model(which is %d)",
                 state.size(), vehicle_model_ptr_->getDimX());
  }
  state_msg.x = state(0);
  state_msg.y = state(1);
  state_msg.theta = state(2);
  if (vehicle_model_ptr_->getDimX() == 4) {
    state_msg.vel = state(3);
  } else if (vehicle_model_ptr_->getDimX() == 5) {
    state_msg.vel = state(3);
    state_msg.steer = state(4);
  }
  return state_msg;
}

void SimVehicle::setCurState(const Eigen::VectorXd& state) {
  if (state.size() < 3) {
    RCLCPP_ERROR(get_logger(),
                 "the size of the state vector(which is %td) is less than 3",
                 state.size());
    return;
  }
  if (state.size() != vehicle_model_ptr_->getDimX()) {
    RCLCPP_ERROR(get_logger(),
                 "the size of the state vector(which is %td) is not equal to "
                 "the number of "
                 "the vehicle model(which is %d)",
                 state.size(), vehicle_model_ptr_->getDimX());
    return;
  }
  cur_state_.x = state(0);
  cur_state_.y = state(1);
  cur_state_.theta = state(2);
  if (vehicle_model_ptr_->getDimX() == 4) {
    cur_state_.vel = state(3);
  } else if (vehicle_model_ptr_->getDimX() == 5) {
    cur_state_.vel = state(3);
    cur_state_.steer = state(4);
  }
}

geometry_msgs::msg::Quaternion SimVehicle::createQuaternionMsgFromYaw(
    double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

void SimVehicle::declareParameter() {
  declare_parameter("origin_x", 0.0);
  declare_parameter("origin_y", 0.0);
  declare_parameter("origin_phi", 0.0);
  declare_parameter("pub_period", 0.005);
  declare_parameter("min_sim_time", 0.001);
  declare_parameter("wheel_base", 0.65);
  declare_parameter("cmd_sub_topic", "cmd_vel");
  declare_parameter("odom_pub_toipc", "odom");
  declare_parameter("state_num", 5);

  /*get parameters*/
  origin_x_ = get_parameter("origin_x").as_double();
  origin_y_ = get_parameter("origin_y").as_double();
  origin_phi_ = utils_tool::deg2Rad(get_parameter("origin_phi").as_double());
  pub_period_ = get_parameter("pub_period").as_double();
  min_sim_time_ = get_parameter("min_sim_time").as_double();
  wheel_base_ = get_parameter("wheel_base").as_double();
  cmd_sub_topic_ = get_parameter("cmd_sub_topic").as_string();
  odom_pub_toipc_ = get_parameter("odom_pub_toipc").as_string();
  state_num_ = get_parameter("state_num").as_int();
}
