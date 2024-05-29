#include "cilqr_planner.hpp"

#include <cmath>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <mpc_msgs/msg/detail/hybrid_lane__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <utility>

#include "cilqr_solver.hpp"
#include "coor_tools.h"
#include "ilqr_optimizer.h"
#include "ilqr_path_optimizer.h"
#include "nav_msgs/msg/path.hpp"
#include "optimizer_utils.h"
#include "vehicle_model_bicycle_rear_drive_five_state.h"
#include "vehicle_model_bicycle_rear_drive_three_state.h"
#include "vehicle_state_ackermann.h"
#include "vehicle_state_interface.h"

CIlqrPlanner::CIlqrPlanner(std::string name) : rclcpp::Node(name) {
  RCLCPP_INFO_STREAM(get_logger(), "start ilqrplanner.... name: " << name);
  back_center_.position.x = 0.0;
  back_center_.position.y = 0.0;
  front_center_.position.x = 0.6;
  front_center_.position.y = 0.0;
  radius_ = 0.1;
  cur_speed_ = 0.2;

  declareParameter();
  opt_path_publisher_ = create_publisher<nav_msgs::msg::Path>(
      "/opt_path", rclcpp::SystemDefaultsQoS());
  vehicle_path_publisher_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("vehicle_path", 1);

  std::stringstream ss;
  ss << "/home/dwayne/workspace/navigation/nav2_ws/src/navigation/"
        "ilqr_planner/cilqr/"
     << get_cur_time_str() << ".txt";
  file_.open(ss.str());
  const auto period_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::duration<double>(0.1));
  control_timer_ = create_wall_timer(
      period_ns, std::bind(&CIlqrPlanner::timerCallback, this));
}

void CIlqrPlanner::timerCallback() {
  if (has_exec) {
    return;
  }
  geometry_msgs::msg::Pose cur_pose;
  cur_pose.position.x = begin_pose_x;
  cur_pose.position.y = begin_pose_y;
  cur_pose.orientation =
      utils_tool::createQuaternionMsgFromYaw(begin_pose_theta);
  nav_msgs::msg::Path ref_path;
  geometry_msgs::msg::PoseStamped final_pose;
  final_pose.pose.position.x = end_pose_x;
  final_pose.pose.position.y = end_pose_y;
  final_pose.pose.orientation =
      utils_tool::createQuaternionMsgFromYaw(end_pose_theta);
  ref_path.poses.emplace_back(std::move(final_pose));
  std::vector<Obstacle> obstacles;
  Obstacle obs1;
  obs1.radius = obs_radius;
  obs1.pos[0] = first_obs_pose_x;
  obs1.pos[1] = first_obs_pose_y;
  Obstacle obs2;
  obs2.radius = obs_radius;
  obs2.pos[0] = second_obs_pose_x;
  obs2.pos[1] = second_obs_pose_y;
  Obstacle obs3;
  obs3.radius = obs_radius;
  obs3.pos[0] = third_obs_pose_x;
  obs3.pos[1] = third_obs_pose_y;
  obstacles.push_back(obs1);
  obstacles.push_back(obs2);
  obstacles.push_back(obs3);
  nav_msgs::msg::Path opt_path;
  auto now = std::chrono::system_clock::now();
  auto before = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now.time_since_epoch())
                    .count();
  if (SolveProblem(cur_pose, ref_path, obstacles, opt_path)) {
    opt_path.header.stamp = get_clock()->now();
    opt_path.header.frame_id = "map";
    opt_path_publisher_->publish(opt_path);
    publishVehiclePath(opt_path, 0.75, 1.0, 2);
    RCLCPP_INFO(get_logger(), "solve success");
    RCLCPP_INFO_STREAM(get_logger(),
                       "opt_path_size:" << opt_path.poses.size()
                                        << "  last: " << opt_path.poses.back());
  } else {
    RCLCPP_INFO(get_logger(), "fail to solve prob....");
  }
  now = std::chrono::system_clock::now();
  auto after = std::chrono::duration_cast<std::chrono::milliseconds>(
                   now.time_since_epoch())
                   .count();
  RCLCPP_INFO_STREAM(get_logger(), "solve time: " << after - before);
  has_exec = true;
}

bool CIlqrPlanner::SolveProblem(const geometry_msgs::msg::Pose& cur_pose,
                                const nav_msgs::msg::Path& ref_path,
                                const std::vector<Obstacle>& obstacles,
                                nav_msgs::msg::Path& opt_path) {
  UnicycleEnv env;
  env.Q = w_Q * Matrix<X_DIM, X_DIM>::Identity();
  env.scaleFactor = scaleFactor;
  env.obstacleFactor = obstacleFactor;
  env.rotCost = rotCost;
  env.xStart = Vector<X_DIM>::Zero();
  env.xStart[0] = cur_pose.position.x;
  env.xStart[1] = cur_pose.position.y;
  env.xStart[2] = utils_tool::getYawFromQuaternion(cur_pose.orientation);

  env.xGoal = Vector<X_DIM>::Zero();
  env.xGoal[0] = ref_path.poses.back().pose.position.x;
  env.xGoal[1] = ref_path.poses.back().pose.position.y;
  env.xGoal[2] =
      utils_tool::getYawFromQuaternion(ref_path.poses.back().pose.orientation);
  env.dt = 0.15;
  env.wheel_base = wheel_base;

  env.T =
      static_cast<int>(utils_tool::mod(ref_path.poses.back().pose - cur_pose)) /
      (cur_speed_ * env.dt);
  RCLCPP_INFO_STREAM(get_logger(), "T: " << env.T);
  env.R = w_R * Matrix<U_DIM, U_DIM>::Identity();
  env.uNominal[0] = cur_speed_;
  env.uNominal[1] = 0;
  env.robotRadius = radius_;
  auto temp_back_pose = utils_tool::absoluteSum(cur_pose, back_center_);
  auto temp_front_pose = utils_tool::absoluteSum(cur_pose, front_center_);
  RCLCPP_INFO_STREAM(get_logger(), "back_pose: " << temp_back_pose);
  RCLCPP_INFO_STREAM(get_logger(), "front_pose: " << temp_front_pose);

  env.bottomLeft[0] = temp_back_pose.position.x;
  env.bottomLeft[1] = temp_back_pose.position.y;
  env.topRight[0] = temp_front_pose.position.x;
  env.topRight[1] = temp_front_pose.position.y;
  env.obstacles = obstacles;
  int numIter;
  Vector<U_DIM> u0 = Vector<U_DIM>::Zero();
  void* g_env = &env;
  std::vector<Matrix<U_DIM, X_DIM>> K;
  std::vector<Vector<U_DIM>> d;
  if (iterativeLQR(env.T, env.xStart, u0, g, evaluateFinalCost, cT,
                   evaluateCost, ct, K, d, true, numIter, g_env)) {
    Vector<X_DIM> x = env.xStart;
    Vector<U_DIM> u;
    std::vector<double> speed;
    for (int k = 0; k < env.T; ++k) {
      geometry_msgs::msg::PoseStamped temp_pose;
      temp_pose.pose.position.x = x[0];
      temp_pose.pose.position.y = x[1];
      temp_pose.pose.orientation = utils_tool::createQuaternionMsgFromYaw(x[2]);
      u = K[k] * x + d[k];
      speed.push_back(u[0]);
      opt_path.poses.emplace_back(std::move(temp_pose));
      x = g(&env, x, u);
      file_ << std::fixed << std::setprecision(5) << x[0] << " " << std::fixed
            << std::setprecision(5) << x[1] << " " << std::fixed
            << std::setprecision(5) << x[2] << " " << std::fixed
            << std::setprecision(5) << u[0] << " " << std::fixed
            << std::setprecision(5) << u[1] << std::endl;
    }
    geometry_msgs::msg::PoseStamped temp_pose;
    temp_pose.pose.position.x = x[0];
    temp_pose.pose.position.y = x[1];
    temp_pose.pose.orientation = utils_tool::createQuaternionMsgFromYaw(x[2]);
    speed.push_back(u[0]);
    opt_path.poses.emplace_back(std::move(temp_pose));
    file_ << std::fixed << std::setprecision(5) << x[0] << " " << std::fixed
          << std::setprecision(5) << x[1] << " " << std::fixed
          << std::setprecision(5) << x[2] << " " << std::fixed
          << std::setprecision(5) << u[0] << " " << std::fixed
          << std::setprecision(5) << u[1] << std::endl;
    return true;
  } else {
    return false;
  }
}

std::string CIlqrPlanner::get_cur_time_str() {
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

void CIlqrPlanner::publishVehiclePath(const nav_msgs::msg::Path& path,
                                      double width, double length,
                                      int interval) {
  visualization_msgs::msg::MarkerArray vehicle_array;
  geometry_msgs::msg::PoseStamped trans;
  trans.pose.position.x = length / 2 - 0.2;
  for (int i = 0; i < path.poses.size(); i += interval) {
    visualization_msgs::msg::Marker vehicle;
    if (i == 0) {
      vehicle.action = 3;
    }
    vehicle.header.frame_id = "map";
    vehicle.header.stamp = get_clock()->now();
    vehicle.ns = "vehicle_path";
    vehicle.type = visualization_msgs::msg::Marker::CUBE;
    vehicle.id = static_cast<int>(i / interval);
    vehicle.scale.x = length;
    vehicle.scale.y = width;
    vehicle.scale.z = 0.01;

    vehicle.color.r = 1.0;
    vehicle.color.g = 0.0;
    vehicle.color.b = 0.0;
    vehicle.color.a = 0.1;
    geometry_msgs::msg::PoseStamped temp;

    vehicle.pose = utils_tool::absoluteSum(path.poses.at(i), trans).pose;
    // vehicle.pose = path.poses.at(i).pose;
    vehicle_array.markers.emplace_back(std::move(vehicle));
  }
  vehicle_path_publisher_->publish(vehicle_array);
}

void CIlqrPlanner::declareParameter() {
  declare_parameter("scaleFactor", 1.0);
  declare_parameter("obstacleFactor", 1.0);
  declare_parameter("rotCost", 0.4);
  declare_parameter("w_Q", 50.0);
  declare_parameter("w_R", 1.0);
  declare_parameter("wheel_base", 0.5);

  declare_parameter("begin_pose_x", 0.0);
  declare_parameter("begin_pose_y", 0.0);
  declare_parameter("begin_pose_theta", 0.0);
  declare_parameter("end_pose_x", 0.0);
  declare_parameter("end_pose_y", 0.0);
  declare_parameter("end_pose_theta", 0.0);
  declare_parameter("obs_radius", 0.0);
  declare_parameter("first_obs_pose_x", 0.0);
  declare_parameter("first_obs_pose_y", 0.0);
  declare_parameter("second_obs_pose_x", 0.0);
  declare_parameter("second_obs_pose_y", 0.0);
  declare_parameter("third_obs_pose_x", 0.0);
  declare_parameter("third_obs_pose_y", 0.0);

  scaleFactor = get_parameter("scaleFactor").as_double();
  obstacleFactor = get_parameter("obstacleFactor").as_double();
  rotCost = get_parameter("rotCost").as_double();
  w_Q = get_parameter("w_Q").as_double();
  w_R = get_parameter("w_R").as_double();
  wheel_base = get_parameter("wheel_base").as_double();

  begin_pose_x = get_parameter("begin_pose_x").as_double();
  begin_pose_y = get_parameter("begin_pose_y").as_double();
  begin_pose_theta = get_parameter("begin_pose_theta").as_double();
  end_pose_x = get_parameter("end_pose_x").as_double();
  end_pose_y = get_parameter("end_pose_y").as_double();
  end_pose_theta = get_parameter("end_pose_theta").as_double();
  obs_radius = get_parameter("obs_radius").as_double();
  first_obs_pose_x = get_parameter("first_obs_pose_x").as_double();
  first_obs_pose_y = get_parameter("first_obs_pose_y").as_double();
  second_obs_pose_x = get_parameter("second_obs_pose_x").as_double();
  second_obs_pose_y = get_parameter("second_obs_pose_y").as_double();
  third_obs_pose_x = get_parameter("third_obs_pose_x").as_double();
  third_obs_pose_y = get_parameter("third_obs_pose_y").as_double();
}
