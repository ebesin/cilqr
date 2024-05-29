#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "coor_tools.h"
#include "ilqr_path_planner.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "vehicle_model_bicycle_rear_drive_five_state.h"
#include "vehicle_model_bicycle_rear_drive_four_state.h"
#include "vehicle_model_bicycle_rear_drive_three_state.h"

class Test : public rclcpp::Node {
 public:
  Test() : rclcpp::Node("test") {
    // std::shared_ptr<VehicleModel::VehicleModelInterface> vehicle_model_ptr_ =
    //     std::make_shared<VehicleModel::VehicleModelBicycleRearDriveFourState>(
    //         0.65);
    std::shared_ptr<VehicleModel::VehicleModelInterface> vehicle_model_ptr_ =
        std::make_shared<VehicleModel::VehicleModelBicycleRearDriveFiveState>(
            0.65);

    Eigen::MatrixXd Q, Q_end, R;
    Q = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                              vehicle_model_ptr_->getDimX());
    Q_end = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimX(),
                                  vehicle_model_ptr_->getDimX());
    R = Eigen::MatrixXd::Zero(vehicle_model_ptr_->getDimU(),
                              vehicle_model_ptr_->getDimU());

    Q_end(0, 0) = 1e4;
    Q_end(1, 1) = 1e4;
    Q_end(2, 2) = 1e4;
    R(0, 0) = 1e-2;
    R(1, 1) = 1e-2;

    Optimizer::IlqrPathPlanner planner(vehicle_model_ptr_, Q, Q_end, R);
    double begin_x = 0.0;
    double begin_y = 0.0;
    double begin_theta = 0.0;
    double end_x = -1.0;
    double end_y = -1.0;
    double end_theta = 0;
    geometry_msgs::msg::Pose begin_pose;
    begin_pose.position.x = begin_x;
    begin_pose.position.y = begin_y;
    tf2::Quaternion q;
    q.setRPY(0, 0, begin_theta);
    begin_pose.orientation = tf2::toMsg(q);
    geometry_msgs::msg::Pose end_pose;
    end_pose.position.x = end_x;
    end_pose.position.y = end_y;
    q.setRPY(0, 0, end_theta);
    end_pose.orientation = tf2::toMsg(q);
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = get_clock()->now();

    // std::cout << "begin_pose: " << begin_pose << "  end_pose:" <<
    // end_pose
    //           << std::endl;
    // Eigen::VectorXd cur_state =
    //     Eigen::VectorXd::Zero(vehicle_model_ptr_->getDimX());
    // cur_state << begin_pose.position.x, begin_pose.position.y,
    //     tf2::getYaw(begin_pose.orientation), 0.0, 0.0;
    //   vehicle_model_ptr_->setCurStateVec(cur_state);

    planner.doPlan(begin_pose, end_pose, path);

    std::cout << path << std::endl;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub =
        create_publisher<nav_msgs::msg::Path>("test_path",
                                              rclcpp::SystemDefaultsQoS());
    pub->publish(path);
    // rclcpp::shutdown();
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test>());
  rclcpp::shutdown();
  return 0;
}
