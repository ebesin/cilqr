#include <tf2/LinearMath/Quaternion.h>
#include <tf2/utils.h>

#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <utility>

#include "coor_tools.h"
#include "ilqr_path_planner.h"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rs_path.h"
#include "vehicle_model_bicycle_rear_drive_five_state.h"
#include "vehicle_model_bicycle_rear_drive_four_state.h"
#include "vehicle_model_bicycle_rear_drive_three_state.h"

typedef Eigen::Vector3d Vec3d;

class Test : public rclcpp::Node {
 public:
  Test() : rclcpp::Node("test") {
    std::shared_ptr<guided_hybrid_a_star::RSPath> rs_path_ptr_ =
        std::make_shared<guided_hybrid_a_star::RSPath>(
            0.65 / tan(utils_tool::deg2Rad(25)));
    Vec3d start;
    start.x() = 0.0;
    start.y() = 0.0;
    start.z() = 0.0;
    Vec3d goal;
    goal.x() = 0.5;
    goal.y() = 0.5;
    goal.z() = -0.5;
    double distance;
    auto pose_vec = rs_path_ptr_->GetRSPath(start, goal, 0.05, distance);
    nav_msgs::msg::Path path;
    path.header.frame_id = "map";
    path.header.stamp = get_clock()->now();
    for (auto p : pose_vec) {
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = p.x();
      pose.pose.position.y = p.y();
      pose.pose.orientation = utils_tool::createQuaternionMsgFromYaw(p.z());
      path.poses.emplace_back(std::move(pose));
    }
    RCLCPP_INFO_STREAM(get_logger(), path);
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub =
        create_publisher<nav_msgs::msg::Path>("test_path",
                                              rclcpp::SystemDefaultsQoS());
    RCLCPP_INFO(get_logger(), "pub....");
    pub->publish(path);
  }
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Test>());
  rclcpp::shutdown();
  return 0;
}
