/*
 * @Author       : dwayne
 * @Date         : 2023-06-29
 * @LastEditTime : 2023-06-29
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <iostream>
#include <nav_msgs/msg/detail/path__struct.hpp>

#include "3rd/backward.hpp"
#include "bezier_plan.h"
#include "coor_tools.h"
#include "gtest/gtest.h"

namespace backward {
backward::SignalHandling sh;
}

TEST(utils_tool, test_getYawFromQuaternion) {
  std::cout << "test_getYawFromQuaternion...." << std::endl;
  std::cout << "angle of [1.0, 0.0]: "
            << utils_tool::getHeadingFromVector(1.0, 0.0) << std::endl;
  // EXPECT_TRUE(astar.getAngleIndex(1.2));
  // EXPECT_EQ(astar.getAngleIndex(0), 0);
  // EXPECT_EQ(astar.getAngleIndex(3.14), 34);
  // EXPECT_EQ(astar.getAngleIndex(6.28), 69);
  // EXPECT_EQ(astar.getAngleIndex(6.29), 0);
}

TEST(utils_tool, bezier_plan) {
  geometry_msgs::msg::PoseStamped begin_pose;
  geometry_msgs::msg::PoseStamped end_pose;
  end_pose.pose.position.x = 3;
  end_pose.pose.position.y = 3;
  nav_msgs::msg::Path path;
  doBezierPlan(begin_pose, end_pose, path, 0.05);
  std::cout << path << std::endl;
}

int main(int argc, char* argv[]) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
