/*
 * @Author       : dwayne
 * @Date         : 2023-06-29
 * @LastEditTime : 2023-06-29
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */

#include "path_generator.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utils_tool::PathGenerator>("path_generator"));
  rclcpp::shutdown();
  return 0;
}
