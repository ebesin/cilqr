/*
 * @Author       : dwayne
 * @Date         : 2023-06-27
 * @LastEditTime : 2023-06-28
 * @Description  : 
 * 
 * Copyright (c) 2023 by dwayne, All Rights Reserved. 
 */

#include "path_generator.hpp"
#include "visualization_tool.hpp"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<utils_tool::VisualizationTools>("visualization_tool"));
    rclcpp::shutdown();
    return 0;
}