'''
Author       : dwayne
Date         : 2023-07-02
LastEditTime : 2023-07-02
Description  : 

Copyright (c) 2023 by dwayne, All Rights Reserved. 
'''
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, RegisterEventHandler, LogInfo, TimerAction, ExecuteProcess
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import HasNodeParams, RewrittenYaml
from launch.logging import launch_config
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    log_level = LaunchConfiguration('log_level')

    param_config = os.path.join(
        get_package_share_directory('ilqr_planner'),
        'config',
        'ilqr_planner.yaml'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description="log_level")

    ilqr_planner_node = Node(
        package='ilqr_planner',
        executable='ilqr_planner_node',
        name='ilqr_planner',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[param_config])

    ld = LaunchDescription(
        [
            declare_log_level_cmd,
            ilqr_planner_node,
        ]
    )

    return ld
