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
    rviz_config_file = LaunchConfiguration('rviz_config')

    param_config = os.path.join(
        get_package_share_directory('utils_tool'),
        'config',
        'generate_path.yaml'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description="log_level")

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config', default_value=os.path.join(
            get_package_share_directory('utils_tool'),
            'rviz', 'rviz.rviz')
    )

    generator_path_node = Node(
        package='utils_tool',
        executable='path_generator_node',
        name='path_generator',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[param_config])

    visualization_node = Node(
        package='utils_tool',
        executable='visualization_tool_node',
        name='visualization_tool',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[param_config])

    sim_robot_node = Node(
        package='sim_robot',
        executable='sim_robot_node',
        name='sim_robot',
        output='screen',
        arguments=['--ros-args', '--log-level', log_level],
        parameters=[param_config])

    # load_nodes = GroupAction(
    #     actions=[generator_path_node, visualization_node])

    load_nodes = GroupAction(
        actions=[generator_path_node, visualization_node, sim_robot_node])

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen')

    register_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=rviz_node,
            on_start=[
                TimerAction(
                    period=5.0,
                    actions=[load_nodes]
                )
            ]
        )
    )

    ld = LaunchDescription(
        [
            declare_log_level_cmd,
            declare_rviz_config_file_cmd,
            rviz_node,
            register_handler
        ]
    )
    # ld = LaunchDescription(
    #     [
    #         declare_log_level_cmd,
    #         declare_rviz_config_file_cmd,
    #         # rviz_node,
    #         load_nodes
    #     ]
    # )

    return ld
