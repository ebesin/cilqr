cmake_minimum_required(VERSION 3.5)

project(sim_robot)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(utils_tool REQUIRED)


# 设置头文件相对路径
include_directories(
    include
)

set(sim_robot_library_name sim_robot_lib)
set(SIM_ROBOT_SRC
    src/bicycle_kinematics.cpp
    src/sim_ackermann.cpp
    src/sim_vehicle.cpp
)
add_library(${sim_robot_library_name} SHARED
    ${SIM_ROBOT_SRC}
)
set(SIM_ROBOT_DEPENDENCIES
    geometry_msgs
    rclcpp
    nav_msgs
    vehicle_model
    tf2
    tf2_ros
    mpc_msgs
    utils_tool
    tf2_geometry_msgs
)
ament_target_dependencies(${sim_robot_library_name}
    ${SIM_ROBOT_DEPENDENCIES}
)

set(sim_robot_exec_name sim_robot_node)
set(SIM_EXEC_ROBOT_SRC
    src/sim_node.cpp
)
add_executable(${sim_robot_exec_name} ${SIM_EXEC_ROBOT_SRC})
target_link_libraries(${sim_robot_exec_name}
    ${sim_robot_library_name}
)
ament_target_dependencies(${sim_robot_exec_name}
    ${SIM_ROBOT_DEPENDENCIES}
)

install(TARGETS ${sim_robot_exec_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${sim_robot_library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)

    # the following line skips the linter which checks for copyrights
    # comment the line when a copyright and license is added to all source files
    set(ament_cmake_copyright_FOUND TRUE)

    # the following line skips cpplint (only works in a git repo)
    # comment the line when this package is in a git repo and when
    # a copyright and license is added to all source files
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

ament_package()
