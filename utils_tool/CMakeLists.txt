cmake_minimum_required(VERSION 3.5)

project(utils_tool)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(GTest REQUIRED)

# 设置头文件相对路径
include_directories(
    include
)

# path_generator
set(path_generator_library_name path_generator_lib)
set(PATH_GENERATOR_SRC
    src/coor_tools.cpp
    src/rs_path.cpp
    src/path_generator.cpp
    src/bezier_plan.cpp
)
add_library(${path_generator_library_name} SHARED
    ${PATH_GENERATOR_SRC}
)
set(PATH_GENERATOR_DEPENDENCIES
    geometry_msgs
    rclcpp
    nav_msgs
    visualization_msgs
    tf2_geometry_msgs
    mpc_msgs
)
ament_target_dependencies(${path_generator_library_name}
    ${PATH_GENERATOR_DEPENDENCIES}
)

# visualization_tool
set(visualization_tool_library_name visualization_tool_lib)
set(VISUALIZATION_TOOL_SRC
    src/visualization_tool.cpp
)
add_library(${visualization_tool_library_name} SHARED
    ${VISUALIZATION_TOOL_SRC}
)
set(VISUALIZATION_TOOL_DEPENDENCIES
    geometry_msgs
    rclcpp
    nav_msgs
    visualization_msgs
    tf2_geometry_msgs
)
ament_target_dependencies(${visualization_tool_library_name}
    ${VISUALIZATION_TOOL_DEPENDENCIES}
)

set(visualization_tool_exec_name visualization_tool_node)
set(VISUALIZATION_TOOL_EXEC_SRC
    src/visualization_node.cpp
)
add_executable(${visualization_tool_exec_name} ${VISUALIZATION_TOOL_EXEC_SRC})
set(VISUALIZATION_TOOL_EXEC_DEPENDENCIES
    mpc_msgs
)
ament_target_dependencies(${visualization_tool_exec_name}
    ${VISUALIZATION_TOOL_EXEC_DEPENDENCIES}
)
target_link_libraries(${visualization_tool_exec_name}
    ${visualization_tool_library_name}
    ${path_generator_library_name}
)

# exec
set(path_generator_exec_name path_generator_node)
set(PATH_GENERATOR_EXEC_SRC
    src/path_generator_node.cpp
)
add_executable(${path_generator_exec_name} ${PATH_GENERATOR_EXEC_SRC})
set(PATH_GENERATOR_EXEC_DEPENDENCIES
    geometry_msgs
    rclcpp
    nav_msgs
    visualization_msgs
    tf2_geometry_msgs
    mpc_msgs
)
ament_target_dependencies(${path_generator_exec_name}
    ${PATH_GENERATOR_EXEC_DEPENDENCIES}
)
target_link_libraries(${path_generator_exec_name}
    ${path_generator_library_name}
)

# test
set(util_tool_test_exec_name util_tool_test)
set(UTIL_TOOL_TEST_EXEC_SRC
    test/test_coor_trans.cpp
)
add_executable(${util_tool_test_exec_name} ${UTIL_TOOL_TEST_EXEC_SRC})
set(UTIL_TOOL_TEST_EXEC_DEPENDENCIES
    GTest
    geometry_msgs
    rclcpp
    nav_msgs
    visualization_msgs
    tf2_geometry_msgs
    mpc_msgs
)
ament_target_dependencies(${util_tool_test_exec_name}
    ${UTIL_TOOL_TEST_EXEC_DEPENDENCIES}
)
target_link_libraries(${util_tool_test_exec_name}
    ${path_generator_library_name}
    gtest
)


install(TARGETS ${visualization_tool_exec_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${path_generator_exec_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${util_tool_test_exec_name}
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${visualization_tool_library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${path_generator_library_name}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY rviz DESTINATION share/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/)



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

ament_export_include_directories(include)
ament_export_libraries(
    ${path_generator_library_name}
)
ament_export_dependencies(${DEPENDENCIES})

ament_package()
