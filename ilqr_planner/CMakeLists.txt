cmake_minimum_required(VERSION 3.8)
project(ilqr_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vehicle_model REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(utils_tool REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTest)

set(CMAKE_BUILD_TYPE "Debug")

# 设置头文件相对路径
include_directories(
    include
)
include_directories(${mpc_msgs_INCLUDE_DIRS})

set(DEPENDENCIES
    rclcpp
    vehicle_model
    nav_msgs
    mpc_msgs
    tf2
    tf2_geometry_msgs
    utils_tool
    Eigen3
    visualization_msgs
)

set(OPTIMIZER_SRC
    src/ilqr_optimizer.cpp
    src/optimizer_utils.cpp
    src/ilqr_path_optimizer.cpp
    src/ilqr_planner.cpp
    src/ilqr_path_planner.cpp
    src/rs_path.cpp
    src/cilqr_solver.cpp
    src/cilqr_planner.cpp
)
add_library(optimizer_lib SHARED
    ${OPTIMIZER_SRC}
)
ament_target_dependencies(optimizer_lib
    ${DEPENDENCIES}
)

# release
add_executable(${PROJECT_NAME}_node src/optimizer_node.cpp)
target_link_libraries(${PROJECT_NAME}_node
    optimizer_lib
)
ament_target_dependencies(${PROJECT_NAME}_node
    ${DEPENDENCIES}
)

# release
add_executable(cilqr_node src/cilqr_node.cpp)
target_link_libraries(cilqr_node
    optimizer_lib
)
ament_target_dependencies(cilqr_node
    ${DEPENDENCIES}
)

# tests
add_executable(${PROJECT_NAME}_test tests/ros_test_optimizer.cpp)
target_link_libraries(${PROJECT_NAME}_test
    optimizer_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_test 
    ${DEPENDENCIES}
)

# tests
add_executable(ilqr_path_planner_test tests/test_ilqr_path_planner.cpp)
target_link_libraries(ilqr_path_planner_test
    optimizer_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
)
ament_target_dependencies(ilqr_path_planner_test
    ${DEPENDENCIES}
)

# tests
add_executable(rs_path_test tests/test_rs_path.cpp)
target_link_libraries(rs_path_test
    optimizer_lib
    ${GTEST_LIBRARIES}
    ${GTEST_MAIN_LIBRARIES}
)
ament_target_dependencies(rs_path_test
    ${DEPENDENCIES}
)


install(TARGETS ${PROJECT_NAME}_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS cilqr_node
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS optimizer_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME}_test
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS ilqr_path_planner_test
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS rs_path_test
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})


install(DIRECTORY include/
  DESTINATION include/)
  
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()


ament_export_include_directories(include)
ament_export_libraries(
    ${PROJECT_NAME}
    optimizer_lib
)

ament_export_dependencies(${DEPENDENCIES})

ament_package()
