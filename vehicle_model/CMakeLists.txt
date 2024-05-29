cmake_minimum_required(VERSION 3.8)
project(vehicle_model)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(mpc_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# 设置头文件相对路径
include_directories(
    include
)


set(DEPENDENCIES
    std_msgs
    nav_msgs
    ackermann_msgs
    geometry_msgs
    Eigen3
    tf2
    tf2_ros
    tf2_geometry_msgs
)


set(vehicle_model_lib vehicle_model_lib)
set(VEHICLE_MODEL_SRC
    src/vehicle_model_interface.cpp
    # src/vehicle_model_diff_three_state.cpp
    # src/vehicle_model_diff_four_state.cpp
    src/vehicle_model_bicycle_rear_drive_five_state.cpp
    src/vehicle_model_bicycle_rear_drive_four_state.cpp
    src/vehicle_model_bicycle_rear_drive_three_state.cpp
    src/sim_robot.cpp
)
add_library(${vehicle_model_lib} SHARED
    ${VEHICLE_MODEL_SRC}
)
ament_target_dependencies(${vehicle_model_lib}
    ${DEPENDENCIES}
)


install(TARGETS ${vehicle_model_lib}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION lib/${PROJECT_NAME}
)


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
  ${vehicle_model_lib}
)
ament_export_dependencies(${DEPENDENCIES})


ament_package()
