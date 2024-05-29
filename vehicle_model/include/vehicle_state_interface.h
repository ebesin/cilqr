#pragma once
#include "nav_msgs/msg/odometry.hpp"

namespace VehicleState {
class VehicleStateInterface {
 public:
  virtual ~VehicleStateInterface() {}

  nav_msgs::msg::Odometry base_state_;
  double acc_vx_{0};
  double aacc_vx_{0};
};
}  // namespace VehicleState
