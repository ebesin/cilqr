#pragma once

#include "ackermann_msgs/msg/ackermann_drive.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "vehicle_state_interface.h"

namespace VehicleState {
class VehicleStateAckermann : public VehicleStateInterface {
 public:
  ackermann_msgs::msg::AckermannDrive expand_state_;
};
}  // namespace VehicleState
