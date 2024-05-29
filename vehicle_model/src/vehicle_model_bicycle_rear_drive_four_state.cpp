
#include "vehicle_model_bicycle_rear_drive_four_state.h"

#include <tf2/utils.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_state_ackermann.h"
#include "vehicle_state_interface.h"

using namespace VehicleState;

namespace VehicleModel {

VehicleModelBicycleRearDriveFourState::VehicleModelBicycleRearDriveFourState(
    const double& wheel_base)
    : VehicleModelInterface(4, 2, wheel_base) {
  m_model_type = VehicleModelType::kAckermann;
}

void VehicleModelBicycleRearDriveFourState::setCurState(
    const std::shared_ptr<VehicleStateInterface>& cur_state) {
  m_cur_state_vec = Eigen::VectorXd::Zero(m_dim_x);
  // VehicleStateInterface state = cur_state;
  std::shared_ptr<VehicleStateAckermann> state_ptr =
      std::dynamic_pointer_cast<VehicleStateAckermann>(cur_state);
  m_cur_state_vec << state_ptr->base_state_.pose.pose.position.x,
      state_ptr->base_state_.pose.pose.position.y,
      tf2::getYaw(state_ptr->base_state_.pose.pose.orientation),
      state_ptr->base_state_.twist.twist.linear.x;
  VehicleModelInterface::setCurState(cur_state);
}

void VehicleModelBicycleRearDriveFourState::setEndState(
    const std::shared_ptr<VehicleStateInterface>& end_state) {
  m_end_state_vec = Eigen::VectorXd::Zero(m_dim_x);
  std::shared_ptr<VehicleStateAckermann> state_ptr =
      std::dynamic_pointer_cast<VehicleStateAckermann>(end_state);
  m_end_state_vec << state_ptr->base_state_.pose.pose.position.x,
      state_ptr->base_state_.pose.pose.position.y,
      tf2::getYaw(state_ptr->base_state_.pose.pose.orientation),
      state_ptr->base_state_.twist.twist.linear.x;
  VehicleModelInterface::setEndState(end_state);
}

int VehicleModelBicycleRearDriveFourState::getMatrixA(const Eigen::VectorXd& x,
                                                      const Eigen::VectorXd& u,
                                                      const double dt,
                                                      Eigen::MatrixXd& a) {
  if (x.size() != m_dim_x || u.size() != m_dim_u) return -1;
  a = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  a(0, 2) = dt * u(0) * sin(x(2));
  a(1, 2) = dt * u(0) * cos(x(2));
  a(2, 3) = dt * u(0) * (1 + tan(x(3)) * tan(x(3))) / m_wheelbase;
  return 0;
}

int VehicleModelBicycleRearDriveFourState::getMatrixB(const Eigen::VectorXd& x,
                                                      const Eigen::VectorXd& u,
                                                      const double dt,
                                                      Eigen::MatrixXd& b) {
  if (x.size() != m_dim_x || u.size() != m_dim_u) return -1;
  b = Eigen::MatrixXd::Zero(m_dim_x, m_dim_u);
  b(0, 0) = dt * cos(x(2));
  b(1, 0) = dt * sin(x(2));
  b(2, 0) = dt * tan(x(3)) / m_wheelbase;
  b(3, 1) = dt;
  return 0;
}

Eigen::VectorXd VehicleModelBicycleRearDriveFourState::getConstrainedU(
    const Eigen::VectorXd& u) {
  Eigen::VectorXd constrained_u = u;
  if (has_constrain) {
    Eigen::VectorXd diff = (m_u_max - m_u_min) / 2.;
    Eigen::VectorXd mean = (m_u_max + m_u_min) / 2.;
    Eigen::VectorXd temp = u.array().tanh();
    constrained_u = diff.cwiseProduct(temp) + mean;
  }
  return constrained_u;
}

int VehicleModelBicycleRearDriveFourState::toNextState(
    const Eigen::VectorXd& x, const Eigen::VectorXd& u, const double dt,
    Eigen::VectorXd& next_state) {
  if (x.size() != m_dim_x || u.size() != m_dim_u) return -1;
  Eigen::VectorXd constrained_u = getConstrainedU(u);
  next_state = Eigen::VectorXd::Zero(m_dim_x);
  next_state(0) = x(0) + dt * constrained_u(0) * cos(x(2));
  next_state(1) = x(1) + dt * constrained_u(0) * sin(x(2));
  next_state(2) = x(2) + dt * constrained_u(0) * tan(x(3)) / m_wheelbase;
  next_state(3) = x(3) + dt * constrained_u(1);
  return 0;
}
}  // namespace VehicleModel
