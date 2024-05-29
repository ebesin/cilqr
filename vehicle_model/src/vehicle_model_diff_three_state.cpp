#include "vehicle_model_diff_three_state.h"

#include <tf2/utils.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "vehicle_model_interface.h"
#include "vehicle_state_diff.h"

using namespace VehicleState;

namespace VehicleModel {

VehicleModelDiffThreeState::VehicleModelDiffThreeState(/* args */)
    : VehicleModelInterface(3, 2) {
  m_model_type = VehicleModelType::kDifferential;
}

void VehicleModelDiffThreeState::setCurState(
    const VehicleStateInterface& cur_state) {
  m_cur_state_vec = Eigen::VectorXd::Zero(3);
  VehicleStateInterface state = cur_state;
  auto state_ptr = dynamic_cast<VehicleStateDiff*>(&state);
  m_cur_state_vec << state_ptr->base_state_.pose.pose.position.x,
      state_ptr->base_state_.pose.pose.position.y,
      tf2::getYaw(state_ptr->base_state_.pose.pose.orientation);
  VehicleModelInterface::setCurState(cur_state);
}

void VehicleModelDiffThreeState::setEndState(
    const VehicleStateInterface& end_state) {
  m_end_state_vec = Eigen::VectorXd::Zero(3);
  VehicleStateInterface state = end_state;
  auto state_ptr = dynamic_cast<VehicleStateDiff*>(&state);
  m_end_state_vec << state_ptr->base_state_.pose.pose.position.x,
      state_ptr->base_state_.pose.pose.position.y,
      tf2::getYaw(state_ptr->base_state_.pose.pose.orientation);
  VehicleModelInterface::setEndState(end_state);
}

int VehicleModelDiffThreeState::getMatrixA(const Eigen::VectorXd& x,
                                           const Eigen::VectorXd& u,
                                           const double dt,
                                           Eigen::MatrixXd& a) {
  if (x.size() != m_dim_x || u.size() != m_dim_u) return -1;
  a = Eigen::MatrixXd::Identity(m_dim_x, m_dim_x);
  a(0, 2) = -dt * u(0) * sin(x(2));
  a(1, 2) = dt * u(0) * cos(x(2));
  return 0;
}

int VehicleModelDiffThreeState::getMatrixB(const Eigen::VectorXd& x,
                                           const Eigen::VectorXd& u,
                                           const double dt,
                                           Eigen::MatrixXd& b) {
  if (x.size() != m_dim_x || u.size() != m_dim_u) return -1;
  b = Eigen::MatrixXd::Zero(m_dim_x, m_dim_u);
  b(0, 0) = dt * cos(x(2));
  b(1, 0) = dt * sin(x(2));
  b(2, 1) = dt;
  return 0;
}

Eigen::VectorXd VehicleModelDiffThreeState::getConstrainedU(
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

int VehicleModelDiffThreeState::toNextState(const Eigen::VectorXd& x,
                                            const Eigen::VectorXd& u,
                                            const double dt,
                                            Eigen::VectorXd& next_state) {
  if (x.size() != m_dim_x || u.size() != m_dim_u) return -1;
  Eigen::VectorXd constrained_u = getConstrainedU(u);
  next_state = Eigen::VectorXd::Zero(m_dim_x);
  next_state(0) = x(0) + dt * constrained_u(0) * cos(x(2));
  next_state(1) = x(1) + dt * constrained_u(0) * sin(x(2));
  next_state(2) = x(2) + dt * constrained_u(1);
  return 0;
}

}  // namespace VehicleModel
