#include "vehicle_model_interface.h"

namespace VehicleModel {
VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u,
                                             double wheelbase)
    : m_dim_x(dim_x), m_dim_u(dim_u), m_wheelbase(wheelbase) {}

VehicleModelInterface::VehicleModelInterface(int dim_x, int dim_u)
    : m_dim_x(dim_x), m_dim_u(dim_u), m_wheelbase(0) {}

int VehicleModelInterface::getDimX() const { return m_dim_x; }

int VehicleModelInterface::getDimU() const { return m_dim_u; }

double VehicleModelInterface::getWheelbase() const { return m_wheelbase; }

void VehicleModelInterface::setCurvature(double curvature) {
  m_curvature = curvature;
}

void VehicleModelInterface::setCurStateVec(const Eigen::VectorXd& cur_state) {
  m_cur_state_vec = cur_state;
}

Eigen::VectorXd VehicleModelInterface::getCurStateVec() const {
  return m_cur_state_vec;
}

Eigen::VectorXd VehicleModelInterface::getEndStateVec() const {
  return m_end_state_vec;
}

VehicleStateInterface VehicleModelInterface::getCurState() const {
  return m_cur_state;
}

VehicleStateInterface VehicleModelInterface::getEndState() const {
  return m_end_state;
}

void VehicleModelInterface::setCurState(
    const std::shared_ptr<VehicleStateInterface>& cur_state) {
  m_cur_state = *cur_state;
}

void VehicleModelInterface::setEndState(
    const std::shared_ptr<VehicleStateInterface>& cur_state) {
  m_end_state = *cur_state;
}

bool VehicleModelInterface::setConstrain(const Eigen::VectorXd& u_min,
                                         const Eigen::VectorXd& u_max) {
  has_constrain = false;
  if (u_min.size() != m_dim_u || u_max.size() != m_dim_u) return false;
  has_constrain = true;
  m_u_min = u_min;
  m_u_max = u_max;
  return true;
}

}  // namespace VehicleModel
