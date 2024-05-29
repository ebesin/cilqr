#include "sim_robot.h"

#include <Eigen/src/Core/Matrix.h>

SimRobot::SimRobot(std::shared_ptr<VehicleModelInterface> vehicle_model_ptr,
                   const Eigen::VectorXd& init_state)
    : vehicle_model_ptr_(vehicle_model_ptr),
      init_state_(init_state),
      cur_state_(init_state) {}

Eigen::VectorXd SimRobot::getCurState() { return cur_state_; }

Eigen::VectorXd SimRobot::toNextState(const Eigen::VectorXd& u, double sim_time,
                                      double dt) {
  Eigen::VectorXd next_state;
  double time = 0;
  while (time < sim_time) {
    vehicle_model_ptr_->toNextState(cur_state_, u, dt, next_state);
    time += dt;
    cur_state_ = next_state;
  }
  double remain_time = sim_time - (time - sim_time);
  vehicle_model_ptr_->toNextState(cur_state_, u, remain_time, next_state);
  cur_state_ = next_state;
  return getCurState();
}

Eigen::VectorXd SimRobot::calNextState(const Eigen::VectorXd& u, double time) {
  Eigen::VectorXd next_state;
  vehicle_model_ptr_->toNextState(cur_state_, u, time, next_state);
  return next_state;
}

void SimRobot::resetState() { cur_state_ = init_state_; }

void SimRobot::setState(const Eigen::VectorXd& state) { cur_state_ = state; }
