#pragma once
#include <memory>

#include "vehicle_model_interface.h"

using namespace VehicleState;
using namespace VehicleModel;

class SimRobot {
 private:
  std::shared_ptr<VehicleModelInterface> vehicle_model_ptr_;
  Eigen::VectorXd init_state_;
  Eigen::VectorXd cur_state_;

 public:
  SimRobot(std::shared_ptr<VehicleModelInterface> vehicle_model_ptr,
           const Eigen::VectorXd& init_state);

  Eigen::VectorXd getCurState();

  Eigen::VectorXd toNextState(const Eigen::VectorXd& u, double sim_time,
                              double dt);

  Eigen::VectorXd calNextState(const Eigen::VectorXd& u, double time);

  void resetState();

  void setState(const Eigen::VectorXd& state);
};
