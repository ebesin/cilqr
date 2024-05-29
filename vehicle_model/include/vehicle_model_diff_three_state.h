#pragma once
#include "vehicle_model_interface.h"

namespace VehicleModel {

/*
 *
 * State & Input
 * x = [x, y,Θ]^T
 * u = [v,ω]^T
 *
 *    Nonlinear model
 * dx0/dt = u(0) * cos(x2)
 * dx1/dt = u(0) * sin(x2)
 * dx2/dt = u(1)
 *
 *    Linearized model
 *
 *       [0, 0, -v*sin(Θ)]       [cos(Θ),0]
 * dx/dt=[0, 0,  v*cos(Θ)] * x + [sin(Θ),0] * u
 *       [0, 0,     0    ]       [  0   ,1]
 *
 */
class VehicleModelDiffThreeState : public VehicleModelInterface {
 private:
  /* data */
 public:
  VehicleModelDiffThreeState(/* args */);
  ~VehicleModelDiffThreeState() = default;

  Eigen::VectorXd getConstrainedU(const Eigen::VectorXd& u);

  void setCurState(const VehicleStateInterface& cur_state) override;
  void setEndState(const VehicleStateInterface& end_state) override;

  int getMatrixA(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                 const double dt, Eigen::MatrixXd& a) override;

  int getMatrixB(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                 const double dt, Eigen::MatrixXd& b) override;

  int toNextState(const Eigen::VectorXd& x, const Eigen::VectorXd& u,
                  const double dt, Eigen::VectorXd& next_state) override;
};

}  // namespace VehicleModel
