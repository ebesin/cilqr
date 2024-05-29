#pragma once
#include "vehicle_model_interface.h"

namespace VehicleModel {
class VehicleModelDiffFourState : public VehicleModelInterface {
 public:
  VehicleModelDiffFourState(/* args */);
  ~VehicleModelDiffFourState() = default;

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
