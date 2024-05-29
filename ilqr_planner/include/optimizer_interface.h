#pragma once

#include <Eigen/Core>
#include <vector>

namespace Optimizer {
class OptimizerInterface {
 public:
  OptimizerInterface() {}

  ~OptimizerInterface() = default;

  virtual int optimize(const std::vector<Eigen::VectorXd>& ref_state,
                       std::vector<Eigen::VectorXd>& optimized_state,
                       std::vector<Eigen::VectorXd>& control) = 0;
};
}  // namespace Optimizer
