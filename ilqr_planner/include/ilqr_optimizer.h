#pragma once

#include <Eigen/Core>
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <vector>

#include "optimizer_interface.h"

// #include "vehicle_model_interface.h"
#include "vehicle_model_interface.h"

namespace Optimizer {
class IlqrOptimizer : public OptimizerInterface {
 public:
  IlqrOptimizer(const std::shared_ptr<VehicleModel::VehicleModelInterface>&
                    m_vehicle_model_ptr,
                const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Q_end,
                const Eigen::MatrixXd& R);

  ~IlqrOptimizer() = default;

  int optimize(const std::vector<Eigen::VectorXd>& ref_state,
               std::vector<Eigen::VectorXd>& optimized_state,
               std::vector<Eigen::VectorXd>& control) override;

  void setTimeInterval(const double time_interval);
  void setStopDiffCost(const double stop_diff_cost);
  void setStep(const int step);
  void setMaxIteration(const int max_iteration);
  int setWightMatrix(const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Q_end,
                     const Eigen::MatrixXd& R);
  int setEndWightMatrix(const Eigen::MatrixXd& Q_end);
  double getBestCost();

 protected:
  std::shared_ptr<VehicleModel::VehicleModelInterface> m_vehicle_model_ptr;

 private:
  int solve(const Eigen::VectorXd& x_init, std::vector<Eigen::VectorXd>& x_res,
            Eigen::MatrixXd& u_res);

  Eigen::VectorXd flatten(const Eigen::MatrixXd& M);
  Eigen::MatrixXd reshape(Eigen::VectorXd& v, unsigned int cols,
                          unsigned int rows);
  Eigen::MatrixXd rollout(const Eigen::VectorXd& x_init,
                          const Eigen::MatrixXd& U);
  Eigen::MatrixXd getSu(const Eigen::MatrixXd& X, const Eigen::MatrixXd& U);
  double cost(const Eigen::MatrixXd& X, const Eigen::MatrixXd& U);

  int m_dim_x;
  int m_dim_u;
  double m_time_interval;
  double m_step;
  int m_max_iteration{100};
  double m_best_cost;
  Eigen::VectorXd m_x_init;
  Eigen::VectorXd m_ref_state;
  Eigen::MatrixXd m_Q;
  Eigen::MatrixXd m_Q_end;
  Eigen::MatrixXd m_R;
  Eigen::MatrixXd m_Qex;
  Eigen::MatrixXd m_Rex;
  double m_stop_diff_cost{0.01};
};
}  // namespace Optimizer
