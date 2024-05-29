#include "ilqr_optimizer.h"

#include <chrono>
#include <cstddef>
#include <ctime>
#include <iostream>
#include <rclcpp/duration.hpp>
#include <rclcpp/logging.hpp>

#include "rclcpp/rclcpp.hpp"

namespace Optimizer {
IlqrOptimizer::IlqrOptimizer(
    const std::shared_ptr<VehicleModel::VehicleModelInterface>&
        vehicle_model_ptr,
    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Q_end,
    const Eigen::MatrixXd& R)
    : OptimizerInterface() {
  m_vehicle_model_ptr = vehicle_model_ptr;
  m_dim_x = vehicle_model_ptr->getDimX();
  m_dim_u = vehicle_model_ptr->getDimU();
  m_Q = Q;
  m_Q_end = Q_end;
  m_R = R;
}

void IlqrOptimizer::setTimeInterval(const double time_interval) {
  m_time_interval = time_interval;
}

void IlqrOptimizer::setStopDiffCost(const double stop_diff_cost) {
  m_stop_diff_cost = stop_diff_cost;
}

void IlqrOptimizer::setStep(const int step) { m_step = step; }

void IlqrOptimizer::setMaxIteration(const int max_iteration) {
  m_max_iteration = max_iteration;
}

int IlqrOptimizer::setWightMatrix(const Eigen::MatrixXd& Q,
                                  const Eigen::MatrixXd& Q_end,
                                  const Eigen::MatrixXd& R) {
  m_Q = Q;
  m_R = R;
  m_Q_end = Q_end;
  return 0;
}

int IlqrOptimizer::setEndWightMatrix(const Eigen::MatrixXd& Q_end) {
  m_Q_end = Q_end;
  return 0;
}

double IlqrOptimizer::getBestCost() { return m_best_cost; }

Eigen::VectorXd IlqrOptimizer::flatten(const Eigen::MatrixXd& M) {
  Eigen::MatrixXd M_T = M.transpose();
  return Eigen::Map<Eigen::VectorXd>(M_T.data(), M_T.size());
}

Eigen::MatrixXd IlqrOptimizer::reshape(Eigen::VectorXd& v, unsigned int cols,
                                       unsigned int rows) {
  return Eigen::Map<Eigen::MatrixXd>(v.data(), rows, cols).transpose();
}

Eigen::MatrixXd IlqrOptimizer::rollout(const Eigen::VectorXd& x_init,
                                       const Eigen::MatrixXd& U) {
  Eigen::MatrixXd X = Eigen::MatrixXd::Zero(m_step, m_dim_x);
  X.row(0) = x_init;
  for (unsigned int i = 0; i < m_step - 1; i++) {
    Eigen::VectorXd next_state;
    m_vehicle_model_ptr->toNextState(X.row(i), U.row(i), m_time_interval,
                                     next_state);
    X.row(i + 1) = next_state;
  }
  return X;
}

Eigen::MatrixXd IlqrOptimizer::getSu(const Eigen::MatrixXd& X,
                                     const Eigen::MatrixXd& U) {
  Eigen::MatrixXd Su =
      Eigen::MatrixXd::Zero(m_dim_x * m_step, m_dim_u * (m_step - 1));
  for (unsigned int j = 0; j < m_step - 1; j++) {
    Eigen::MatrixXd temp_b;
    int res = m_vehicle_model_ptr->getMatrixB(X.row(j), U.row(j),
                                              m_time_interval, temp_b);
    Su.block((j + 1) * m_dim_x, j * m_dim_u, m_dim_x, m_dim_u) = temp_b;
    for (unsigned int i = 0; i < m_step - 2 - j; i++) {
      Eigen::MatrixXd temp_a;
      m_vehicle_model_ptr->getMatrixA(X.row(i + j + 1), U.row(i + j + 1),
                                      m_time_interval, temp_a);
      Su.block((j + m_dim_u + i) * m_dim_x, j * m_dim_u, m_dim_x, m_dim_u) =
          temp_a *
          Su.block((j + 1 + i) * m_dim_x, j * m_dim_u, m_dim_x, m_dim_u);
    }
  }
  return Su;
}

double IlqrOptimizer::cost(const Eigen::MatrixXd& X, const Eigen::MatrixXd& U) {
  return (flatten(X) - m_ref_state).dot(m_Qex * (flatten(X) - m_ref_state)) +
         flatten(U).dot(m_Rex * flatten(U));
}

int IlqrOptimizer::solve(const Eigen::VectorXd& x_init,
                         std::vector<Eigen::VectorXd>& x_res,
                         Eigen::MatrixXd& u_res) {
  Eigen::MatrixXd U = Eigen::MatrixXd::Zero(m_step - 1, m_dim_u);
  for (int k = 0; k < m_max_iteration; k++) {
    Eigen::MatrixXd X = rollout(x_init, U);
    double current_cost = cost(X, U);
    Eigen::MatrixXd Su = getSu(X, U);
    Eigen::VectorXd delta_u =
        (Su.transpose() * m_Qex * Su + m_Rex)
            .llt()
            .solve(Su.transpose() * m_Qex * (m_ref_state - flatten(X)) -
                   m_Rex * flatten(U));
    // Line search
    double alpha = 1.0;
    double best_cost = current_cost;
    Eigen::MatrixXd U_best = U;
    for (unsigned int i = 0; i < 10; i++) {
      Eigen::VectorXd u_tmp = flatten(U) + alpha * delta_u;
      Eigen::MatrixXd U_tmp = reshape(u_tmp, m_step - 1, m_dim_u);
      X = rollout(x_init, U_tmp);
      double cost_tmp = cost(X, U_tmp);
      if (cost_tmp < best_cost) {
        best_cost = cost_tmp;
        U_best = U_tmp;
      }
      alpha = alpha / 2.;
    }
    std::cout << "best_cost: " << best_cost << std::endl;
    if ((flatten(U) - flatten(U_best)).squaredNorm() < m_stop_diff_cost) {
      U = U_best;
      break;
    }
    U = U_best;
  }

  Eigen::MatrixXd X = rollout(x_init, U);
  std::vector<Eigen::VectorXd> X_vec(m_step);
  for (unsigned int i = 0; i < m_step; i++) {
    X_vec[i] = X.row(i);
  }
  u_res = U;
  x_res = X_vec;
  return 0;
}

int IlqrOptimizer::optimize(const std::vector<Eigen::VectorXd>& ref_state,
                            std::vector<Eigen::VectorXd>& optimized_state,
                            std::vector<Eigen::VectorXd>& control) {
  m_step = ref_state.size();
  // set weight matrix
  m_Qex = Eigen::MatrixXd::Zero(m_step * m_vehicle_model_ptr->getDimX(),
                                m_step * m_vehicle_model_ptr->getDimX());
  for (int i = 0; i < m_step; i++) {
    m_Qex.block(
        i * m_vehicle_model_ptr->getDimX(), i * m_vehicle_model_ptr->getDimX(),
        m_vehicle_model_ptr->getDimX(), m_vehicle_model_ptr->getDimX()) = m_Q;
  }
  m_Qex.bottomRightCorner(m_vehicle_model_ptr->getDimX(),
                          m_vehicle_model_ptr->getDimX()) = m_Q_end;
  m_Rex = Eigen::MatrixXd::Zero((m_step - 1) * m_vehicle_model_ptr->getDimU(),
                                (m_step - 1) * m_vehicle_model_ptr->getDimU());
  for (int i = 0; i < m_step - 1; i++) {
    m_Rex.block(
        i * m_vehicle_model_ptr->getDimU(), i * m_vehicle_model_ptr->getDimU(),
        m_vehicle_model_ptr->getDimU(), m_vehicle_model_ptr->getDimU()) = m_R;
  }
  m_ref_state = Eigen::VectorXd::Zero(m_step * m_dim_x);
  for (int k = 0; k < ref_state.size(); ++k) {
    m_ref_state.segment(k * m_dim_x, ref_state[k].size()) =
        ref_state[k].head(ref_state[k].size());
  }
  Eigen::MatrixXd temp_control;

  std::chrono::system_clock::time_point before =
      std::chrono::system_clock::now();
  auto before_micros =
      std::chrono::time_point_cast<std::chrono::microseconds>(before)
          .time_since_epoch()
          .count();

  solve(m_vehicle_model_ptr->getCurStateVec(), optimized_state, temp_control);
  std::chrono::system_clock::time_point after =
      std::chrono::system_clock::now();
  auto after_micros =
      std::chrono::time_point_cast<std::chrono::microseconds>(after)
          .time_since_epoch()
          .count();

  std::cout << "cal time: " << (after_micros - before_micros) / 1e6
            << std::endl;
  control.resize(m_step - 1);
  for (int i = 0; i < m_step - 1; i++) {
    control[i] = temp_control.row(i);
  }
  return 0;
}

}  // namespace Optimizer
