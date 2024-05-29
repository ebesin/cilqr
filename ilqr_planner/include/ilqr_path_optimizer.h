#pragma once

#include "ilqr_optimizer.h"
#include "nav_msgs/msg/path.hpp"

namespace Optimizer {
class IlqrPathOptimizer : public IlqrOptimizer {
 public:
  ~IlqrPathOptimizer() = default;
  IlqrPathOptimizer(const std::shared_ptr<VehicleModel::VehicleModelInterface>&
                        vehicle_model_ptr,
                    const Eigen::MatrixXd& Q, const Eigen::MatrixXd& Q_end,
                    const Eigen::MatrixXd& R);

  int optimize(const nav_msgs::msg::Path& ref_path,
               nav_msgs::msg::Path& opt_path);

  int optimize(const geometry_msgs::msg::Pose begin_pose,
               const geometry_msgs::msg::Pose end_pose,
               nav_msgs::msg::Path& opt_path);
};
}  // namespace Optimizer
