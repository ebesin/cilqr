#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>

#include "ilqr_optimizer.h"

namespace Optimizer {

class IlqrPathPlanner {
 public:
  IlqrPathPlanner(const std::shared_ptr<VehicleModel::VehicleModelInterface>
                      &vehicle_model_ptr,
                  const Eigen::MatrixXd &Q, const Eigen::MatrixXd &Q_end,
                  const Eigen::MatrixXd &R);

  ~IlqrPathPlanner() = default;

  int doPlan(const geometry_msgs::msg::Pose &begin_pose,
             const geometry_msgs::msg::Pose &end_pose,
             nav_msgs::msg::Path &path);

  int doPlan(const nav_msgs::msg::Path &ref_path,
             nav_msgs::msg::Path &opt_path);

 private:
  std::shared_ptr<VehicleModel::VehicleModelInterface> vehicle_model_ptr_;
  std::shared_ptr<IlqrOptimizer> planner_ptr_;

  Eigen::MatrixXd Q_;
  Eigen::MatrixXd Q_end_;
  Eigen::MatrixXd R_;
};
}  // namespace Optimizer
