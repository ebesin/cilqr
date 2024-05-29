#include <gtest/gtest.h>

#include <fstream>
#include <iostream>
#include <memory>
#include <nav_msgs/msg/detail/path__struct.hpp>

#include "ilqr_optimizer.h"
#include "ilqr_path_optimizer.h"
#include "optimizer_utils.h"
#include "vehicle_model_bicycle_rear_drive_five_state.h"
#include "vehicle_model_bicycle_rear_drive_four_state.h"
#include "vehicle_model_bicycle_rear_drive_three_state.h"
#include "vehicle_model_interface.h"
#include "vehicle_state_ackermann.h"
#include "vehicle_state_interface.h"

using namespace VehicleModel;
using Eigen::Vector4d;

using Eigen::Vector4d;
std::vector<Vector4d> signals = {
    Vector4d(55.0144, -42.0004, -0.0300363, 0.25),
    Vector4d(55.0394, -42.0012, -0.0300363, 0.5),
    Vector4d(55.0744, -42.0022, -0.0300363, 0.729927),
    Vector4d(55.1194, -42.0036, -0.0300363, 0.704835),
    Vector4d(55.1594, -42.0048, -0.0300363, 0.681757),
    Vector4d(55.1993, -42.006, -0.0300363, 0.65787),
    Vector4d(55.2393, -42.0072, -0.0300363, 0.633082),
    Vector4d(55.2793, -42.0084, -0.0300363, 0.607283),
    Vector4d(55.3193, -42.0096, -0.0300363, 0.580339),
    Vector4d(55.3593, -42.0108, -0.0300363, 0.566386),
    Vector4d(55.3993, -42.012, -0.0300363, 0.566386),
    Vector4d(55.4392, -42.0132, -0.0300363, 0.566386),
    Vector4d(55.4792, -42.0144, -0.0300363, 0.566386),
    Vector4d(55.5192, -42.0156, -0.0300363, 0.566386),
    Vector4d(55.5687, -42.0171, -0.0283413, 0.552081),
    Vector4d(55.6159, -42.0182, -0.0199304, 0.529899),
    Vector4d(55.6627, -42.0189, -0.00573053, 0.506747),
    Vector4d(55.7088, -42.0187, 0.0139663, 0.482486),
    Vector4d(55.7541, -42.0175, 0.0389775, 0.456939),
    Vector4d(55.7984, -42.0151, 0.0691942, 0.429876),
    Vector4d(55.8344, -42.0121, 0.0983021, 0.405947),
    Vector4d(55.8696, -42.0081, 0.130945, 0.380517),
    Vector4d(55.9038, -42.003, 0.167081, 0.353261),
    Vector4d(55.937, -41.9967, 0.206646, 0.323717),
    Vector4d(55.9692, -41.9892, 0.249539, 0.293405),
    Vector4d(56.0003, -41.9806, 0.295608, 0.268157),
    Vector4d(56.0303, -41.9706, 0.344638, 0.247544),
    Vector4d(56.0534, -41.9618, 0.385806, 0.233969),
    Vector4d(56.0757, -41.9522, 0.428507, 0.222714),
    Vector4d(56.0973, -41.9417, 0.47253, 0.213579),
    Vector4d(56.1182, -41.9305, 0.517634, 0.206404),
    Vector4d(56.1382, -41.9185, 0.563555, 0.201059),
    Vector4d(56.1574, -41.9057, 0.61001, 0.197437),
    Vector4d(56.1759, -41.8921, 0.656707, 0.195457),
    Vector4d(56.1936, -41.8778, 0.70335, 0.195055),
    Vector4d(56.2105, -41.8628, 0.749651, 0.196182),
    Vector4d(56.2267, -41.847, 0.795337, 0.198806),
    Vector4d(56.242, -41.8306, 0.840156, 0.202906),
    Vector4d(56.2567, -41.8136, 0.883885, 0.208473),
    Vector4d(56.2705, -41.7959, 0.926333, 0.21551),
    Vector4d(56.2837, -41.7776, 0.967343, 0.224026),
    Vector4d(56.2961, -41.7588, 1.00679, 0.234041),
    Vector4d(56.3107, -41.7345, 1.05377, 0.24871),
    Vector4d(56.3242, -41.7095, 1.09805, 0.265838),
    Vector4d(56.3367, -41.6837, 1.13959, 0.285519),
    Vector4d(56.3482, -41.6574, 1.17838, 0.307874),
    Vector4d(56.3587, -41.6305, 1.21448, 0.333056),
    Vector4d(56.3684, -41.6032, 1.24795, 0.361262),
    Vector4d(56.3789, -41.5698, 1.28477, 0.399462),
    Vector4d(56.3882, -41.536, 1.31813, 0.442971),
    Vector4d(56.3964, -41.5019, 1.3482, 0.492609),
    Vector4d(56.4048, -41.4619, 1.37941, 0.559831),
    Vector4d(56.412, -41.4217, 1.4067, 0.639712),
    Vector4d(56.4182, -41.3817, 1.43036, 0.736481),
    Vector4d(56.424, -41.3362, 1.45327, 0.876572),
    Vector4d(56.4295, -41.2856, 1.47415, 1.09341),
    Vector4d(56.4343, -41.2305, 1.49179, 1.2),
    Vector4d(56.4386, -41.1713, 1.50509, 1.2),
    Vector4d(56.4421, -41.1139, 1.51281, 1.2),
    Vector4d(56.4452, -41.0582, 1.51608, 1.2),
    Vector4d(56.4482, -41.0038, 1.51643, 1.2),
    Vector4d(56.4511, -40.9502, 1.51599, 1.2),
    Vector4d(56.4541, -40.895, 1.51599, 1.2),
    Vector4d(56.4571, -40.8401, 1.51599, 1.2),
    Vector4d(56.4602, -40.7852, 1.51599, 1.2),
    Vector4d(56.4632, -40.7303, 1.51599, 1.2),
    Vector4d(56.4662, -40.6753, 1.51599, 1.2),
    Vector4d(56.4692, -40.6204, 1.51599, 1.2),
    Vector4d(56.4722, -40.5655, 1.51599, 1.2),
    Vector4d(56.4752, -40.5106, 1.51599, 1.2),
    Vector4d(56.4782, -40.4557, 1.51599, 1.2),
    Vector4d(56.4812, -40.4008, 1.51599, 1.2),
    Vector4d(56.4843, -40.3458, 1.51599, 1.2),
    Vector4d(56.4873, -40.2909, 1.51599, 1.2),
    Vector4d(56.4903, -40.236, 1.51599, 1.2),
    Vector4d(56.4933, -40.1811, 1.51599, 1.2),
    Vector4d(56.4963, -40.1262, 1.51599, 1.2),
    Vector4d(56.4993, -40.0712, 1.51599, 1.2),
    Vector4d(56.5023, -40.0163, 1.51599, 1.2),
    Vector4d(56.5053, -39.9614, 1.51599, 1.2),
    Vector4d(56.5084, -39.9065, 1.51599, 1.2),
    Vector4d(56.5114, -39.8516, 1.51599, 1.2),
    Vector4d(56.5144, -39.7967, 1.51599, 1.2),
    Vector4d(56.5174, -39.7417, 1.51599, 1.2),
    Vector4d(56.5204, -39.6868, 1.51599, 1.2),
    Vector4d(56.5234, -39.6319, 1.51599, 1.2),
    Vector4d(56.5264, -39.577, 1.51599, 1.2),
    Vector4d(56.5294, -39.5221, 1.51599, 1.2),
};

TEST(OPTIMIZER, ILQROPTIMIZER_PATH) {
  std::shared_ptr<VehicleModelInterface> vehicle_model =
      std::make_shared<VehicleModelBicycleRearDriveThreeState>(0.2);
  std::shared_ptr<VehicleStateAckermann> max_state_ptr =
      std::make_shared<VehicleStateAckermann>();
  max_state_ptr->base_state_.twist.twist.linear.x = 1.0;
  max_state_ptr->acc_vx_ = 1.0;
  vehicle_model->setMaxState(max_state_ptr);

  // set ref_state
  int nbSteps = signals.size();
  std::vector<Eigen::VectorXd> ref_state;
  ref_state.resize(nbSteps);
  for (int i = 0; i < nbSteps; i++) {
    ref_state[i] = signals[i].head(3);
  }
  nav_msgs::msg::Path path;
  OptimizerUtils::convertToMsg(ref_state, path);

  // state vehicle state
  std::shared_ptr<VehicleStateAckermann> cur_state_ptr =
      std::make_shared<VehicleStateAckermann>();
  cur_state_ptr->base_state_.pose.pose = path.poses.front().pose;
  cur_state_ptr->base_state_.twist.twist.linear.x = 0.0;
  vehicle_model->setCurState(cur_state_ptr);
  std::shared_ptr<VehicleStateAckermann> end_state_ptr =
      std::make_shared<VehicleStateAckermann>();
  end_state_ptr->base_state_.pose.pose = path.poses.back().pose;
  end_state_ptr->base_state_.twist.twist.linear.x = 0.0;
  vehicle_model->setEndState(end_state_ptr);
  // set weight matrix
  double Q_t = 1e1;
  double Q_T = 1e4;
  double R_t = 1e-2;
  Eigen::MatrixXd Q, Q_end, R;
  Q = Q_t * Eigen::MatrixXd::Identity(vehicle_model->getDimX(),
                                      vehicle_model->getDimX());
  Q_end = Q_T * Eigen::MatrixXd::Identity(vehicle_model->getDimX(),
                                          vehicle_model->getDimX());
  R = R_t * Eigen::MatrixXd::Identity(vehicle_model->getDimU(),
                                      vehicle_model->getDimU());

  // create optimizer
  Optimizer::IlqrPathOptimizer optimizer(vehicle_model, Q, Q_end, R);
  nav_msgs::msg::Path opt_path;
  optimizer.optimize(path, opt_path);
  std::cout << "test...." << std::endl;
}

TEST(OPTIMIZER, ILQROPTIMIZER_PATH_4_STATE) {
  std::shared_ptr<VehicleModelInterface> vehicle_model =
      std::make_shared<VehicleModelBicycleRearDriveFiveState>(0.2);
  std::shared_ptr<VehicleStateAckermann> max_state_ptr =
      std::make_shared<VehicleStateAckermann>();
  max_state_ptr->base_state_.twist.twist.linear.x = 1.0;
  max_state_ptr->acc_vx_ = 1.0;
  vehicle_model->setMaxState(max_state_ptr);

  // set ref_state
  int nbSteps = signals.size();
  std::vector<Eigen::VectorXd> ref_state;
  ref_state.resize(nbSteps);
  for (int i = 0; i < nbSteps; i++) {
    ref_state[i] = Eigen::VectorXd::Zero(vehicle_model->getDimX());
    ref_state[i].head(4) = signals[i].head(4);
  }
  nav_msgs::msg::Path path;
  OptimizerUtils::convertToMsg(ref_state, path);
  // state vehicle state
  std::shared_ptr<VehicleStateAckermann> cur_state_ptr =
      std::make_shared<VehicleStateAckermann>();
  cur_state_ptr->base_state_.pose.pose = path.poses.front().pose;
  cur_state_ptr->base_state_.twist.twist.linear.x = 0.25;
  vehicle_model->setCurState(cur_state_ptr);
  std::shared_ptr<VehicleStateAckermann> end_state_ptr =
      std::make_shared<VehicleStateAckermann>();
  end_state_ptr->base_state_.pose.pose = path.poses.back().pose;
  end_state_ptr->base_state_.twist.twist.linear.x = 1.2;
  vehicle_model->setEndState(end_state_ptr);
  // set weight matrix
  double Q_t = 1e1;
  double Q_T = 1e4;
  double R_t = 1e-2;
  Eigen::MatrixXd Q, Q_end, R;
  Q = Q_t * Eigen::MatrixXd::Identity(vehicle_model->getDimX(),
                                      vehicle_model->getDimX());
  Q(vehicle_model->getDimX() - 1, vehicle_model->getDimX() - 1) = 0;
  Q(vehicle_model->getDimX() - 2, vehicle_model->getDimX() - 2) = 0;
  Q_end = Q_T * Eigen::MatrixXd::Identity(vehicle_model->getDimX(),
                                          vehicle_model->getDimX());
  Q_end(vehicle_model->getDimX() - 1, vehicle_model->getDimX() - 1) = 0;
  Q_end(vehicle_model->getDimX() - 2, vehicle_model->getDimX() - 2) = 0;
  R = R_t * Eigen::MatrixXd::Identity(vehicle_model->getDimU(),
                                      vehicle_model->getDimU());

  // create optimizer
  // Optimizer::IlqrPathOptimizer optimizer(vehicle_model, Q, Q_end, R);
  Optimizer::IlqrOptimizer optimizer(vehicle_model, Q, Q_end, R);
  optimizer.setTimeInterval(0.1);
  nav_msgs::msg::Path opt_path;
  std::vector<Eigen::VectorXd> optimized_state;
  std::vector<Eigen::VectorXd> control;
  optimizer.optimize(ref_state, optimized_state, control);
  std::cout << "optimized_state: " << std::endl;
  for (auto o : optimized_state) {
    std::cout << o << std::endl;
  }
  // optimizer.optimize(path, opt_path);
  std::cout << "test...." << std::endl;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
