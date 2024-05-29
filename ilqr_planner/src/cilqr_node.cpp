
#include "cilqr_planner.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CIlqrPlanner>("CIlqrPlanner"));
  rclcpp::shutdown();
  return 0;
}
