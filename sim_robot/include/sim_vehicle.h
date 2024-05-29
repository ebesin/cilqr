#include <mpc_msgs/msg/detail/vehicle_state__struct.hpp>
#include <fstream>
#include "mpc_msgs/msg/vehicle_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sim_robot.h"

class SimVehicle : public rclcpp::Node {
 public:
  SimVehicle(std::string name);

  ~SimVehicle() = default;

 private:
  /**
   * @description  : 声明参数
   * @return        {*}
   */
  void declareParameter();
  
  std::string get_cur_time_str();

  void cmdCallback(const geometry_msgs::msg::Twist& cmd_vel);

  void ctrlCallback(const mpc_msgs::msg::VehicleState& state_control);

  void timerCallback();

  void setCurState(const Eigen::VectorXd& state);

  Eigen::VectorXd getUFromControl(const mpc_msgs::msg::VehicleState& control);

  Eigen::VectorXd getStateFromMsg(const mpc_msgs::msg::VehicleState& msg);

  mpc_msgs::msg::VehicleState getMsgFromState(const Eigen::VectorXd& state);

  geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);
  /* data */
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber_;
  rclcpp::Subscription<mpc_msgs::msg::VehicleState>::SharedPtr ctrl_subscriber_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<mpc_msgs::msg::VehicleState>::SharedPtr
      vehicle_state_publisher_;
  rclcpp::TimerBase::SharedPtr sim_timer_;

  std::shared_ptr<VehicleModel::VehicleModelInterface> vehicle_model_ptr_;
  mpc_msgs::msg::VehicleState cur_state_;
  mpc_msgs::msg::VehicleState reveive_control_;
  Eigen::VectorXd use_control_;
  std::shared_ptr<SimRobot> sim_robot_ptr_;

  bool is_init_ = false;
  rclcpp::Time last_sim_time_;

  /*parameters*/
  double origin_x_;             // 初始点x
  double origin_y_;             // 初始点y
  double origin_phi_;           // 初始航向
  double pub_period_;           // 机器人状态发布间隔
  double min_sim_time_;         // 最小仿真时间
  double wheel_base_;           // 机器人轴距
  std::string cmd_sub_topic_;   // 控制指令接收话题名
  std::string odom_pub_toipc_;  // 里程计发布话题名
  int state_num_;               // 里程计发布话题名
  std::ofstream file_;
};
