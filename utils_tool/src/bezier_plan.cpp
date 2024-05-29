#include "bezier_plan.h"

#include <cmath>
#include <cstdint>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <vector>

#include "coor_tools.h"

using namespace utils_tool;

static constexpr double kQuarter = 0.25;
static constexpr double kHalf = 0.5;
static constexpr double kThreeFourths = 0.75;

static constexpr int kStrainEnergyNum = 12;

static inline void ChangeMinMax(const double& temp, double& min, double& max) {
  if (temp > max) {
    max = temp;
  }
  if (temp < min) {
    min = temp;
  }
}

void kMatch(const PoseStamped& begin_pose, const PoseStamped& end_pose,
            const double& step, double& k1, double& k2) {
  static constexpr double kDiffK{0.05};
  static constexpr double kRange = 10.0;
  static constexpr double kMatchMin = 0.2;
  static constexpr double kMatchMax = 0.8;

  double path_dist = mod(begin_pose - end_pose);
  double delta_t = step / path_dist;
  double min_sum{DBL_MAX};
  PoseStamped bezier_p1;
  PoseStamped bezier_p2;
  PoseStamped prev_temp;
  for (double i = kMatchMin; i < kMatchMax; i += kDiffK) {
    for (double j = kMatchMin; j < kMatchMax; j += kDiffK) {
      double diff_length1 = i * path_dist;
      double diff_length2 = j * path_dist;
      bezier_p1.pose.position.x =
          diff_length1 *
              cos(getYawFromQuaternion(begin_pose.pose.orientation)) +
          begin_pose.pose.position.x;
      bezier_p1.pose.position.y =
          diff_length1 *
              sin(getYawFromQuaternion(begin_pose.pose.orientation)) +
          begin_pose.pose.position.y;
      bezier_p2.pose.position.x =
          diff_length2 *
              cos(getYawFromQuaternion(end_pose.pose.orientation) + M_PI) +
          end_pose.pose.position.x;
      bezier_p2.pose.position.y =
          diff_length2 *
              sin(getYawFromQuaternion(end_pose.pose.orientation) + M_PI) +
          end_pose.pose.position.y;
      double min_r{kRange};
      double max_r{-kRange};
      prev_temp = begin_pose;
      double diff_prev{0};
      for (double t = delta_t; t <= 1.0; t += delta_t) {
        PoseStamped temp_point;
        temp_point.pose.position.x =
            begin_pose.pose.position.x * pow(1 - t, 3) +
            3 * bezier_p1.pose.position.x * pow(1 - t, 2) * t +
            3 * bezier_p2.pose.position.x * (1 - t) * pow(t, 2) +
            end_pose.pose.position.x * pow(t, 3);
        temp_point.pose.position.y =
            begin_pose.pose.position.y * pow(1 - t, 3) +
            3 * bezier_p1.pose.position.y * pow(1 - t, 2) * t +
            3 * bezier_p2.pose.position.y * (1 - t) * pow(t, 2) +
            end_pose.pose.position.y * pow(t, 3);
        if (fabs(t - delta_t) < 0.0001) {
          diff_prev = dir(temp_point - prev_temp);
          prev_temp = temp_point;
        } else {
          double diff = dir(temp_point - prev_temp);
          double temp_r = (normalizeDeg(diff - diff_prev));
          diff_prev = diff;
          prev_temp = temp_point;
          ChangeMinMax(temp_r, min_r, max_r);
        }
      }
      if (max_r - min_r < min_sum) {
        min_sum = max_r - min_r;
        k1 = i;
        k2 = j;
      }
    }
  }
}

double doBezierPlan(const PoseStamped& begin_pose, const PoseStamped& end_pose,
                    Path& path, const double& step) {
  std::vector<PoseStamped> ctrl_list(6);
  ctrl_list.at(0) = begin_pose;
  ctrl_list.at(5) = end_pose;
  double path_dist = mod(begin_pose - end_pose);
  path.poses.clear();
  if (path_dist < step) {
    path.poses.emplace_back(end_pose);
    return path_dist;
  }
  if (!fixedModePlan(begin_pose, end_pose, ctrl_list)) {
    std::array<double, 4> vk = {};
    vk[0] = 0.1;
    vk[1] = 0.4;
    vk[2] = 0.4;
    vk[3] = 0.1;
    double k1, k2;
    kMatch(begin_pose, end_pose, step * 2, vk[1], vk[2]);

    double diff_length1 = vk[1] / 3 * path_dist;
    double diff_length2 = (vk[1] * 1.3) * path_dist;

    double diff_length3 = (vk[2] * 1.3) * path_dist;
    double diff_length4 = vk[2] / 3 * path_dist;
    std::cout << vk[1] / 3 << " " << vk[1] * 1.3 << " " << vk[2] * 1.3 << " "
              << vk[2] / 3 << "\n";
    ctrl_list.at(1).pose.position.x =
        diff_length1 * cos(getYawFromQuaternion(begin_pose.pose.orientation)) +
        begin_pose.pose.position.x;
    ctrl_list.at(1).pose.position.y =
        diff_length1 * sin(getYawFromQuaternion(begin_pose.pose.orientation)) +
        begin_pose.pose.position.y;
    ctrl_list.at(2).pose.position.x =
        diff_length2 * cos(getYawFromQuaternion(begin_pose.pose.orientation)) +
        begin_pose.pose.position.x;
    ctrl_list.at(2).pose.position.y =
        diff_length2 * sin(getYawFromQuaternion(begin_pose.pose.orientation)) +
        begin_pose.pose.position.y;
    ctrl_list.at(3).pose.position.x =
        diff_length3 *
            cos(getYawFromQuaternion(end_pose.pose.orientation) + M_PI) +
        end_pose.pose.position.x;
    ctrl_list.at(3).pose.position.y =
        diff_length3 *
            sin(getYawFromQuaternion(end_pose.pose.orientation) + M_PI) +
        end_pose.pose.position.y;
    ctrl_list.at(4).pose.position.x =
        diff_length4 *
            cos(getYawFromQuaternion(end_pose.pose.orientation) + M_PI) +
        end_pose.pose.position.x;
    ctrl_list.at(4).pose.position.y =
        diff_length4 *
            sin(getYawFromQuaternion(end_pose.pose.orientation) + M_PI) +
        end_pose.pose.position.y;
    std::cout << "其它 "
              << " " << ctrl_list.at(1) << " " << ctrl_list.at(2) << " "
              << ctrl_list.at(3) << " " << ctrl_list.at(4) << "\n";
  }
  return doBezierFiveOrder(ctrl_list, step, path);
}

bool fixedModePlan(const PoseStamped& begin_pose, const PoseStamped& end_pose,
                   std::vector<PoseStamped>& ctrl_list) {
  PoseStamped local_target = absoluteDifference(end_pose, begin_pose);
  if (fabs(normalizeDeg(getYawFromQuaternion(
          (begin_pose - end_pose).pose.orientation))) < deg2Rad(5)) {
    // 变道
    double dist = fabs(local_target.pose.position.x);
    Point temp;
    temp.x = kQuarter * dist;
    ctrl_list.at(1).pose.position = absoluteSum(begin_pose, temp);
    temp.x = kHalf * dist;
    ctrl_list.at(2).pose.position = absoluteSum(begin_pose, temp);
    temp.x = -kHalf * dist;
    ctrl_list.at(3).pose.position = absoluteSum(end_pose, temp);
    temp.x = -kQuarter * dist;
    ctrl_list.at(4).pose.position = absoluteSum(end_pose, temp);
    std::cout << "变道 "
              << " " << ctrl_list.at(1) << " " << ctrl_list.at(2) << " "
              << ctrl_list.at(3) << " " << ctrl_list.at(4) << "\n";
    return true;
  }
  if (fabs(normalizeDeg(
          getYawFromQuaternion((begin_pose - end_pose).pose.orientation) +
          M_PI)) < deg2Rad(5) &&
      fabs(local_target.pose.position.x) < 0.1 &&
      fabs(local_target.pose.position.y) > 0.3) {
    // 掉头
    double dist = fabs(local_target.pose.position.y);
    Point temp;
    temp.x = kQuarter * dist;
    ctrl_list.at(1).pose.position = absoluteSum(begin_pose, temp);
    temp.x = kHalf * dist;
    ctrl_list.at(2).pose.position = absoluteSum(begin_pose, temp);
    temp.x = -kHalf * dist;
    ctrl_list.at(3).pose.position = absoluteSum(end_pose, temp);
    temp.x = -kQuarter * dist;
    ctrl_list.at(4).pose.position = absoluteSum(end_pose, temp);
    std::cout << "掉头 "
              << " " << ctrl_list.at(1) << " " << ctrl_list.at(2) << " "
              << ctrl_list.at(3) << " " << ctrl_list.at(4) << "\n";
    return true;
  }
  if (fabs(normalizeDeg(getYawFromQuaternion(
          (begin_pose - end_pose).pose.orientation))) < deg2Rad(95) &&
      fabs(normalizeDeg(getYawFromQuaternion(
          (begin_pose - end_pose).pose.orientation))) > deg2Rad(85) &&
      fabs(local_target.pose.position.x) > 0.1 &&
      fabs(local_target.pose.position.y) > 0.1) {
    // 转弯
    double dist1 = fabs(local_target.pose.position.x);
    double dist2 = fabs(local_target.pose.position.y);
    Point temp;
    temp.x = kQuarter * dist1;
    ctrl_list.at(1).pose.position = absoluteSum(begin_pose, temp);
    temp.x = dist1;
    ctrl_list.at(2).pose.position = absoluteSum(begin_pose, temp);
    temp.x = -kHalf * dist2;
    ctrl_list.at(3).pose.position = absoluteSum(end_pose, temp);
    temp.x = -kQuarter * dist2;
    ctrl_list.at(4).pose.position = absoluteSum(end_pose, temp);
    std::cout << "转弯 "
              << " " << ctrl_list.at(1) << " " << ctrl_list.at(2) << " "
              << ctrl_list.at(3) << " " << ctrl_list.at(4) << "\n";
    return true;
  }
  return false;
}

double doBezierThreeOrder(const std::vector<PoseStamped>& ctrl_list,
                          const double& step, Path& path) {}

template <typename T>
void oneBezierFiveOrder(const std::vector<PoseStamped>& ctrl_list, const T& t,
                        PoseStamped& cur_pose, T& cur_curvature) {
  T p1_1 = ctrl_list.at(0).pose.position.x;
  T p1_2 = ctrl_list.at(1).pose.position.x;
  T p1_3 = ctrl_list.at(2).pose.position.x;
  T p1_4 = ctrl_list.at(3).pose.position.x;
  T p1_5 = ctrl_list.at(4).pose.position.x;
  T p1_6 = ctrl_list.at(5).pose.position.x;

  T p2_1 = ctrl_list.at(0).pose.position.y;
  T p2_2 = ctrl_list.at(1).pose.position.y;
  T p2_3 = ctrl_list.at(2).pose.position.y;
  T p2_4 = ctrl_list.at(3).pose.position.y;
  T p2_5 = ctrl_list.at(4).pose.position.y;
  T p2_6 = ctrl_list.at(5).pose.position.y;

  cur_pose.pose.position.x = p1_6 * pow(t, 5) - p1_1 * pow(t - T(1), 5) +
                             T(5) * p1_2 * t * pow(t - T(1), 4) -
                             p1_5 * pow(t, 4) * (T(5) * t - T(5)) -
                             T(10) * p1_3 * pow(t, 2) * pow(t - T(1), 3) +
                             T(10) * p1_4 * pow(t, 3) * pow(t - T(1), 2);

  cur_pose.pose.position.y = p2_6 * pow(t, 5) - p2_1 * pow(t - T(1), 5) +
                             T(5) * p2_2 * t * pow(t - T(1), 4) -
                             p2_5 * pow(t, 4) * (T(5) * t - T(5)) -
                             T(10) * p2_3 * pow(t, 2) * pow(t - T(1), 3) +
                             T(10) * p2_4 * pow(t, 3) * pow(t - T(1), 2);

  T x_der = T(5) * p1_2 * pow(t - T(1), 4) - T(5) * p1_1 * pow(t - T(1), 4) -
            T(5) * p1_5 * pow(t, 4) + T(5) * p1_6 * pow(t, 4) +
            T(20) * p1_2 * t * pow(t - T(1), 3) -
            T(20) * p1_3 * t * pow(t - T(1), 3) +
            T(10) * p1_4 * pow(t, 3) * (T(2) * t - T(2)) -
            T(4) * p1_5 * pow(t, 3) * (T(5) * t - T(5)) -
            T(30) * p1_3 * pow(t, 2) * pow(t - T(1), 2) +
            T(30) * p1_4 * pow(t, 2) * pow(t - T(1), 2);

  T y_der = T(5) * p2_2 * pow(t - T(1), 4) - T(5) * p2_1 * pow(t - T(1), 4) -
            T(5) * p2_5 * pow(t, 4) + T(5) * p2_6 * pow(t, 4) +
            T(20) * p2_2 * t * pow(t - T(1), 3) -
            T(20) * p2_3 * t * pow(t - T(1), 3) +
            T(10) * p2_4 * pow(t, 3) * (T(2) * t - T(2)) -
            T(4) * p2_5 * pow(t, 3) * (T(5) * t - T(5)) -
            T(30) * p2_3 * pow(t, 2) * pow(t - T(1), 2) +
            T(30) * p2_4 * pow(t, 2) * pow(t - T(1), 2);

  T x_der_der = T(40) * p1_2 * pow(t - T(1), 3) -
                T(20) * p1_1 * pow(t - T(1), 3) -
                T(20) * p1_3 * pow(t - T(1), 3) + T(20) * p1_4 * pow(t, 3) -
                T(40) * p1_5 * pow(t, 3) + T(20) * p1_6 * pow(t, 3) +
                T(60) * p1_2 * t * pow(t - T(1), 2) -
                T(120) * p1_3 * t * pow(t - T(1), 2) +
                T(60) * p1_4 * t * pow(t - T(1), 2) -
                T(30) * p1_3 * pow(t, 2) * (T(2) * t - T(2)) +
                T(60) * p1_4 * pow(t, 2) * (T(2) * t - T(2)) -
                T(12) * p1_5 * pow(t, 2) * (T(5) * t - T(5));

  T y_der_der = T(40) * p2_2 * pow(t - T(1), 3) -
                T(20) * p2_1 * pow(t - T(1), 3) -
                T(20) * p2_3 * pow(t - T(1), 3) + T(20) * p2_4 * pow(t, 3) -
                T(40) * p2_5 * pow(t, 3) + T(20) * p2_6 * pow(t, 3) +
                T(60) * p2_2 * t * pow(t - T(1), 2) -
                T(120) * p2_3 * t * pow(t - T(1), 2) +
                T(60) * p2_4 * t * pow(t - T(1), 2) -
                T(30) * p2_3 * pow(t, 2) * (T(2) * t - T(2)) +
                T(60) * p2_4 * pow(t, 2) * (T(2) * t - T(2)) -
                T(12) * p2_5 * pow(t, 2) * (T(5) * t - T(5));
  cur_pose.pose.orientation = createQuaternionMsgFromYaw(atan2(y_der, x_der));
  cur_curvature = (x_der * y_der_der - y_der * x_der_der) /
                  (pow((x_der * x_der + y_der * y_der), 1.5));
}

double doBezierFiveOrder(const std::vector<PoseStamped>& ctrl_list,
                         const double& step, Path& path) {
  double path_length{0};
  if (ctrl_list.size() != 6) {
    return path_length;
  }
  path.poses.clear();
  PoseStamped temp_point;
  const auto num =
      static_cast<int>(mod(ctrl_list.front() - ctrl_list.back())) / step;
  if (num < 2) {
    temp_point.pose = ctrl_list.back().pose;
    temp_point.pose.orientation =
        (ctrl_list.at(5) - ctrl_list.at(4)).pose.orientation;
    path.poses.emplace_back(temp_point);
    return mod(ctrl_list.front() - ctrl_list.back());
  }
  path.poses.reserve(num);
  std::cout << "num: " << num << std::endl;
  auto prev_pose = ctrl_list.front();
  for (int i = 1; i < num + 1; ++i) {
    double t = static_cast<double>(i) / static_cast<double>(num);
    double curvature;
    oneBezierFiveOrder(ctrl_list, t, temp_point, curvature);
    path.poses.emplace_back(temp_point);
    path_length += mod(temp_point - prev_pose);
    prev_pose = temp_point;
  }
  // ResetPathDensity(pose_list, step);
  return path_length;
}
