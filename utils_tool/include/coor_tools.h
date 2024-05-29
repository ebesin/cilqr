//        _
//       | |
//     __| |_      ____ _ _   _ _ __   ___
//    / _` \ \ /\ / / _` | | | | '_ \ / _ \
//   | (_| |\ V  V / (_| | |_| | | | |  __/
//    \__,_| \_/\_/ \__,_|\__, |_| |_|\___|
//                         __/ |
//                        |___/

/**
 * @file coor_transform.hpp
 * @author dwayne (ebesin@outlook.com)
 * @brief 坐标转换
 * @version 0.1
 * @date 2022-04-30
 *
 * @copyright Copyright (c) 2022
 *
 */

#pragma once
#include <cmath>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "nav_msgs/msg/path.hpp"

geometry_msgs::msg::Pose operator-(const geometry_msgs::msg::Pose& a,
                                   const geometry_msgs::msg::Pose& b);

geometry_msgs::msg::PoseStamped operator-(
    const geometry_msgs::msg::PoseStamped& a,
    const geometry_msgs::msg::PoseStamped& b);

geometry_msgs::msg::Pose operator+(const geometry_msgs::msg::Pose& a,
                                   const geometry_msgs::msg::Pose& b);

geometry_msgs::msg::PoseStamped operator+(
    const geometry_msgs::msg::PoseStamped& a,
    const geometry_msgs::msg::PoseStamped& b);

geometry_msgs::msg::Point operator+(const geometry_msgs::msg::Point& a,
                                    const geometry_msgs::msg::Point& b);

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point& p);

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Pose& p);

std::ostream& operator<<(std::ostream& os,
                         const geometry_msgs::msg::PoseStamped& p);

std::ostream& operator<<(std::ostream& os, const nav_msgs::msg::Path& p);

namespace utils_tool {

constexpr static const double a = 6378137;
constexpr static const double b = 6356752.3142;
constexpr static const double f = (a - b) / a;
constexpr static const double e_sq = f * (2 - f);

#ifndef WGS84_CLASS_H_
#define WGS84_CLASS_H_

class WGS84 {
 private:
  double latitude_;
  double longitude_;
  double altitude_;

 public:
  WGS84(double longitude, double latitude, double altitude)
      : latitude_(latitude), longitude_(longitude), altitude_(altitude){};
  WGS84(){};

  void setLatitude(double latitude) { latitude_ = latitude; };

  void setLongitude(double longitude) { longitude_ = longitude; };

  void setAltitude(double altitude) { altitude_ = altitude; };

  double getLatitude() const { return latitude_; };

  double getLongitude() const { return longitude_; };

  double getAltitude() const { return altitude_; };
};
#endif

#ifndef ECEF_H_
#define ECEF_H_

class ECEF {
 private:
  /* data */
  double x_;
  double y_;
  double z_;

 public:
  ECEF(double x, double y, double z) : x_(x), y_(y), z_(z){};
  ECEF(){};

  void setX(double x) { x_ = x; };

  void setY(double y) { y_ = y; };

  void setZ(double z) { z_ = z; };

  double getX() const { return x_; };

  double getY() const { return y_; };

  double getZ() const { return z_; };
};
#endif

#ifndef ENU_H_
#define ENU_H_

class ENU {
  /* data */
  double x_east_;
  double y_north_;
  double z_up_;

 public:
  ENU(double xEast, double yNorth, double zUp)
      : x_east_(xEast), y_north_(yNorth), z_up_(zUp){};
  ENU(){};

  void setXEast(double xEast) { x_east_ = xEast; };

  void setYNorth(double yNorth) { y_north_ = yNorth; };

  void setZUp(double zUp) { z_up_ = zUp; };

  double getXEast() const { return x_east_; };

  double getYNorth() const { return y_north_; };

  double getZUp() const { return z_up_; };
};
#endif

/**
 * @brief 角度制转弧度制
 *
 * @param angle
 * @return double
 */
double deg2Rad(double angle);

/**
 * @brief 弧度制转角度制
 *
 * @param radian
 * @return double
 */
double rad2Deg(double radian);

/**
 * @brief 将wgs84坐标转换为ecef坐标
 *
 * @param wgs84 需要转换的坐标
 * @return ECEF 转换后的坐标
 */
ECEF wgs84ToEcef(const WGS84& source);

/**
 * @brief 将wgs84坐标转换为enu坐标
 *
 * @param origin 原点坐标
 * @param current 当前坐标
 * @return ENU 当前坐标以原点坐标为原点转换后的enu坐标
 */
ENU wgs84ToEnu(const WGS84& origin, const WGS84& current);

/**
 * @brief enu转ecef
 *
 * @param origin 原点坐标
 * @param enu 当前坐标
 * @return ECEF 当前坐标以原点坐标为原点转换后的ECEF坐标
 */
ECEF enuToEcef(const WGS84& origin, const ENU& enu);

/**
 * @brief ecef转wgs84
 *
 * @param ecef
 * @return WGS84
 */
WGS84 ecefToWgs84(const ECEF& ecef);

WGS84 enuToWgs84(const ENU& enu, const WGS84& origin);

/**
 * @description  : 根据航向创建四元数
 * @param         {double} yaw:
 * @return        {*}
 */
geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw);

/**
 * @description  : 根据四元数获取航向
 * @param         {Quaternion&} q:
 * @return        {*}
 */
double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q);

/**
 * @description  : 获取向量角度
 * @param         {double} x:
 * @param         {double} y:
 * @return        {*}
 */
double getHeadingFromVector(double x, double y);

double mod(const geometry_msgs::msg::Pose& a);
double mod(const geometry_msgs::msg::PoseStamped& a);
double dir(const geometry_msgs::msg::Pose& a);
double dir(const geometry_msgs::msg::PoseStamped& a);

double normalizeDeg(const double& deg);

// p1在p2坐标系下的位置
geometry_msgs::msg::PoseStamped absoluteDifference(
    const geometry_msgs::msg::PoseStamped& p1,
    const geometry_msgs::msg::PoseStamped& p2);

// 在p1的基础上运动p2后的位置，p2在p1的坐标系下
geometry_msgs::msg::PoseStamped absoluteSum(
    const geometry_msgs::msg::PoseStamped& p1,
    const geometry_msgs::msg::PoseStamped& p2);

// p2为p1局部坐标系下的点，求p2在全局坐标系下的位置
geometry_msgs::msg::Point absoluteSum(const geometry_msgs::msg::PoseStamped& p1,
                                      const geometry_msgs::msg::Point& p2);

// p2为p1局部坐标系下的点，求p2在全局坐标系下的位置
geometry_msgs::msg::Pose absoluteSum(const geometry_msgs::msg::Pose& p1,
                                      const geometry_msgs::msg::Pose& p2);

}  // namespace utils_tool
