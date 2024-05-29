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
inline double deg2Rad(double angle) { return angle / 180.0 * M_PI; }

/**
 * @brief 弧度制转角度制
 *
 * @param radian
 * @return double
 */
inline double rad2Deg(double radian) { return radian * 180.0 / M_PI; }

/**
 * @brief 将wgs84坐标转换为ecef坐标
 *
 * @param wgs84 需要转换的坐标
 * @return ECEF 转换后的坐标
 */
ECEF wgs84ToEcef(const WGS84& source) {
  double lamb = deg2Rad(source.getLatitude());
  double phi = deg2Rad(source.getLongitude());
  double s = sin(lamb);
  double N = a / sqrt(1 - e_sq * s * s);

  double sin_lambda = sin(lamb);
  double cos_lambda = cos(lamb);
  double sin_phi = sin(phi);
  double cos_phi = cos(phi);

  ECEF ecef;
  ecef.setX((source.getAltitude() + N) * cos_lambda * cos_phi);
  ecef.setY((source.getAltitude() + N) * cos_lambda * sin_phi);
  ecef.setZ((source.getAltitude() + (1 - f) * (1 - f) * N) * sin_lambda);
  return ecef;
}

/**
 * @brief 将wgs84坐标转换为enu坐标
 *
 * @param origin 原点坐标
 * @param current 当前坐标
 * @return ENU 当前坐标以原点坐标为原点转换后的enu坐标
 */
ENU wgs84ToEnu(const WGS84& origin, const WGS84& current) {
  ECEF coor0 = wgs84ToEcef(origin);
  ECEF coor1 = wgs84ToEcef(current);
  double xd = coor1.getX() - coor0.getX();
  double yd = coor1.getY() - coor0.getY();
  double zd = coor1.getZ() - coor0.getZ();
  double phi = deg2Rad(origin.getLongitude());
  double lamb = deg2Rad(origin.getLatitude());
  double sin_lambda = sin(lamb);
  double cos_lambda = cos(lamb);
  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  double t = -cos_phi * xd - sin_phi * yd;
  ENU enu;
  enu.setXEast(-sin_phi * xd + cos_phi * yd);
  enu.setYNorth(t * sin_lambda + cos_lambda * zd);
  enu.setZUp(cos_lambda * cos_phi * xd + cos_lambda * sin_phi * yd +
             sin_lambda * zd);
  return enu;
}

/**
 * @brief enu转ecef
 *
 * @param origin 原点坐标
 * @param enu 当前坐标
 * @return ECEF 当前坐标以原点坐标为原点转换后的ECEF坐标
 */
ECEF enuToEcef(const WGS84& origin, const ENU& enu) {
  double lamb = deg2Rad(origin.getLatitude());  // 纬度转弧度
  double phi = deg2Rad(origin.getLongitude());  // 经度转弧度
  ECEF ecef = wgs84ToEcef(origin);
  double sin_lambda = sin(lamb);
  double cos_lambda = cos(lamb);
  double sin_phi = sin(phi);
  double cos_phi = cos(phi);
  ECEF res;
  res.setX(-sin_phi * enu.getXEast() - sin_lambda * cos_phi * enu.getYNorth() +
           cos_lambda * cos_phi * enu.getZUp() + ecef.getX());
  res.setY(cos_phi * enu.getXEast() - sin_lambda * sin_phi * enu.getYNorth() +
           cos_lambda * sin_phi * enu.getZUp() + ecef.getY());
  res.setZ(cos_lambda * enu.getYNorth() + sin_lambda * enu.getZUp() +
           ecef.getZ());
  return res;
}

/**
 * @brief ecef转wgs84
 *
 * @param ecef
 * @return WGS84
 */
WGS84 ecefToWgs84(const ECEF& ecef) {
  double x = ecef.getX();
  double y = ecef.getY();
  double z = ecef.getZ();
  double c = sqrt(((a * a) - (b * b)) / (a * a));
  double d = sqrt(((a * a) - (b * b)) / (b * b));
  double p = sqrt((x * x) + (y * y));
  double q = atan2((z * a), (p * b));
  double lon = atan2(y, x);
  double lat = atan2((z + (d * d) * b * pow(sin(q), 3)),
                     (p - (c * c) * a * pow(cos(q), 3)));
  double N = a / sqrt(1 - ((c * c) * pow(sin(lat), 2)));
  double alt = (p / cos(lat)) - N;
  WGS84 wgs84(rad2Deg(lon), rad2Deg(lat), alt);
  return wgs84;
}

WGS84 enuToWgs84(const ENU& enu, const WGS84& origin) {
  return ecefToWgs84(enuToEcef(origin, enu));
}

}  // namespace utils_tool
