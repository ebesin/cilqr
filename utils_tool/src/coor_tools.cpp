#include "coor_tools.h"

#include <cmath>
#include <geometry_msgs/msg/detail/point__struct.hpp>
#include <geometry_msgs/msg/detail/pose__struct.hpp>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace utils_tool {

double deg2Rad(double angle) { return angle / 180.0 * M_PI; }

double rad2Deg(double radian) { return radian * 180.0 / M_PI; }

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

geometry_msgs::msg::Quaternion createQuaternionMsgFromYaw(double yaw) {
  tf2::Quaternion q;
  q.setRPY(0, 0, yaw);
  return tf2::toMsg(q);
}

double getYawFromQuaternion(const geometry_msgs::msg::Quaternion& q) {
  tf2::Quaternion tf_q;
  tf2::fromMsg(q, tf_q);
  tf2::Matrix3x3 m(tf_q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  return yaw;
}

double getHeadingFromVector(double x, double y) {
  double theta = acos(x / sqrt(x * x + y * y));
  theta = y >= 0 ? theta : -theta;
  return theta;
}

double mod(const geometry_msgs::msg::Pose& a) {
  return sqrt((a.position.x * a.position.x) + (a.position.y * a.position.y));
}

double mod(const geometry_msgs::msg::PoseStamped& a) {
  return sqrt((a.pose.position.x * a.pose.position.x) +
              (a.pose.position.y * a.pose.position.y));
}

double dir(const geometry_msgs::msg::Pose& a) {
  return atan2(a.position.y, a.position.x);
}

double dir(const geometry_msgs::msg::PoseStamped& a) {
  return atan2(a.pose.position.y, a.pose.position.x);
}

double normalizeDeg(const double& deg) { return atan2(sin(deg), cos(deg)); }

geometry_msgs::msg::PoseStamped absoluteDifference(
    const geometry_msgs::msg::PoseStamped& p1,
    const geometry_msgs::msg::PoseStamped& p2) {
  geometry_msgs::msg::PoseStamped delta = p1 - p2;
  double s = sin(getYawFromQuaternion(p2.pose.orientation)),
         c = cos(getYawFromQuaternion(p2.pose.orientation));
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = c * delta.pose.position.x + s * delta.pose.position.y;
  pose.pose.position.y = -s * delta.pose.position.x + c * delta.pose.position.y;
  pose.pose.orientation = delta.pose.orientation;
  return pose;
}

geometry_msgs::msg::PoseStamped absoluteSum(
    const geometry_msgs::msg::PoseStamped& p1,
    const geometry_msgs::msg::PoseStamped& p2) {
  double s = sin(getYawFromQuaternion(p1.pose.orientation)),
         c = cos(getYawFromQuaternion(p1.pose.orientation));
  geometry_msgs::msg::PoseStamped pose;
  pose.pose.position.x = c * p2.pose.position.x - s * p2.pose.position.y;
  pose.pose.position.y = s * p2.pose.position.x + c * p2.pose.position.y;
  pose.pose.orientation = p2.pose.orientation;
  return pose + p1;
}

geometry_msgs::msg::Pose absoluteSum(const geometry_msgs::msg::Pose& p1,
                                     const geometry_msgs::msg::Pose& p2) {
  double s = sin(getYawFromQuaternion(p1.orientation)),
         c = cos(getYawFromQuaternion(p1.orientation));
  geometry_msgs::msg::Pose pose;
  pose.position.x = c * p2.position.x - s * p2.position.y;
  pose.position.y = s * p2.position.x + c * p2.position.y;
  pose.orientation = p2.orientation;
  return pose + p1;
}

geometry_msgs::msg::Point absoluteSum(const geometry_msgs::msg::PoseStamped& p1,
                                      const geometry_msgs::msg::Point& p2) {
  double s = sin(getYawFromQuaternion(p1.pose.orientation)),
         c = cos(getYawFromQuaternion(p1.pose.orientation));
  geometry_msgs::msg::Point point;
  point.x = c * p2.x - s * p2.y;
  point.y = s * p2.x + c * p2.y;
  return point + p1.pose.position;
}

}  // namespace utils_tool

geometry_msgs::msg::Pose operator-(const geometry_msgs::msg::Pose& a,
                                   const geometry_msgs::msg::Pose& b) {
  double delta_theta = utils_tool::getYawFromQuaternion(a.orientation) -
                       utils_tool::getYawFromQuaternion(b.orientation);
  double delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
  geometry_msgs::msg::Pose res;
  res.position.x = a.position.x - b.position.x;
  res.position.y = a.position.y - b.position.y;
  res.orientation = utils_tool::createQuaternionMsgFromYaw(delta_theta_norm);
  return res;
}

geometry_msgs::msg::PoseStamped operator-(
    const geometry_msgs::msg::PoseStamped& a,
    const geometry_msgs::msg::PoseStamped& b) {
  double delta_theta = utils_tool::getYawFromQuaternion(a.pose.orientation) -
                       utils_tool::getYawFromQuaternion(b.pose.orientation);
  double delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
  geometry_msgs::msg::PoseStamped res;
  res.pose.position.x = a.pose.position.x - b.pose.position.x;
  res.pose.position.y = a.pose.position.y - b.pose.position.y;
  res.pose.orientation =
      utils_tool::createQuaternionMsgFromYaw(delta_theta_norm);
  return res;
}

geometry_msgs::msg::Pose operator+(const geometry_msgs::msg::Pose& a,
                                   const geometry_msgs::msg::Pose& b) {
  double delta_theta = utils_tool::getYawFromQuaternion(a.orientation) +
                       utils_tool::getYawFromQuaternion(b.orientation);
  double delta_theta_norm = atan2(sin(delta_theta), cos(delta_theta));
  geometry_msgs::msg::Pose res;
  res.position.x = a.position.x + b.position.x;
  res.position.y = a.position.y + b.position.y;
  res.orientation = utils_tool::createQuaternionMsgFromYaw(delta_theta_norm);
  return res;
}

geometry_msgs::msg::PoseStamped operator+(
    const geometry_msgs::msg::PoseStamped& a,
    const geometry_msgs::msg::PoseStamped& b) {
  double theta = utils_tool::getYawFromQuaternion(a.pose.orientation) +
                 utils_tool::getYawFromQuaternion(b.pose.orientation);
  double theta_norm = atan2(sin(theta), cos(theta));
  geometry_msgs::msg::PoseStamped res;
  res.pose.position.x = a.pose.position.x + b.pose.position.x;
  res.pose.position.y = a.pose.position.y + b.pose.position.y;
  res.pose.orientation = utils_tool::createQuaternionMsgFromYaw(theta_norm);
  return res;
}

geometry_msgs::msg::Point operator+(const geometry_msgs::msg::Point& a,
                                    const geometry_msgs::msg::Point& b) {
  geometry_msgs::msg::Point res;
  res.x = a.x + b.x;
  res.y = a.y + b.y;
  return res;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Point& p) {
  os << p.x << " " << p.y;
  return os;
}

std::ostream& operator<<(std::ostream& os, const geometry_msgs::msg::Pose& p) {
  os << p.position.x << " " << p.position.y << " "
     << utils_tool::getYawFromQuaternion(p.orientation);
  return os;
}

std::ostream& operator<<(std::ostream& os,
                         const geometry_msgs::msg::PoseStamped& p) {
  os << p.pose.position.x << " " << p.pose.position.y << " "
     << utils_tool::getYawFromQuaternion(p.pose.orientation);
  return os;
}

std::ostream& operator<<(std::ostream& os, const nav_msgs::msg::Path& p) {
  for (auto p : p.poses) {
    os << p;
  }
  return os;
}
