/*
 * @Author       : dwayne
 * @Date         : 2023-06-04
 * @LastEditTime : 2023-06-28
 * @Description  :
 *
 * Copyright (c) 2023 by dwayne, All Rights Reserved.
 */
#include <cmath>
#include <iostream>

#include "coor_tools.h"

// "lng":121.03048009344441,"h":0,"lat":30.92736924265438},{"lng":121.03023433353715,"h":0,"lat":30.925893330897928

/**
 * @description  :
 * @param         {float} traj:路径数组
 * @param         {float} first_goal_x:第一条路径目标x
 * @param         {float} first_goal_y:第一条路径目标y
 * @param         {float} direction:横向偏移方向
 * @param         {float} distance:横向偏移距离
 * @param         {int  } size:路径条数
 * @return        {*}
 */
void setTraj(float traj[][4], float first_goal_x, float first_goal_y,
             float direction, float distance, int size) {
  std::cout << "direction:" << direction << " cos():" << cos(direction)
            << "  sin():" << sin(direction) << std::endl;
  for (int i = 0; i < size; i++) {
    if (i % 2 == 0) {
      traj[i][0] = i * distance * cos(direction);
      traj[i][1] = i * distance * sin(direction);
      traj[i][2] = first_goal_x + i * distance * cos(direction);
      traj[i][3] = first_goal_y + i * distance * sin(direction);
    } else {
      traj[i][0] = first_goal_x + i * distance * cos(direction);
      traj[i][1] = first_goal_y + i * distance * sin(direction);
      traj[i][2] = i * distance * cos(direction);
      traj[i][3] = i * distance * sin(direction);
    }
  }
}

int main() {
  // utils_tool::WGS84 origin(121.0304800, 30.92736924, 0);
  // utils_tool::WGS84 target(121.0302343, 30.92589333, 0);
  // 院子纵向
  // utils_tool::WGS84 origin(121.0329400, 30.9292135, 0);
  // utils_tool::WGS84 target(121.0329354, 30.9290759, 0);
  // // 四号田
  // utils_tool::WGS84 origin(121.0337227, 30.9289874, 0);
  // utils_tool::WGS84 target(121.0337678, 30.9296207, 0);
  // // // 四号田
  // utils_tool::WGS84 origin(121.0335637, 30.9290215, 0);
  // utils_tool::WGS84 target(121.0336024, 30.9295840, 0);
  // // // 四号田
  // utils_tool::WGS84 origin(121.0337310, 30.9290158, 0);
  // utils_tool::WGS84 target(121.0337719, 30.9295823, 0);
  // // // 四号田
  utils_tool::WGS84 origin(121.0336336, 30.9295960, 0);
  utils_tool::WGS84 target(121.0337631, 30.9295971, 0);
  // 院子横向
  // utils_tool::WGS84 origin(121.0327009, 30.9292545, 0);
  // utils_tool::WGS84 target(121.0329049, 30.9292493, 0);

  utils_tool::ENU enu = utils_tool::wgs84ToEnu(origin, target);
  std::cout << " enu.getXEast():" << enu.getXEast()
            << "  enu.getYNorth():" << enu.getYNorth() << std::endl;

  std::cout << "航向角: "
            << std::acos(enu.getXEast() /
                         (sqrt(enu.getXEast() * enu.getXEast() +
                               enu.getYNorth() * enu.getYNorth()))) *
                   180 / M_PI
            << std::endl;

  // 需要偏移的航向
  double theta = -1.69364 + 180;
  // double theta = -91.6507;
  // double distance  = 15.2555;
  double distance = 290;
  // double distance  = sqrt(enu.getXEast() * enu.getXEast() +
  //                        enu.getYNorth() * enu.getYNorth());
  double theta_rad = utils_tool::deg2Rad(theta);
  std::cout << "x:" << distance * std::cos(utils_tool::deg2Rad(theta))
            << "\ty:" << distance * std::sin(utils_tool::deg2Rad(theta))
            << std::endl;

  float traj[10][4];
  setTraj(traj, 10.0f, 0.0f, -90.0 / 180.0 * M_PI, 3, 5);
  for (int i = 0; i < 5; i++) {
    std::cout << "start_x:" << traj[i][0] << "  start_y:" << traj[i][1]
              << "  goal_x:" << traj[i][2] << " goal_y:" << traj[i][3]
              << std::endl;
  }
  return 0;
}
