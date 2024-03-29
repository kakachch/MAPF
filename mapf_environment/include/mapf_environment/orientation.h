/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-15 19:36:42
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 23:34:51
 */

#pragma once
#ifndef MAPF_ENVIRONMENT_ORIENTATION_H
#define MAPF_ENVIRONMENT_ORIENTATION_H

#include <vector>
#include <cmath>
//#define M_PI 3.1415926535
namespace mapf_environment
{
    const int LEFT = 0;
    const int UP = 1;
    const int RIGHT = 2;
    const int DOWN = 3;
    
    bool getNextOrientation(int current_orientation, std::vector<int> &next_orientation_vec, std::vector<int> &delta_x, std::vector<int> &delta_y);

    bool yawToOrientation(double yaw, int &orientation);

    bool orientationToYaw(int orientation, double &yaw);

}

#endif