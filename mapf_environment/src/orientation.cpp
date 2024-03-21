/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-15 23:33:30
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-16 11:17:55
 */

#include <mapf_environment/orientation.h>

namespace mapf_environment
{
    const std::vector<std::pair<int, int>> DELTA_XY = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

    bool getNextOrientation(int current_orientation, std::vector<int> &next_orientation_vec, std::vector<int> &delta_x, std::vector<int> &delta_y)
    {
        next_orientation_vec = std::vector<int>{(current_orientation + 3) % 4, current_orientation, (current_orientation + 1) % 4, current_orientation};
        delta_x.resize(4);
        delta_y.resize(4);
        delta_x[3] = DELTA_XY[current_orientation].first;
        delta_y[3] = DELTA_XY[current_orientation].second;
        return true;
    }

    bool yawToOrientation(double yaw, int &orientation)
    {
        if (yaw < -M_PI || yaw > M_PI)
            return false;
        if (yaw < 0)
            yaw += 2 * M_PI;
        double orientation_index = yaw / (M_PI / 2);
        orientation = (int)round(orientation_index) % 4;
        return true;
    }

    bool orientationToYaw(int orientation, double &yaw)
    {
        yaw = orientation * (M_PI / 2);
        if (yaw > M_PI)
        {
            yaw -= 2 * M_PI;
        }
        return true;
    }
}