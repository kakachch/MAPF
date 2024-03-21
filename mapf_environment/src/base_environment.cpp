/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-10 11:29:31
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:37:43
 */
#include <mapf_environment/base_environment.h>
#include <cassert>
#include <cmath>

namespace mapf_environment
{
    double BaseEnvironment::getEuclideanDistance(const int &id_1, const int &id_2)
    {
        double world_x_1, world_y_1, world_x_2, world_y_2;
        assert(idToWorld(id_1, world_x_1, world_y_1) && idToWorld(id_2, world_x_2, world_y_2));

        return sqrt(hypot(world_x_1 - world_x_2, world_y_1 - world_y_2));
    }
    double BaseEnvironment::getManhattanDistance(const int &id_1, const int &id_2)
    {
        double world_x_1, world_y_1, world_x_2, world_y_2;
        assert(idToWorld(id_1, world_x_1, world_y_1) && idToWorld(id_2, world_x_2, world_y_2));

        return fabs(world_x_1 - world_x_2) + fabs(world_y_1 - world_y_2);
    }
}
