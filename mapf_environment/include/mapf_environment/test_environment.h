/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-14 09:40:49
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 20:41:28
 */

#pragma once
#ifndef MAPF_ENVIRONMENT_TEST_ENVIRONMENT
#define MAPF_ENVIRONMENT_TEST_ENVIRONMENT
#include "base_environment.h"

namespace mapf_environment
{
    class TestEnvironment : public BaseEnvironment
    {
    public:
        TestEnvironment();

        void initialize(std::string name) override;

        bool worldToId(const double &world_x, const double &world_y, int &id) override;

        bool idToWorld(const int &id, double &world_x, double &world_y) override;

        bool getNeighbors(const Pose &current, std::vector<Pose> &neighbors, std::vector<double> &costs) override;

        std::string getGlobalFrameID() override;

        void setObstacles(const std::unordered_set<int> &obstacle_set) override;

        ~TestEnvironment();

        int map_height_, map_width_;
        std::unordered_set<int> obstacle_set_;
        std::vector<bool> free_map_;
    };
}

#endif