/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-02 14:52:27
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 20:41:37
 */

#pragma once
#ifndef MAPF_ENVIRONMENT_GRID_ENVIRONMENT_H
#define MAPF_ENVIRONMENT_GRID_ENVIRONMENT_H
#include <unordered_set>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include "base_environment.h"

namespace mapf_environment
{

    
    class GridEnvironment : public BaseEnvironment
    {
        struct Grid
        {
            Grid() : x(0.0), y(0.0), free_space(false) {}
            Grid(double x, double y, bool free_space) : x(x), y(y), free_space(free_space) {}

            double x;
            double y;
            bool free_space;
        };

        typedef std::vector<Grid> GridMap;

    public:
        GridEnvironment(std::string name);

        void initialize(std::string name) override;

        bool worldToId(const double &world_x, const double &world_y, int &id) override;

        bool idToWorld(const int &id, double &world_x, double &world_y) override;

        bool getNeighbors(const Pose &current, std::vector<Pose> &neighbors, std::vector<double> &costs) override;

        std::string getGlobalFrameID() override;

        void setObstacles(const std::unordered_set<int> &obstacle_set) override;

        ~GridEnvironment();

    private:
        bool idIsValid(const int &id);

        bool coordIsValid(const int &map_x, const int &map_y);

        void publishGridMarker();

        ros::Publisher grid_marker_pub_;

        std::vector<double> x_coords_, y_coords_;
        int map_width_, map_height_;

        GridMap grid_map_;

        std::string global_frame_;

        std::unordered_set<int> obstacle_set_;
    };

}

#endif