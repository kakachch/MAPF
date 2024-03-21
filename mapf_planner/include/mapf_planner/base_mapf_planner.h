/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-04 10:14:21
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:55:40
 */
#pragma once

#ifndef MAPF_PLANNER_BASE_MAPF_PLANNER_H
#define MAPF_PLANNER_BASE_MAPF_PLANNER_H
#include <ros/ros.h>
#include <mapf_environment/base_environment.h>
#include <nav_msgs/Path.h>

namespace mapf_planner
{
    class BaseMAPFPlanner
    {
    public:
        void initialize(std::string name, mapf_environment::BaseEnvironment *env);

        virtual bool makePlan(const std::vector<Pose> &start_loc, const std::vector<Pose> &goal_loc, std::vector<std::vector<Pose>> &plan);

        virtual ~BaseMAPFPlanner(){}

    protected:

        virtual bool search(const std::vector<Pose> &start_loc, const std::vector<Pose> &goal_pose,
                            std::vector<std::vector<State>> &solution) = 0;

        mapf_environment::BaseEnvironment *env_;
        std::string global_frame_;
    };
}

#endif