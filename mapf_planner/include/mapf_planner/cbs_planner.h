/*
 * @brief        : 
 * @Author       : Quyicheng
 * @Date         : 2024-02-01 21:41:47
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:53:17
 */

#pragma once

#ifndef MAPF_PLANNER_CBS_PLANNER_H
#define MAPF_PLANNER_CBS_PLANNER_H
#include "base_mapf_planner.h"

namespace mapf_planner
{
    class CBSPlanner : public mapf_planner::BaseMAPFPlanner
    {
    public:
        CBSPlanner(){}

        bool search(const std::vector<Pose> &start_loc, const std::vector<Pose> &goal_loc,
                    std::vector<std::vector<State>> &solution) override;
        ~CBSPlanner(){}
    };

}
#endif