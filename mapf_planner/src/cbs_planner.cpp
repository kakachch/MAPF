/*
 * @brief        : 
 * @Author       : Quyicheng
 * @Date         : 2024-02-27 11:47:55
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:53:00
 */

#include <pluginlib/class_list_macros.h>
#include "mapf_planner/cbs_planner.h"
#include "mapf_planner/cbs.h"

PLUGINLIB_EXPORT_CLASS(mapf_planner::CBSPlanner, mapf_planner::BaseMAPFPlanner)

namespace mapf_planner
{
    bool CBSPlanner::search(const std::vector<Pose> &start_loc, const std::vector<Pose> &goal_loc,
                    std::vector<std::vector<State>> &solution)
    {
        return CBS(env_).search(start_loc, goal_loc, solution);
    }

}