/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-02-04 17:02:07
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:52:05
 */

#pragma once

#ifndef MAPF_PLANNER_CBS_H
#define MAPF_PLANNER_CBS_H

#include <vector>
#include <boost/heap/d_ary_heap.hpp>

#include "a_star.h"

namespace mapf_planner
{
    struct CBSNode
    {
        std::vector<Solution_t> solution;
        std::vector<Constraint> constraints;
        double cost;
        boost::heap::d_ary_heap<CBSNode, boost::heap::arity<2>,
                                boost::heap::mutable_<true>>::handle_type handle;
        bool operator<(const CBSNode &other) const
        {
            return cost > other.cost;
        }
    };
    class CBS
    {
    public:
        CBS(mapf_environment::BaseEnvironment *env) : env_(env) {}

        /**
         * @brief Main search function for the CBS algorithm. Finds paths for multiple agents avoiding conflicts.
         * @param start_locations Starting points for each agent
         * @param goal_locations Goal points for each agent
         * @param solution A vector to store the solution paths for each agent
         * @return Returns true if a conflict-free path for all agents is found, false if no such path exists or an error occurs
         */
        bool search(const std::vector<Pose> &start_poses, const std::vector<Pose> &goal_poses,
                    std::vector<std::vector<State>> &solution);

    private:

        bool getConstraints(const std::vector<Solution_t> &solution, std::unordered_map<size_t, Constraint>& new_constraints);

        mapf_environment::BaseEnvironment *env_;
    };

}

#endif