/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-02-04 17:02:07
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:47:37
 */

#pragma once
#ifndef MAPF_PLANNER_A_STAR_H
#define MAPF_PLANNER_A_STAR_H

#include <iostream>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>
#include <mapf_environment/base_environment.h>
#include <boost/smart_ptr/shared_ptr.hpp>

namespace std
{
    template <>
    struct hash<std::pair<State, State>>
    {
        size_t operator()(const std::pair<State, State> &sp) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, sp.first.location);
            boost::hash_combine(seed, sp.first.time);
            boost::hash_combine(seed, sp.second.location);
            boost::hash_combine(seed, sp.second.time);
            return seed;
        }
    };
}

namespace mapf_planner
{
    struct StateHasher_
    {
        std::size_t operator()(const State &state) const
        {
            size_t seed = 0;
            boost::hash_combine(seed, state.location);
            boost::hash_combine(seed, state.time);
            return seed;
        }
    };
    struct Solution_t
    {
        std::vector<State> states;
        double cost;
    };

    struct Constraint
    {
        Constraint() {}

        std::unordered_set<State, StateHasher_> vertex_constraint;
        std::unordered_set<std::pair<State, State>> edge_constraint;

        bool stateIsValid(const State &state)
        {
            return vertex_constraint.find(state) == vertex_constraint.end();
        }

        bool transitionIsValid(const State &state1, const State &state2)
        {
            return edge_constraint.find(std::make_pair(state1, state2)) == edge_constraint.end();
        }
    };

    class AStar
    {
    public:
        AStar(Constraint &constraint, mapf_environment::BaseEnvironment *env);
        bool search(const Pose &start_pose, const Pose &goal_pose, Solution_t &solution);

    private:
        void reconstruct_path(std::unordered_map<State, State> &came_from, State current, std::vector<State> &solution_states);

        double admissible_heuristic(const int &current_location, const int &goal_location);

        bool getNeighborStates(const State &state, std::vector<State> &neighbor_states, std::vector<double> &costs);

        Constraint constraint_;

        mapf_environment::BaseEnvironment *env_;
    };
}

#endif