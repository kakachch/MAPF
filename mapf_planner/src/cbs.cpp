/*
 * @Description:
 * @Version: V1.0.0
 * @Author: Quyicheng
 * @Date: 2024-02-02 18:48:20
 * @LastEditors: Quyicheng
 * @LastEditTime: 2024-02-06 22:38:05
 */

#include <mapf_planner/cbs.h>

namespace mapf_planner
{

    bool CBS::search(const std::vector<Pose> &start_poses, const std::vector<Pose> &goal_poses,
                     std::vector<std::vector<State>> &solution)
    {
        size_t agent_num = start_poses.size();
        CBSNode start_node;
        start_node.solution.resize(agent_num);
        start_node.constraints.resize(agent_num);
        start_node.cost = 0;
        {
            Constraint constraint;
            AStar a_star(constraint, env_);

            for (size_t agent_id = 0; agent_id < agent_num; ++agent_id)
            {
                bool success = a_star.search(start_poses[agent_id], goal_poses[agent_id], start_node.solution[agent_id]);
                if (!success)
                {
                    return false;
                }
                start_node.cost += start_node.solution[agent_id].cost;
            }
        }
        typename boost::heap::d_ary_heap<CBSNode, boost::heap::arity<2>,
                                         boost::heap::mutable_<true>>
            open;

        auto handle = open.push(start_node);
        (*handle).handle = handle;

        solution.clear();
        while (!open.empty())
        {
            CBSNode current_node = open.top();
            open.pop();
            std::unordered_map<size_t, Constraint> new_constraints;

            if (!getConstraints(current_node.solution, new_constraints))
            {
                solution.resize(agent_num);
                for (size_t agent_id = 0; agent_id < agent_num; ++agent_id)
                {
                    solution[agent_id] = current_node.solution[agent_id].states;
                }
                return true;
            }

            auto insert = [](Constraint &target_constraint, const Constraint &new_constraint)
            {
                for (const auto &vertex_constraint : new_constraint.vertex_constraint)
                {
                    target_constraint.vertex_constraint.emplace(vertex_constraint);
                }
                for (const auto &edge_constraint : new_constraint.edge_constraint)
                {
                    target_constraint.edge_constraint.emplace(edge_constraint);
                }
            };

            for (const auto &it : new_constraints)
            {
                size_t agent_id = it.first;
                Constraint new_constraint = it.second;
                CBSNode expanded_node = current_node;
                insert(expanded_node.constraints[agent_id], new_constraint);

                Constraint constraint = expanded_node.constraints[agent_id];

                expanded_node.cost -= expanded_node.solution[agent_id].cost;
                bool success = AStar(constraint, env_).search(start_poses[agent_id], goal_poses[agent_id], expanded_node.solution[agent_id]);

                if (success)
                {
                    expanded_node.cost += expanded_node.solution[agent_id].cost;
                    auto handle = open.push(expanded_node);
                    (*handle).handle = handle;
                }
            }
        }
        return false;
    }

    bool CBS::getConstraints(const std::vector<Solution_t> &solution, std::unordered_map<size_t, Constraint> &constraints)
    {
        if (solution.size() < 2)
        {
            return false;
        }
        int max_t = 0;
        for (const auto &sol : solution)
        {
            max_t = std::max<int>(max_t, sol.states.size() - 1);
        }
        auto get_state = [&](size_t agent_id, int time)
        {
            return (size_t)time < solution[agent_id].states.size() ? solution[agent_id].states[time] : solution[agent_id].states.back();
        };

        for (int t = 0; t <= max_t; ++t)
        {
            // check drive-drive vertex collisions
            for (size_t i = 0; i < solution.size(); ++i)
            {
                State cur_state1 = get_state(i, t), target_state1 = get_state(i, t + 1);
                for (size_t j = i + 1; j < solution.size(); ++j)
                {
                    State cur_state2 = get_state(j, t), target_state2 = get_state(j, t + 1);
                    if (cur_state1.location == cur_state2.location)
                    {
                        constraints[i].vertex_constraint.emplace(cur_state1);
                        constraints[j].vertex_constraint.emplace(cur_state2);
                        return true;
                    }

                    if (cur_state1.location == target_state2.location && target_state1.location == cur_state2.location)
                    {
                        constraints[i].edge_constraint.emplace(cur_state1, target_state1);
                        constraints[j].edge_constraint.emplace(cur_state2, target_state2);
                        return true;
                    }
                }
            }
        }
        return false;
    }

}