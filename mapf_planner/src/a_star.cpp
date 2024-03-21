/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-02-18 15:21:43
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:49:46
 */
#include <mapf_planner/a_star.h>

namespace mapf_planner
{
    AStar::AStar(Constraint &constraint, mapf_environment::BaseEnvironment *env) : constraint_(constraint), env_(env) {}

    bool AStar::search(const Pose &start_pose, const Pose &goal_pose, Solution_t &solution)
    {
        State start_state(0, start_pose);
        std::unordered_set<State> closed_set;
        std::unordered_set<State> open_set = {start_state};

        std::unordered_map<State, State> came_from;

        std::unordered_map<State, float> g_score;
        g_score[start_state] = 0;

        std::unordered_map<State, float> f_score;
        f_score[start_state] = admissible_heuristic(start_pose.location, goal_pose.location);

        while (!open_set.empty())
        {
            State current = *std::min_element(open_set.begin(), open_set.end(),
                                              [&](const State &a, const State &b)
                                              { return f_score[a] < f_score[b]; });

            if (current.location == goal_pose.location)
            {
                reconstruct_path(came_from, current, solution.states);
                solution.cost = f_score[current];
                return true;
            }

            open_set.erase(current);
            closed_set.insert(current);

            std::vector<State> neighbor_states;
            std::vector<double> cost_list;
            getNeighborStates(current, neighbor_states, cost_list);

            for (size_t i = 0; i < neighbor_states.size(); ++i)
            {
                State neighbor_state = neighbor_states[i];
                if (!constraint_.stateIsValid(neighbor_state))
                {
                    continue;
                }

                if (closed_set.find(neighbor_state) != closed_set.end())
                {
                    continue;
                }

                float tentative_g_score = g_score[current] + cost_list[i];

                if (!open_set.count(neighbor_state))
                {
                    open_set.insert(neighbor_state);
                }
                else if (tentative_g_score >= g_score[neighbor_state])
                {
                    continue;
                }

                came_from[neighbor_state] = current;
                g_score[neighbor_state] = tentative_g_score;
                f_score[neighbor_state] = g_score[neighbor_state] + admissible_heuristic(neighbor_state.location, goal_pose.location);
            }
        }
        return false;
    }
    void AStar::reconstruct_path(std::unordered_map<State, State> &came_from, State current, std::vector<State> &solution_states)
    {
        solution_states.clear();
        solution_states.emplace_back(current);
        while (came_from.find(current) != came_from.end())
        {
            current = came_from[current];
            solution_states.emplace_back(current);
        }
        std::reverse(solution_states.begin(), solution_states.end());
    }

    double AStar::admissible_heuristic(const int &current_location, const int &goal_location)
    {
        double cur_world_x, cur_world_y, goal_world_x, goal_world_y;
        assert(env_->idToWorld(current_location, cur_world_x, cur_world_y) && env_->idToWorld(goal_location, goal_world_x, goal_world_y));
        return fabs(cur_world_x - goal_world_x) + fabs(cur_world_y - goal_world_y);
    }

    bool AStar::getNeighborStates(const State &state, std::vector<State> &neighbor_states, std::vector<double> &costs)
    {
        std::vector<Pose> neighbor_list;
        std::vector<double> neighbor_costs;
        env_->getNeighbors(state.getPose(), neighbor_list, neighbor_costs);
        for (size_t i = 0; i < neighbor_list.size(); i++)
        {
            State neighbor_state(state.time + 1, neighbor_list[i]);
            if (constraint_.stateIsValid(neighbor_state) && constraint_.transitionIsValid(state, neighbor_state))
            {
                neighbor_states.emplace_back(neighbor_state);
                costs.emplace_back(neighbor_costs[i]);
            }
        }
        return neighbor_states.size();
    }
}