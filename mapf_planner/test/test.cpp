/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-12 09:29:57
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:48:39
 */
#include <iostream>
#include <mapf_environment/test_environment.h>
#include <mapf_planner/cbs.h>
#include <chrono>
#include "test_environment.cpp"
#include "cbs.cpp"
#include "a_star.cpp"

int main()
{
    mapf_environment::BaseEnvironment *env = new mapf_environment::TestEnvironment();
    std::vector<int> start_locations{1, 19,26,45,17,79,35,116,112,194,171,5,88,144};
    std::vector<int> goal_locations{19, 53,66,11,3,85,33,37,147,152,175,122,116,199};
    std::vector<std::vector<State>> solution;
    auto start = std::chrono::high_resolution_clock::now();
    mapf_planner::CBS(env).search(start_locations, goal_locations, solution);
    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "Search method took " << duration << " milliseconds." << std::endl;

    for (size_t agent_id = 0; agent_id < start_locations.size(); ++agent_id)
    {
        std::cout << "agent id: " << agent_id << std::endl;
        for (size_t time_step = 0; time_step < solution[agent_id].size(); ++time_step)
        {
            std::cout << "time step:" << time_step << " position:" << solution[agent_id][time_step].loc_id << std::endl;
        }
    }
    return 0;
}