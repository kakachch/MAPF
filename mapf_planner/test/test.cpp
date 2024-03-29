/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-12 09:29:57
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:48:39
 */
#include <iostream>
#include <mapf_environment/test_environment.h>
#include <task_schedule/task_schedule.h>
#include <task_schedule/taskAndRobot.h>
#include <mapf_planner/cbs.h>
#include <chrono>
#include "test_environment.cpp"
#include "cbs.cpp"
#include "a_star.cpp"
#include "orientation.cpp"
#include "task_schedule.cpp"
#include "pso.cpp"


int main()
{
    // mapf_environment::BaseEnvironment *env = new mapf_environment::TestEnvironment();
    // std::vector<Pose> start_poses{{1,1},{19,1},{26,1},{45,1},{17,1},{79,1},{35,1},{116,1},{112,1},{194,1},{171,1},{5,1},{88,1},{144,1}};
    // std::vector<Pose> goal_poses{{19,1}, {53,1},{66,1},{11,1},{3,1},{85,1},{33,1},{37,1},{147,1},{152,1},{175,1},{122,1},{116,1},{199,1}};
    // // std::vector<int> start_poses{1, 19,26,45,17,79,35,116,112,194,171,5,88,144};
    // // std::vector<int> goal_poses{19, 53,66,11,3,85,33,37,147,152,175,122,116,199};
    // std::vector<std::vector<State>> solution;
    // auto start = std::chrono::high_resolution_clock::now();
    // mapf_planner::CBS(env).search(start_poses, goal_poses, solution);
    // auto end = std::chrono::high_resolution_clock::now();

    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    // std::cout << "Search method took " << duration << " milliseconds." << std::endl;

    // for (size_t agent_id = 0; agent_id < start_poses.size(); ++agent_id)
    // {
    //     std::cout << "agent id: " << agent_id << std::endl;
    //     for (size_t time_step = 0; time_step < solution[agent_id].size(); ++time_step)
    //     {
    //         std::cout << "time step:" << time_step << " position:" << solution[agent_id][time_step].location << " orientation:" << solution[agent_id][time_step].orientation << std::endl;
    //     }
    // }

    std::vector<task_schedule::Task> tasks = {{0,1,19,1},{1,19,53,1},{2,45,11,1},{3,35,33,1},{4,112,147,1}};
    std::vector<task_schedule::Robot> robots = {{0,2,1},{1,4,1},{2,7,1}};
    mapf_environment::BaseEnvironment *env = new mapf_environment::TestEnvironment();
    task_schedule::taskSchedule TS(tasks, robots, env);
    TS.setTaskCost();
    TS.assignTasks();
    TS.showRobotsAssignments();
    std::cout << "end";
    return 0;
}