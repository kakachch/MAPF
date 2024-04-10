/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-12 09:29:57
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:48:39
 */
#include <iostream>
#include <chrono>
#include <random>
#include <ctime>
#include <mapf_environment/test_environment.h>
#include <task_schedule/task_schedule.h>
#include <task_schedule/taskAndRobot.h>
#include <mapf_planner/cbs.h>

#include "test_environment.cpp"
#include "cbs.cpp"
#include "a_star.cpp"
#include "orientation.cpp"
#include "task_schedule.cpp"
#include "pso.cpp"

void randomTest(std::vector<Pose>& start_poses, std::vector<Pose>& goal_poses, std::vector<std::vector<State>>& solution, int num, mapf_environment::BaseEnvironment *env);

int main()
{
    // std::vector<Pose> start_poses{{1,1},{19,1},{26,1},{45,1},{17,1},{79,1},{35,1},{116,1},{112,1},{194,1},{171,1},{5,1},{88,1},{144,1}};
    // std::vector<Pose> goal_poses{{19,1}, {53,1},{66,1},{11,1},{3,1},{85,1},{33,1},{37,1},{147,1},{152,1},{175,1},{122,1},{116,1},{199,1}};
    // // std::vector<int> start_poses{1, 19,26,45,17,79,35,116,112,194,171,5,88,144};
    // // std::vector<int> goal_poses{19, 53,66,11,3,85,33,37,147,152,175,122,116,199};
    // std::vector<std::vector<State>> solution;
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
    auto start = std::chrono::high_resolution_clock::now();
    TS.setTaskCost();
    TS.assignTasks();
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "task schedule took " << duration << " milliseconds." << std::endl;
    TS.showRobotsAssignments();
    std::cout << "task schedule end" << std::endl;

    //CBS
    std::vector<Pose> start_poses, goal_poses;
    std::vector<std::vector<State>> solution;
    //start_poses is robots' location
    //goal_poses is the first robots'taskassignment[0]
    for(const task_schedule::Robot& robot : TS.robots)
    {
        start_poses.push_back({robot.location, robot.orientation});
        if(robot.taskAssignment.size() != 0)
            goal_poses.push_back({TS.tasks[robot.taskAssignment[0]].start_location, robot.orientation});
        else 
            goal_poses.push_back({robot.location, robot.orientation});
    }
    //find robots max length of taskassignment
    int maxLengthTaskAssiment = 0;
    for(const task_schedule::Robot& robot : TS.robots) 
    {
        if(maxLengthTaskAssiment < robot.taskAssignment.size())
            maxLengthTaskAssiment = robot.taskAssignment.size();
    }
    std::cout << "CBS" << std::endl; 
    //对每个机器人进行CBS规划
    for(size_t i = 0; i < maxLengthTaskAssiment; ++i) 
    {
        auto start = std::chrono::high_resolution_clock::now();

        mapf_planner::CBS(env).search(start_poses, goal_poses, solution);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
        std::cout << "path finding took " << duration << " milliseconds." << std::endl;
        
        start_poses.clear();
        goal_poses.clear();
        solution.clear();
        for(const task_schedule::Robot& robot : TS.robots)
        {
            //start_Poses and goal_Poses
            if(i >= robot.taskAssignment.size())
            {
                start_poses.push_back({TS.tasks[robot.taskAssignment.back()].goal_location, robot.orientation});
                goal_poses.push_back({TS.tasks[robot.taskAssignment.back()].goal_location, robot.orientation});
            }
            else 
            {
                start_poses.push_back({TS.tasks[robot.taskAssignment[i]].start_location, robot.orientation});
                goal_poses.push_back({TS.tasks[robot.taskAssignment[i]].goal_location, robot.orientation});
            }
        }
    }
    // for(int num = 6; num <= 40; num+=2)
    //     randomTest(start_poses, goal_poses, solution, num, env);
    randomTest(start_poses, goal_poses, solution, 40, env);
    std::cout << "end" << std::endl;
    
    return 0;
}

void randomTest(std::vector<Pose>& start_poses, std::vector<Pose>& goal_poses, std::vector<std::vector<State>>& solution, int num, mapf_environment::BaseEnvironment *env) 
{
    start_poses.clear();
    goal_poses.clear();
    solution.clear();
    
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> dis(1, 700);
    // std::uniform_int_distribution<> dis_st(1, 100);
    // std::uniform_int_distribution<> dis_gl(201, 300);

    for(int i = 0; i < num; ++i)
    {
        int st = dis(gen);
        int gl = dis(gen);
        std::cout << "st:" << st << " gl:" << gl << std::endl;
        start_poses.push_back({st,1});
        goal_poses.push_back({gl,1});
    }

    auto start = std::chrono::high_resolution_clock::now();
        
        mapf_planner::CBS(env).search(start_poses, goal_poses, solution);

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "robots:" << num <<  " path finding took " << duration << " milliseconds." << std::endl;
    
}