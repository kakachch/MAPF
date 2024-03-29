/**
 * @file task_schedule.cpp
 * @author Peng XianKang (718257480@qq.com)
 * @brief 
 * @date 2024-03-21
 */

#include "task_schedule/task_schedule.h"

namespace task_schedule 
{
    void taskSchedule::setTaskCost() 
    {
        //简单使用曼哈顿距离作为耗费时间
        double s_world_x, s_world_y;
        double g_world_x, g_world_y;
        for(auto &task : tasks) 
        {
            env_->idToWorld(task.start_location, s_world_x, s_world_y);
            env_->idToWorld(task.goal_location, g_world_x, g_world_y);
            task.cost = abs(s_world_x - g_world_x) + abs(s_world_y - g_world_y);
        }
        //调用A*作简单的预估
        // mapf_planner::Constraint constraint;
        // mapf_planner::AStar astar(constraint, env_);
        // mapf_planner::Solution_t solution;
        // for(auto &task : tasks) 
        // {
        //     solution.states.clear();
        //     astar.search(Pose(task.start_location,1), Pose(task.goal_location,1), solution);
        //     task.cost = solution.cost;
        // }
        
    }

    //任务调度：输入一组任务和机器人;分配任务给智能体
    void taskSchedule::assignTasks() 
    {   
        //使用PSO算法来进行任务分配
        std::vector<std::vector<int>> robotsTaskAssignments(robots.size());
        robotsTaskAssignments = particleSwarmOptimization(tasks, robots, 20, 500, env_);
        // for(auto &robot : robots) 
        // {
        //     robot.taskAssignment = robotsTaskAssignments[robot.robot_id];
        // }
        for(size_t i = 0; i < robots.size(); ++i) 
        {
            robots[i].taskAssignment = robotsTaskAssignments[i];
        }
    }

    void taskSchedule::showRobotsAssignments() 
    {
        for(auto robot : robots) 
        {
            std::cout <<"robotid"<<robot.robot_id << ":";
            for(size_t i = 0; i < robot.taskAssignment.size(); ++i) 
            {
                std::cout  <<" "<< robot.taskAssignment[i] << " ";
            }
            std::cout << std::endl;
        }
    }
}
