/**
 * @file task_schedule.h
 * @author Peng XianKang (718257480@qq.com)
 * @brief 
 * @date 2024-03-21
 */
#pragma once
#ifndef TASK_ASSIGNMENT_H
#define TASK_ASSIGNMENT_H
#include <mapf_environment/base_environment.h>
#include <mapf_planner/a_star.h>
#include "task_schedule/pso.h"
// #include "taskAndRobot.h"
namespace task_schedule 
{
    class taskSchedule
    {
    private:
    std::vector<Task> tasks;
    std::vector<Robot> robots;
    mapf_environment::BaseEnvironment *env_;

    public:
    taskSchedule(const std::vector<Task>& t, const std::vector<Robot>& r, mapf_environment::BaseEnvironment *env) : tasks(t), robots(r), env_(env) {}
    
    //给任务预估cost
    void setTaskCost();
        //使用简单版 或者 Astar预估
    
    //分配任务给智能体
    void assignTasks();
        //使用pso或者其他
    
    void showRobotsAssignments();
    ~taskSchedule(){}

    };
}


#endif