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
#include "pso.h"
//任务的起始点和目标点
struct Task
{
    int task_id;
    int start_location;
    int goal_location;
    int priority;
    double cost;
    //任务类型（搬运货物、充电、抬运货物等等）
    int type;
    Task(): task_id(0), start_location(0), goal_location(0), priority(0), type(0){}
    //cost暂时不初始化，根据具体地图来使用astar给出简单评估
    Task(const int &id, const int &s,const int &g, const int &p, const int &t = 0): task_id(id),start_location(s), goal_location(g), priority(p), type(t){}
};
bool compareTasks(const Task& a, const Task& b) {
    // 优先级相同的情况下，根据耗时评估来排序，耗时少的任务优先级更高
    if (a.priority == b.priority) {
        return a.cost > b.cost;
    }
    return a.priority < b.priority; // 优先级高的任务优先级更高
}


//robot的位置、方向、以及种类（ctu/agv）
struct Robot
{
    int robot_id;
    int location;
    int orientation;
    //机器人类型
    int type;
    //机器人任务列表
    std::vector<int> taskAssignment;
    Robot(): robot_id(0),location(0), orientation(0), type(0){}
    Robot(const int &id, const int &location, const int &orientation, const int &type = 0):robot_id(id), location(location), orientation(orientation), type(type){}
    Robot(const int &id, const Pose& pose,const int &type = 0) :robot_id(id), location(pose.location), orientation(pose.orientation), type(type){}
};

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
    void setTaskCost(std::vector<Task>& tasks);
        //使用简单版 或者 Astar预估
    
    //分配任务给智能体
    void assignTasks(std::vector<Task> &tasks, std::vector<Robot> &robots);
        //使用pso或者其他
    

    ~taskSchedule(){}

    };
}


#endif