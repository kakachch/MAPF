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
//任务的起始点和目标点
struct task
{
    int start_location;
    int goal_location;
    task(): start_location(0),goal_location(0){}
    task(const int &start_location,const int &goal_location): start_location(start_location), goal_location(goal_location){}
    
};
//robot的位置、方向、以及种类（ctu/agv）
struct robot
{
    int location;
    int orientation;
    int type;
    robot(): location(0), orientation(0), type(){}
    robot(const int &location, const int &orientation, const int &type): location(location), orientation(orientation), type(type){}
    robot(const Pose &pose,const int &type) : location(pose.location), orientation(pose.orientation), type(type){}
};

#endif