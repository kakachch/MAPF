/**
 * @file pso.h
 * @author Peng XianKang (718257480@qq.com)
 * @brief 
 * @date 2024-03-28
 */
#pragma once
#ifndef PSO_H
#define PSO_H
#include "task_schedule/taskAndRobot.h"
#include <vector>
#include <iostream>
#include <algorithm>
#include <random>
#include <limits>

namespace task_schedule 
{
    double w = 0.7, c1 = 1.4, c2 = 1.4;
    struct Particle 
    {
        std::vector<int> taskAssignment; // 任务分配方案
        std::vector<int> bestTaskAssignment; // 最好的任务分配方案
        double bestFitness; // 最好的适应度值
        double fitness; // 当前适应度值
        std::vector<double> velocity; // 速度
        // double w; // 惯性权重
        // double c1; // 个体学习因子
        // double c2; // 社会学习因子
    };
    // 初始化粒子群 
    std::vector<Particle> initializeSwarm(size_t numParticles, 
                                            const std::vector<Task> &tasks,const std::vector<Robot> &robots, 
                                            mapf_environment::BaseEnvironment *env);
    //计算适应度函数
    double calculateFitness(const std::vector<int>& taskAssignment, 
                            const std::vector<Task>& tasks, 
                            const std::vector<Robot>& robots,
                            mapf_environment::BaseEnvironment *env);

    double calculateTaskCompletionTime(int& start_location, 
                                        int& goal_location, 
                                        mapf_environment::BaseEnvironment *env);
    //更新速度和位置
    double updateVelocityAndPosition(Particle& particle, const std::vector<Particle>& swarm, 
                                    const std::vector<Task>& tasks, const std::vector<Robot>& robots, 
                                    mapf_environment::BaseEnvironment *env); 

    std::vector<std::vector<int>> particleSwarmOptimization(const std::vector<Task>& tasks, const std::vector<Robot>& robots, size_t numParticles, int maxIterations, mapf_environment::BaseEnvironment *env);
            

}
#endif