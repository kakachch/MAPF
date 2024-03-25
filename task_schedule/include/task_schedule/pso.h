#include <task_schedule/task_schedule.h>

namespace task_schedule 
{
    struct Particle {
        std::vector<int> taskAssignment; // 任务分配方案
        std::vector<int> bestTaskAssignment; // 最好的任务分配方案
        double bestFitness; // 最好的适应度值
        double fitness; // 当前适应度值
        std::vector<double> velocity; // 速度
        double w; // 惯性权重
        double c1; // 个体学习因子
        double c2; // 社会学习因子
    };
    // 初始化粒子群
    std::vector<Particle> initializeSwarm(size_t numParticles, std::vector<Task> tasks) 
    {
        std::vector<Particle> swarm;
        for (size_t i = 0; i < numParticles; ++i) 
        {
            Particle p;
            p.taskAssignment.resize(tasks.size());
            p.bestTaskAssignment.resize(tasks.size());
            p.velocity.resize(robots.size());
            p.w = 0.7; // 惯性权重
            p.c1 = 1.4; // 个体学习因子
            p.c2 = 1.4; // 社会学习因子
            // 随机初始化任务分配和速度
            for (size_t j = 0; j < tasks.size(); ++j) 
            {
                p.taskAssignment[j] = rand() % robots.size();
                p.velocity[j] = (std::numeric_limits<double>::max)() - static_cast<double>(rand()) / RAND_MAX;
            }
            // 计算初始适应度
            p.fitness = calculateFitness(p.taskAssignment, tasks, robots);
            p.bestFitness = p.fitness;
            p.bestTaskAssignment = p.taskAssignment;
            swarm.push_back(p);
        }
        return swarm;
    }
}