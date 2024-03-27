#include "pso.h"
namespace task_schedule 
{
    
    std::vector<Particle> initializeSwarm(size_t numParticles, std::vector<Task> tasks, std::vector<Robot> robots, mapf_environment::BaseEnvironment *env) 
    {
        std::vector<Particle> swarm;
        for (size_t i = 0; i < numParticles; ++i) 
        {
            Particle p;
            p.taskAssignment.resize(tasks.size());
            p.bestTaskAssignment.resize(tasks.size());
            p.velocity.resize(tasks.size());
            // 随机初始化任务分配和速度
            for (size_t j = 0; j < tasks.size(); ++j) 
            {
                //给粒子中的任务分配机器人
                p.taskAssignment[j] = rand() % robots.size();
                p.velocity[j] = (std::numeric_limits<double>::max)() - static_cast<double>(rand()) / RAND_MAX;
            }
            // 计算初始适应度
            p.fitness = calculateFitness(p.taskAssignment, tasks, robots, env);
            p.bestFitness = p.fitness;
            p.bestTaskAssignment = p.taskAssignment;
            swarm.push_back(p);
        }
        return swarm;
    }

    //计算分配方案的Fitness
    double calculateFitness(const std::vector<int>& taskAssignment, const std::vector<Task>& tasks, const std::vector<Robot>& robots, mapf_environment::BaseEnvironment *env)
    {
        double totalCompletionCost = 0.0;
        std::vector<std::vector<int>> robotsTaskAssignments(robots.size());
        for(size_t i = 0; i < taskAssignment.size(); ++i) 
        {
            int robot_id = taskAssignment[i];
            int task_id = i;
            robotsTaskAssignments[robot_id].push_back(task_id);
        }

        for(size_t i = 0; i < robotsTaskAssignments.size(); ++i) 
        {
            double taskCompletionCost = 0.0;
            int start_location = robots[i].location;
            int goal_location = tasks[robotsTaskAssignments[i][0]].start_location;
            taskCompletionCost += calculateTaskCompletionTime(start_location, goal_location, env) + tasks[robotsTaskAssignments[i][0]].cost;

            for(size_t j = 1; j < robotsTaskAssignments[i].size(); ++j) 
            {
                start_location = tasks[robotsTaskAssignments[i][j-1]].goal_location;
                goal_location = tasks[robotsTaskAssignments[i][j]].start_location;
                taskCompletionCost += calculateTaskCompletionTime(start_location, goal_location, env) + tasks[robotsTaskAssignments[i][j]].cost;
            }
            totalCompletionCost += taskCompletionCost;
        }
        
        return -totalCompletionCost;
    }


    double calculateTaskCompletionTime(int& start_location, int& goal_location, mapf_environment::BaseEnvironment *env)
    {
        double s_world_x, s_world_y;
        double g_world_x, g_world_y;
        env->idToWorld(start_location, s_world_x, s_world_y);
        env->idToWorld(goal_location, g_world_x, g_world_y);
        return abs(s_world_x - g_world_x) + abs(s_world_y - g_world_y);
    }


    void updateVelocityAndPosition(Particle& particle, const std::vector<Particle>& swarm, const std::vector<Task>& tasks, const std::vector<Robot>& robots, mapf_environment::BaseEnvironment *env) 
    {
        // 找到全局最优解
        std::vector<int> bestGlobalTaskAssignment = particle.bestTaskAssignment;
        double bestGlobalFitness = particle.bestFitness;
        for (const Particle& p : swarm) 
        {
            if (p.bestFitness > bestGlobalFitness) 
            {
                bestGlobalFitness = p.bestFitness;
                bestGlobalTaskAssignment = p.bestTaskAssignment;
            }
        }

        // 更新速度和位置
        for (size_t i = 0; i < particle.taskAssignment.size(); ++i) 
        {
            double cognitiveComponent = c1 * (particle.bestTaskAssignment[i] - particle.taskAssignment[i]);
            double socialComponent = c2 * (bestGlobalTaskAssignment[i] - particle.taskAssignment[i]);
            double newVelocity = w * particle.velocity[i] + cognitiveComponent + socialComponent;
            
            // 更新速度，同时限制速度的范围
            if (newVelocity < -10.0) {
                particle.velocity[i] = -10.0; // 如果速度小于-10，则设置为-10
            } else if (newVelocity > 10.0) {
                particle.velocity[i] = 10.0; // 如果速度大于10，则设置为10
            } else {
                particle.velocity[i] = newVelocity; // 否则，保持原始速度值
            }

            // 计算新的任务分配
            int newTaskAssignment = particle.taskAssignment[i] + static_cast<int>(newVelocity);
            // 检查并限制新任务分配的值
            if (newTaskAssignment < 0) {
                newTaskAssignment = 0; // 如果结果小于0，则限制为最小值
            } else if (newTaskAssignment >= robots.size()) {
                newTaskAssignment = robots.size() - 1; // 如果结果大于等于机器人的数量，则限制为最大值
            }
            // 更新粒子的任务分配
            particle.taskAssignment[i] = newTaskAssignment;

            if (newTaskAssignment != particle.taskAssignment[i]) {
                particle.taskAssignment[i] = newTaskAssignment;
                // 这里可能需要检查新分配是否有效，例如没有违反任务和机器人的约束
            }
        }

        // 重新计算适应度
        particle.fitness = calculateFitness(particle.taskAssignment, tasks, robots, env);

        // 更新个体最优
        if (particle.fitness > particle.bestFitness) 
        {
            particle.bestFitness = particle.fitness;
            particle.bestTaskAssignment = particle.taskAssignment;
        }
    }



    // 粒子群算法主循环
void particleSwarmOptimization(const std::vector<Task>& tasks, const std::vector<Robot>& robots, size_t numParticles, int maxIterations, mapf_environment::BaseEnvironment *env) {
    std::vector<Particle> swarm = initializeSwarm(numParticles, tasks, robots, env);
    for (int iteration = 0; iteration < maxIterations; ++iteration) 
    {
        for (Particle& particle : swarm) 
        {
            updateVelocityAndPosition(particle, swarm, tasks, robots, env);
        }
        // 检查停止条件，例如适应度不再显著改善
        // ...
    }
    // 输出最终的最优解
    double bestFitness = std::numeric_limits<double>::max();
    std::vector<int> bestTaskAssignment;
    for (const Particle& p : swarm) 
    {
        if (p.bestFitness < bestFitness) 
        {
            bestFitness = p.bestFitness;
            bestTaskAssignment = p.bestTaskAssignment;
        }
    }
    // bestTaskAssignment 包含了最优的任务分配方案
    // ...
}
}
