/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-02-02 11:09:44
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 22:54:18
 */

#include <mapf_planner/base_mapf_planner.h>
#include <algorithm>

namespace mapf_planner
{
    void BaseMAPFPlanner::initialize(std::string name, mapf_environment::BaseEnvironment *env)
    {
        ros::NodeHandle private_nh("~/" + name);
        env_ = env;
        global_frame_ = env_->getGlobalFrameID();
    }

    bool BaseMAPFPlanner::makePlan(const std::vector<Pose> &start_pose, const std::vector<Pose> &goal_pose, std::vector<std::vector<Pose>> &plan)
    {

        if (start_pose.empty() || goal_pose.empty())
        {
            ROS_ERROR("Start and goal vectors are empty!");
            return false;
        }
        if (start_pose.size() != goal_pose.size())
        {
            ROS_ERROR("Start and goal vectors are not the same length!");
            return false;
        }
        size_t agent_num = start_pose.size();

        std::vector<std::vector<State>> solution;

        bool success = search(start_pose, goal_pose, solution);

        if (!success)
        {
            ROS_INFO("BaseMAPFPlanner::makeplan,failed to make plan");
        }
        else
        {
            plan.resize(agent_num);
            for (size_t agent_id = 0; agent_id < agent_num; ++agent_id)
            {
                std::vector<State> states = solution[agent_id];
                std::vector<Pose> single_plan(states.size());
                for (size_t i = 0; i < states.size(); ++i)
                {
                    single_plan[i] = states[i].getPose();
                }
                plan[agent_id] = single_plan;
            }
        }
        return success;
    }
}