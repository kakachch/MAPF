/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-02-19 22:15:25
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-17 19:23:36
 */
#pragma once

#ifndef MAPF_PLANNER_SIMPLE_LOCAL_PLANNER_H
#define MAPF_PLANNER_SIMPLE_LOCAL_PLANNER_H
#include <nav_core/base_local_planner.h>
#include <mapf_msgs/ActionFeedback.h>
#include <nav_msgs/Path.h>
#include <tf2/utils.h>
#include <base_local_planner/odometry_helper_ros.h>
#include <base_local_planner/local_planner_util.h>
#include <base_local_planner/goal_functions.h>
#include <mutex>

namespace mapf_planner
{

    class SimpleLocalPlanner : public nav_core::BaseLocalPlanner
    {
    public:
        SimpleLocalPlanner() : initialized_(false){};
        ~SimpleLocalPlanner(){};

        void initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros) override;

        bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan) override;

        bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel) override;

        bool isGoalReached() override;

        bool getFeedback(mapf_msgs::ActionFeedback &feedback);

        bool getDistanceAndThelta(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &source_pose, double &distance, double &thelta);

    private:
        tf2_ros::Buffer *tf_;
        ros::Publisher plan_pub_;

        costmap_2d::Costmap2DROS *costmap_ros_;

        base_local_planner::OdometryHelperRos odom_helper_;

        std::string global_frame_, robot_base_frame_;

        bool initialized_, holonomic_robot_, feedback_modified_;

        double yaw_goal_tolerance_, xy_goal_tolerance_, acc_lim_x_, acc_lim_theta_, max_vel_x_, min_vel_x_, max_vel_theta_;
        int smoothing_factor_;

        std::vector<geometry_msgs::PoseStamped> current_plan_;

        size_t plan_index_;

        mapf_msgs::ActionFeedback current_feedback_;

        ros::Time start_time_, last_compute_time;
        double carry_over_time_, time_step_;

        std::vector<std::pair<double, double>> history_buffer_;
        size_t history_buffer_index_;
        int history_buffer_size_;

        std::mutex plan_mutex_;
    };
}

#endif
