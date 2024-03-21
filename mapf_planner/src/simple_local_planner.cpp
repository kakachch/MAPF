/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-02-20 11:56:41
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-18 11:18:44
 */
#include <mapf_planner/simple_local_planner.h>

#include <base_local_planner/goal_functions.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(mapf_planner::SimpleLocalPlanner, nav_core::BaseLocalPlanner)

template <typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

namespace mapf_planner
{
    void SimpleLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf, costmap_2d::Costmap2DROS *costmap_ros)
    {
        if (!initialized_)
        {
            ros::NodeHandle private_nh("~/" + name);

            costmap_ros_ = costmap_ros;
            tf_ = tf;

            global_frame_ = costmap_ros_->getGlobalFrameID();
            robot_base_frame_ = costmap_ros_->getBaseFrameID();
            std::string odom_topic;
            private_nh.param<std::string>("odom_topic", odom_topic, "odom");
            odom_helper_.setOdomTopic(odom_topic);

            private_nh.param<double>("yaw_goal_tolerance", yaw_goal_tolerance_, 0.2);
            private_nh.param<double>("xy_goal_tolerance", xy_goal_tolerance_, 0.2);
            private_nh.param<double>("acc_lim_x", acc_lim_x_, 2.5);
            private_nh.param<double>("acc_lim_theta", acc_lim_theta_, 4.0);
            private_nh.param<bool>("holonomic_robot", holonomic_robot_, true);
            private_nh.param<double>("max_vel_x", max_vel_x_, 0.5);
            private_nh.param<double>("min_vel_x", min_vel_x_, 0.0);
            private_nh.param<double>("max_vel_theta", max_vel_theta_, 1.0);
            private_nh.param<double>("time_step", time_step_, 6.0);
            private_nh.param<int>("smoothing_factor", smoothing_factor_, 0);
            private_nh.param<int>("history_buffer_size", history_buffer_size_, 1);

            plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);

            initialized_ = true;
            history_buffer_index_ = 0;

            feedback_modified_ = true;
        }
    }
    bool SimpleLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped> &plan)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        start_time_ = ros::Time::now();
        last_compute_time = start_time_;

        base_local_planner::publishPlan(plan, plan_pub_);

        std::unique_lock<std::mutex> lock(plan_mutex_);

        if (current_plan_.empty())
        {
            plan_index_ = 0;
        }
        else
        {
            geometry_msgs::Point plan_point = current_plan_[plan_index_].pose.position;
            double cur_x = plan_point.x, cur_y = plan_point.y;

            plan_index_ = 0;
            for (size_t i = 0; i < plan.size(); i++)
            {
                geometry_msgs::Point point = plan[i].pose.position;
                if (point.x == cur_x && point.y == cur_y)
                {
                    plan_index_ = i;
                    break;
                }
            }
        }
        current_plan_ = plan;
        carry_over_time_ = time_step_ * plan_index_;
        carry_over_time_ -= ros::Time::now().toSec() - start_time_.toSec();
        start_time_ = ros::Time::now();

        current_feedback_.plan_index = plan_index_;
        feedback_modified_ = true;
        lock.unlock();

        return true;
    }
    bool SimpleLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        bool rotate_step = plan_index_ > 0 && current_plan_[plan_index_].pose.orientation != current_plan_[plan_index_ - 1].pose.orientation;

        geometry_msgs::PoseStamped current_pose, smoothed_pose, transformed_smoothed_pose;
        costmap_ros_->getRobotPose(current_pose);

        std::unique_lock<std::mutex> lock(plan_mutex_);
        size_t smoothed_index = std::min<size_t>(plan_index_ + smoothing_factor_, current_plan_.size() - 1);
        smoothed_pose = current_plan_[smoothed_index];
        lock.unlock();

        geometry_msgs::TransformStamped plan_to_global_transform = tf_->lookupTransform(global_frame_, ros::Time(),
                                                                                        smoothed_pose.header.frame_id, smoothed_pose.header.stamp, smoothed_pose.header.frame_id, ros::Duration(0.5));
        tf2::doTransform(smoothed_pose, transformed_smoothed_pose, plan_to_global_transform);

        double smoothed_distance, smoothed_thelta, distance, theta;

        getDistanceAndThelta(transformed_smoothed_pose.pose, current_pose.pose, smoothed_distance, smoothed_thelta);

        distance = smoothed_distance / (smoothing_factor_ + 1);

        nav_msgs ::Odometry robot_vel;
        odom_helper_.getOdom(robot_vel);

        double sim_period = ros::Time::now().toSec() - last_compute_time.toSec();
        last_compute_time = ros::Time::now();

        double remaining_second = time_step_ - ros::Time::now().toSec() + start_time_.toSec();

        if (!rotate_step)
        {
            theta = smoothed_thelta;
            if (distance <= xy_goal_tolerance_)
            {
                current_feedback_.pose = current_pose;
                current_feedback_.plan_index = plan_index_;

                carry_over_time_ += ros::Time::now().toSec() - start_time_.toSec() - time_step_;
                start_time_ = ros::Time::now();
                ROS_INFO("carry over time: %fs reach goal index: %d", carry_over_time_, plan_index_);

                plan_index_++;
                return true;
            }
        }
        else
        {
            theta = tf2::getYaw(transformed_smoothed_pose.pose.orientation) - tf2::getYaw(current_pose.pose.orientation);
            theta = theta > M_PI ? theta - 2 * M_PI : (theta < -M_PI ? theta + 2 * M_PI : theta);
            if (theta <= yaw_goal_tolerance_)
            {
                current_feedback_.pose = current_pose;
                current_feedback_.plan_index = plan_index_;
                carry_over_time_ += ros::Time::now().toSec() - start_time_.toSec() - time_step_;
                start_time_ = ros::Time::now();
                plan_index_++;
                ROS_INFO("carry over time: %fs reach goal index: %d", carry_over_time_, plan_index_);

                return true;
            }
            distance = 0;
        }
        double vel_theta = 0, vel_x = 0;

        double cur_vel_theta = robot_vel.twist.twist.angular.z;

        vel_theta = sgn(theta) * sqrt(2 * acc_lim_theta_ * fabs(theta));

        double max_vel_theta = std::min<double>(max_vel_theta_, cur_vel_theta + acc_lim_theta_ * sim_period);
        double min_vel_theta = std::max<double>(-max_vel_theta_, cur_vel_theta - acc_lim_theta_ * sim_period);
        vel_x = distance / std::max<double>(1e-7, remaining_second);
        double cur_vel_x = robot_vel.twist.twist.linear.x;
        double max_vel_x = std::min<double>(max_vel_x_, cur_vel_x + acc_lim_x_ * sim_period);
        double min_vel_x = std::max<double>(min_vel_x_, cur_vel_x - acc_lim_x_ * sim_period);

        vel_theta = std::max<double>(min_vel_theta, std::min<double>(max_vel_theta, vel_theta));

        if (fabs(theta) > M_PI / 6)
        {
            vel_x = min_vel_x;
        }
        else
        {
            vel_x = std::max<double>(min_vel_x, std::min<double>(max_vel_x, vel_x));
        }
        cmd_vel.linear.x = vel_x;
        cmd_vel.angular.z = vel_theta;

        ROS_INFO("vel x %f theta %f", vel_x, vel_theta);
        return true;
    }

    bool SimpleLocalPlanner::isGoalReached()
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        return plan_index_ == current_plan_.size();
    }
    bool SimpleLocalPlanner::getFeedback(mapf_msgs::ActionFeedback &feedback)
    {
        if (!initialized_)
        {
            ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
            return false;
        }
        if (feedback_modified_)
        {
            feedback = current_feedback_;
            feedback_modified_ = false;
            return true;
        }
        else
        {
            return false;
        }
    }

    bool SimpleLocalPlanner::getDistanceAndThelta(const geometry_msgs::Pose &target_pose, const geometry_msgs::Pose &source_pose, double &distance, double &theta)
    {
        const geometry_msgs::Point &target_point = target_pose.position;
        const geometry_msgs::Point &source_point = source_pose.position;

        double dx = target_point.x - source_point.x;
        double dy = target_point.y - source_point.y;

        distance = hypot(dx, dy);

        double target_angle = std::atan2(dy, dx);
        theta = target_angle - tf2::getYaw(source_pose.orientation);

        theta = theta > M_PI ? theta - 2 * M_PI : theta;
        theta = theta <= -M_PI ? theta + 2 * M_PI : theta;

        return true;
    }
}

// TODO history_buffer