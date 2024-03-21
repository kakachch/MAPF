/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-02 16:04:47
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-16 22:46:33
 */
#include <std_msgs/ColorRGBA.h>
#include <mapf_environment/grid_environment.h>
#include <mapf_environment/orientation.h>

template <typename T>
bool parseVector(const XmlRpc::XmlRpcValue &xmlrpc, std::vector<T> &result)
{
    if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        return false;
    }
    for (int i = 0; i < xmlrpc.size(); ++i)
    {
        const XmlRpc::XmlRpcValue &value = xmlrpc[i];
        result.emplace_back((T)value);
    }
    return true;
}

template <typename T1, typename T2, typename T3>
bool parseVectorTuple(const XmlRpc::XmlRpcValue &xmlrpc, std::vector<std::tuple<T1, T2, T3>> &result)
{
    if (xmlrpc.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
        return false;
    }
    for (int i = 0; i < xmlrpc.size(); ++i)
    {
        const XmlRpc::XmlRpcValue &value = xmlrpc[i];
        if (value.getType() != XmlRpc::XmlRpcValue::TypeArray)
        {
            return false;
        }
        if (value.size() != 3)
        {
            return false;
        }
        result.emplace_back(std::make_tuple((T1)value[0], (T2)value[1], (T3)value[2]));
    }
    return true;
}

namespace mapf_environment
{
    GridEnvironment::GridEnvironment(std::string name)
    {
        initialize(name);
    }

    void GridEnvironment::initialize(std::string name)
    {
        ros::NodeHandle private_nh("~/" + name);
        ros::NodeHandle nh;

        private_nh.param<int>("map_width", map_width_, 45);
        private_nh.param<int>("map_height", map_height_, 17);
        private_nh.param<std::string>("global_frame", global_frame_, "map");

        XmlRpc::XmlRpcValue xmlrpc;

        private_nh.getParam("x_coords", xmlrpc);

        if (!parseVector<double>(xmlrpc, x_coords_))
        {
            ROS_ERROR("grid environment: parse vector x_coords error!");
            return;
        }

        if (x_coords_.size() != (size_t)map_width_)
        {
            ROS_ERROR("grid environment: x_coords size error!");
            return;
        }

        private_nh.getParam("y_coords", xmlrpc);

        if (!parseVector<double>(xmlrpc, y_coords_))
        {
            ROS_ERROR("grid environment: parse vector y_coords error!");
            return;
        }

        if (y_coords_.size() != (size_t)map_height_)
        {
            ROS_ERROR("grid environment: y_coords size error!");
            return;
        }

        grid_map_.resize(map_width_ * map_height_);

        for (int id = 0; (int)id < map_height_ * map_width_; ++id)
        {
            grid_map_[id].x = x_coords_[id % map_width_];
            grid_map_[id].y = y_coords_[id / map_width_];
        }

        std::vector<std::tuple<int, double, double>> modified_grids;

        if (private_nh.hasParam("modified_grids"))
        {
            private_nh.getParam("modified_grids", xmlrpc);

            if (!parseVectorTuple<int, double, double>(xmlrpc, modified_grids))
            {
                ROS_ERROR("grid_environment: parse modified_grids error");
                return;
            }

            for (const auto &grid : modified_grids)
            {
                int id = std::get<0>(grid);
                grid_map_[id].x = std::get<1>(grid);
                grid_map_[id].y = std::get<2>(grid);
            }
        }

        std::vector<int> free_grids;
        private_nh.getParam("free_grids", xmlrpc);

        if (!parseVector<int>(xmlrpc, free_grids))
        {
            ROS_ERROR("grid environment: parse vector y_coords error!");
            return;
        }

        for (const int &id : free_grids)
        {
            if (!idIsValid(id))
            {
                return;
            }

            grid_map_[id].free_space = true;
        }

        grid_marker_pub_ = nh.advertise<visualization_msgs::MarkerArray>("grid_marker", 1, true);

        publishGridMarker();
    }

    bool GridEnvironment::worldToId(const double &world_x, const double &world_y, int &id)
    {
        int x_index = std::distance(x_coords_.begin(), std::lower_bound(x_coords_.begin(), x_coords_.end(), world_x));
        int y_index = std::distance(y_coords_.begin(), std::lower_bound(y_coords_.begin(), y_coords_.end(), world_y));

        if ((size_t)x_index == x_coords_.size())
        {
            --x_index;
        }

        if ((size_t)y_index == y_coords_.size())
        {
            --y_index;
        }

        if (x_coords_[x_index] == world_x && y_coords_[y_index] == world_y)
        {
            id = y_index * map_width_ + x_index;
            return true;
        }

        std::vector<std::vector<int>> grids_vec({{x_index, y_index}, {x_index - 1, y_index}, {x_index, y_index - 1}, {x_index - 1, y_index - 1}});

        double distance = DBL_MAX;
        for (auto &grid : grids_vec)
        {
            if (grid[0] < 0 || grid[1] < 0)
            {
                continue;
            }
            int loc_id = grid[1] * map_width_ + grid[0];
            if (coordIsValid(grid[0], grid[1]) && grid_map_[loc_id].free_space)
            {
                double dist = sqrt(hypot(world_x - x_coords_[grid[0]], world_y - y_coords_[grid[1]]));
                if (dist < distance)
                {
                    distance = dist;
                    id = grid[1] * map_width_ + grid[0];
                }
            }
        }

        if (distance == DBL_MAX)
        {
            ROS_ERROR("grid environment: no free grid found!");
            return false;
        }
        return true;
    }

    bool GridEnvironment::idToWorld(const int &id, double &world_x, double &world_y)
    {
        if (!idIsValid(id) || !grid_map_[id].free_space)
        {
            return false;
        }
        world_x = grid_map_[id].x;
        world_y = grid_map_[id].y;
        return true;
    }

    bool GridEnvironment::getNeighbors(const Pose &current, std::vector<Pose> &neighbors, std::vector<double> &costs)
    {
        int map_x = current.location % map_width_, map_y = current.location / map_width_;
        int orientation = current.orientation;
        auto checkAndInsert = [&](int target_x, int target_y, int orientation, double cost)
        {
            int target_id = target_y * map_width_ + target_x;
            if (!coordIsValid(target_x, target_y) || !grid_map_[target_id].free_space || obstacle_set_.find(target_id) != obstacle_set_.end())
            {
                return;
            }
            neighbors.emplace_back(Pose(target_id, orientation));
            costs.emplace_back(cost);
        };

        std::vector<int> next_orientation_vec, delta_x, delta_y;
        getNextOrientation(orientation, next_orientation_vec, delta_x, delta_y);

        for (size_t i = 0; i < next_orientation_vec.size(); ++i)
        {
            checkAndInsert(map_x + delta_x[i], map_y + delta_y[i], next_orientation_vec[i], 1.0);
        }

        return true;
    }

    void GridEnvironment::setObstacles(const std::unordered_set<int> &obstacle_set)
    {
        obstacle_set_ = obstacle_set;
    }

    bool GridEnvironment::coordIsValid(const int &map_x, const int &map_y)
    {
        return map_x < map_width_ && map_y < map_height_;
    }

    bool GridEnvironment::idIsValid(const int &id)
    {
        if ((int)id >= map_height_ * map_width_ || id > INT_MAX)
        {
            ROS_ERROR("grid environment: map coordinate out of range!");
            return false;
        }
        return true;
    }

    GridEnvironment::~GridEnvironment()
    {
    }

    std::string GridEnvironment::getGlobalFrameID()
    {
        return global_frame_;
    }
    void GridEnvironment::publishGridMarker()
    {
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker lines_marker, points_marker;

        lines_marker.header.frame_id = "map";
        lines_marker.header.stamp = ros::Time::now();
        lines_marker.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker.action = visualization_msgs::Marker::ADD;
        lines_marker.color.r = 0.0f;
        lines_marker.color.g = 1.0f;
        lines_marker.color.b = 0.0f;
        lines_marker.color.a = 1.0f;
        lines_marker.id = 0;
        lines_marker.scale.x = 0.1;
        lines_marker.scale.y = 0.1;
        lines_marker.scale.z = 0.1;
        lines_marker.lifetime = ros::Duration();
        lines_marker.ns = "lines_marker";

        points_marker.header.frame_id = "map";
        points_marker.header.stamp = ros::Time::now();
        points_marker.type = visualization_msgs::Marker::POINTS;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.color.r = 0.0f;
        points_marker.color.g = 1.0f;
        points_marker.color.b = 0.0f;
        points_marker.color.a = 1.0f;
        lines_marker.id = 1;
        points_marker.scale.x = 0.2;
        points_marker.scale.y = 0.2;
        points_marker.scale.z = 0.2;
        points_marker.lifetime = ros::Duration();
        points_marker.ns = "points_marker";

        for (int id = 0; id < map_width_ * map_height_; ++id)
        {
            if (grid_map_[id].free_space)
            {
                geometry_msgs::Point p;
                p.x = grid_map_[id].x;
                p.y = grid_map_[id].y;
                points_marker.points.emplace_back(p);

                if (id % map_width_ < map_width_ - 1)
                {
                    if (grid_map_[id + 1].free_space)
                    {
                        lines_marker.points.emplace_back(p);
                        geometry_msgs::Point tmp;
                        tmp.x = grid_map_[id + 1].x;
                        tmp.y = grid_map_[id + 1].y;
                        lines_marker.points.emplace_back(tmp);
                    }
                }

                if (id / map_width_ < map_height_ - 1)
                {
                    if (grid_map_[id + map_width_].free_space)
                    {
                        lines_marker.points.emplace_back(p);
                        geometry_msgs::Point tmp;
                        tmp.x = grid_map_[id + map_width_].x;
                        tmp.y = grid_map_[id + map_width_].y;
                        lines_marker.points.emplace_back(tmp);
                    }
                }
            }
        }
        marker_array.markers.emplace_back(points_marker);
        marker_array.markers.emplace_back(lines_marker);

        grid_marker_pub_.publish(marker_array);
    }
}
