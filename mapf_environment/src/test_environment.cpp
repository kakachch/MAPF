/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-14 09:40:26
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-16 22:45:20
 */
#include <mapf_environment/test_environment.h>
#include <mapf_environment/orientation.h>

namespace mapf_environment
{
    TestEnvironment::TestEnvironment()
    {
        initialize(" ");
    }
    void TestEnvironment::initialize(std::string name)
    {
        map_width_ = 45;
        map_height_ = 17;

        std::vector<int> obstacle_vec{6, 7, 36, 52, 77, 100, 125, 174};

        free_map_.resize(map_width_ * map_height_, true);

        for (const int &obstacle : obstacle_vec)
        {
            free_map_[obstacle] = false;
        }
    }

    bool TestEnvironment::worldToId(const double &world_x, const double &world_y, int &id)
    {
        assert(world_x < map_width_ && world_y < map_height_);
        return world_y * map_width_ + world_x;
    }

    bool TestEnvironment::idToWorld(const int &id, double &world_x, double &world_y)
    {
        world_x = id % map_width_;
        world_y = id / map_width_;
        return true;
    }

    bool TestEnvironment::getNeighbors(const Pose &current, std::vector<Pose> &neighbors, std::vector<double> &costs)
    {
        int map_x = current.location % map_width_, map_y = current.location / map_width_;
        int orientation = current.orientation;
        auto checkAndInsert = [&](int target_x, int target_y, int orientation, double cost)
        {
            int target_id = target_y * map_width_ + target_x;
            if (target_x < map_width_ && target_y < map_height_ && obstacle_set_.find(target_id) != obstacle_set_.end())
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

    std::string TestEnvironment::getGlobalFrameID()
    {
        return "";
    }

    void TestEnvironment::setObstacles(const std::unordered_set<int> &obstacle_set)
    {
        obstacle_set_ = obstacle_set;
    }

    TestEnvironment::~TestEnvironment()
    {
    }
}