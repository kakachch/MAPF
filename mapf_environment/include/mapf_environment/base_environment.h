/*
 * @brief        :
 * @Author       : Quyicheng
 * @Date         : 2024-03-04 21:49:40
 * @LastEditors  : Quyicheng
 * @LastEditTime : 2024-03-15 23:25:56
 */

#pragma once
#ifndef MAPF_ENVIRONMENT_BASE_ENVIRONMENT_H
#define MAPF_ENVIRONMENT_BASE_ENVIRONMENT_H
#include <vector>
#include <string>
#include <unordered_set>
#include <boost/functional/hash/hash.hpp>
#include "orientation.h"

struct Pose
{
  int location;
  int orientation;

  Pose():location(0),orientation(0){}

  Pose(const int &location, const int &orientation) : location(location), orientation(orientation) {}
};

struct State
{
  int time;
  int location;
  int orientation;
  State() : time(0), location(0) {}

  State(int time, int location, int orientation) : time(time), location(location), orientation(orientation) {}

  State(int time, const Pose &pose) : time(time), location(pose.location), orientation(pose.orientation) {}

  bool operator==(const State &state) const
  {
    return time == state.time && location == state.location;
  }

  Pose getPose() const
  {
    return Pose(location, orientation);
  }
};

namespace std
{
  template <>
  struct hash<State>
  {
    size_t operator()(const State &s) const
    {
      size_t seed = 0;
      boost::hash_combine(seed, s.time);
      boost::hash_combine(seed, s.location);
      boost::hash_combine(seed, s.orientation);
      return seed;
    }
  };
} // namespace std

namespace mapf_environment
{

  class BaseEnvironment
  {

  public:
    virtual void initialize(std::string name) = 0;

    virtual bool worldToId(const double &world_x, const double &world_y, int &id) = 0;

    virtual bool idToWorld(const int &id, double &world_x, double &world_y) = 0;

    virtual bool getNeighbors(const Pose &current, std::vector<Pose> &neighbors, std::vector<double> &costs) = 0;

    virtual std::string getGlobalFrameID() = 0;

    virtual void setObstacles(const std::unordered_set<int> &obstacle_set) = 0;

    double getEuclideanDistance(const int &id_1, const int &id_2);

    double getManhattanDistance(const int &id_1, const int &id_2);

    virtual ~BaseEnvironment() {}
  };

}

#endif