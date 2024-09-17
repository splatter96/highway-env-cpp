#ifndef ROAD_H
#define ROAD_H

#include <array>
#include <string>
#include <map>
#include <memory>
#include <vector>
#include <tuple>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <algorithm>

#include "lane.h"
//#include "kinematics.h"

#include <Eigen/Dense>
//#include <pybind11/eigen.h>
//#include <pybind11/numpy.h>
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>

class Vehicle;

typedef Eigen::Vector2d Vector;
typedef std::tuple<std::string, std::string, int> LaneIndex;

class RoadNetwork{
public:
    std::map<std::string, std::map<std::string, std::vector<std::unique_ptr<AbstractLane>>>> graph;

    std::vector<LaneIndex> lane_indices;
    std::vector<std::unique_ptr<AbstractLane>> lanes;

    std::vector<Vehicle> vehicles;
    //std::vector<RoadObject> objects;

    RoadNetwork();

    void add_straight_lane(std::string _from, std::string _to, StraightLane lane);

    void add_sine_lane(std::string _from, std::string _to, SineLane lane);

    template <typename TLane>
    void add_lane(std::string _from, std::string _to, TLane const& lane);

    std::unique_ptr<AbstractLane> get_lane(LaneIndex index);

    LaneIndex get_closest_lane_index(Vector position, float heading);

    LaneIndex next_lane(LaneIndex current_index, Vector position);

    std::vector<LaneIndex> side_lanes(LaneIndex lane_index);

    void step(float dt);

};

#endif // ROAD_H
