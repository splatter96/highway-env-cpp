#ifndef KINEMATICS_H
#define KINEMATICS_H

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

#include <Eigen/Dense>

#include "lane.h"
//#include "road.h"
class RoadNetwork;

typedef Eigen::Vector2d Vector;
typedef std::tuple<std::string, std::string, int> LaneIndex;

/*
    A moving vehicle on a road, and its kinematics.

    The vehicle is represented by a dynamical system: a modified bicycle model.
    It's state is propagated depending on its steering and acceleration actions.
*/
class Vehicle{
public:
    float LENGTH = 5.0;
    float LENGTH_SQUARE = LENGTH * LENGTH; // Nedded for faster distance comparison
    float WIDTH = 2.0;
    float MAX_SPEED = 40.;

    RoadNetwork *road;
    Vector position;
    float heading;
    float speed;
    LaneIndex lane_index;
    std::unique_ptr<AbstractLane> lane;
    bool crashed;
    float steering_action = 0;
    float acceleration_action = 0;

    Vehicle(RoadNetwork *road,
            Vector position,
            float heading,
            float speed);

    void clip_actions();

    void on_state_update();

    void step(float dt);
};

class RoadObject{
    RoadObject();
};

#endif // KINEMATICS_H
