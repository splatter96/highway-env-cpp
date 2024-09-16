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

#include "road.h"

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
            float speed){

        this->road = road;
        this->position = position;
        this->heading = heading;
        this->speed = speed;
        this->lane_index = this->road->get_closest_lane_index(this->position, this->heading);
        this->crashed = false;
    };

    void clip_actions(){
        if (this->crashed){
            this->steering_action = 0;
            this->acceleration_action = -1.0 * this->speed;
        }

        if (this->speed > this->MAX_SPEED)
            this->acceleration_action = std::min(this->acceleration_action, this->MAX_SPEED - this->speed);
        else if (this->speed < -this->MAX_SPEED)
            this->acceleration_action = std::max(this->acceleration_action, this->MAX_SPEED - this->speed);
    }

    void on_state_update(){
        this->lane_index = this->road->get_closest_lane_index(this->position, this->heading);
        this->lane = this->road->get_lane(this->lane_index);
    }

    void step(float dt){
    /*
    Propagate the vehicle state given its actions.

    Integrate a modified bicycle model with a 1st-order response on the steering wheel dynamics.
    If the vehicle is crashed, the actions are overridden with erratic steering and braking until complete stop.
    The vehicle's current lane is updated.

    :param dt: timestep of integration of the model [s]
    */

        this->clip_actions();
        float delta_f = this->steering_action;
        float beta = atan(0.5 * tan(delta_f));

        Vector h;
        h << cos(heading + beta), sin(heading + beta);

        Vector v = this->speed * h;
        this->position += v * dt;
        this->heading += this->speed * sin(beta) / (this->LENGTH / 2) * dt;
        //TODO
        //this->heading = utils.wrap_to_pi(self.heading);
        this->speed += this->acceleration_action * dt;
        this->on_state_update();
    }
};

class RoadObject{
    RoadObject(){};
};



#endif // KINEMATICS_H
