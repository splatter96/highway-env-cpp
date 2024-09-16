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

#include <Eigen/Dense>
//#include <pybind11/eigen.h>
//#include <pybind11/numpy.h>
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>

typedef Eigen::Vector2d Vector;
typedef std::tuple<std::string, std::string, int> LaneIndex;

class RoadNetwork{
public:
    std::map<std::string, std::map<std::string, std::vector<std::unique_ptr<AbstractLane>>>> graph;

    std::vector<LaneIndex> lane_indices;
    std::vector<std::unique_ptr<AbstractLane>> lanes;

    RoadNetwork(){ };

    void add_straight_lane(std::string _from, std::string _to, StraightLane lane){
        this->graph[_from][_to].push_back(std::move(std::make_unique<StraightLane>(lane)));
    }

    void add_sine_lane(std::string _from, std::string _to, SineLane lane){
        this->graph[_from][_to].push_back(std::move(std::make_unique<SineLane>(lane)));
    }

    template <typename TLane>
    void add_lane(std::string _from, std::string _to, TLane const& lane){
        /*
        A lane is encoded as an edge in the road network.

        :param _from: the node at which the lane starts.
        :param _to: the node at which the lane ends.
        :param AbstractLane lane: the lane geometry.
        */

        this->graph[_from][_to].push_back(std::make_unique<TLane>(lane));
    }


    // TODO implement catch for missing LaneIndex
    std::unique_ptr<AbstractLane> get_lane(LaneIndex index){
        /*
        Get the lane geometry corresponding to a given index in the road network.

        :param index: a tuple (origin node, destination node, lane id on the road).
        :return: the corresponding lane geometry.
        */
        std::string _from;
        std::string _to;
        int _id;
        std::tie(_from, _to, _id) = index;

        return std::move(this->graph[_from][_to][_id]->clone());
    }


    LaneIndex get_closest_lane_index(Vector position, float heading){
        /*
        Get the index of the lane closest to a world position.

        :param position: a world position [m].
        :param heading: a heading angle [rad].
        :return: the index of the closest lane.
        */

        if (this->lane_indices.size() == 0){
            for ( const auto &[_from, to_dict]: this->graph ) {
                for ( const auto &[_to, lanes]: to_dict ) {
                    for (size_t i = 0; i < lanes.size(); i++){
                        this->lane_indices.push_back(LaneIndex(_from, _to, i));
                        this->lanes.push_back(this->get_lane(LaneIndex(_from, _to, i)));
                    }
                }
            }
        }


        float current_smallest = 1e8;
        LaneIndex closest_lane_index;

        for (size_t i = 0; i < this->lane_indices.size(); i++){
            auto index = this->lane_indices[i];
            std::unique_ptr<AbstractLane> &lane = this->lanes[i];
            float curr_dist = lane->distance_with_heading(position, heading);
            if(curr_dist < current_smallest){
                current_smallest = curr_dist;
                closest_lane_index = index;
            }
        }

        return closest_lane_index;
    }

    LaneIndex next_lane(LaneIndex current_index, Vector position){
        /*
        Get the index of the next lane that should be followed after finishing the current lane.

        :param current_index: the index of the current lane.
        :param position: the vehicle position.
        :return: the index of the next lane to be followed when current lane is finished.
        */

        std::string _from;
        std::string _to;
        int _id;
        std::tie(_from, _to, _id) = current_index;

        std::vector<LaneIndex> lane_candidates;

        for ( const auto &[next_to, v]: this->graph[_to] ) {
            for (size_t i = 0; i < this->graph[_to][next_to].size(); i++){
                lane_candidates.push_back(LaneIndex(_to, next_to, i));
            }
        }

        return *std::min_element(lane_candidates.begin(), lane_candidates.end(),
                    [this, position] (const LaneIndex l1, const LaneIndex l2) { return this->get_lane(l1)->distance(position) < this->get_lane(l2)->distance(position);}
                );

    }
};

#endif // ROAD_H
