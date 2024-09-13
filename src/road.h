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

#include <Eigen/Dense>
//#include <pybind11/eigen.h>
//#include <pybind11/numpy.h>
//#include <pybind11/pybind11.h>
//#include <pybind11/stl.h>

typedef Eigen::Vector2d Vector;
typedef std::tuple<std::string, std::string, int> LaneIndex;

class RoadNetwork{
public:
    std::vector<LaneIndex> lane_indices;
    std::vector<AbstractLane> lanes;
    std::map<std::string, std::map<std::string, std::vector<std::unique_ptr<AbstractLane>>>> graph;

    std::vector<std::unique_ptr<AbstractLane>> my_road;

    RoadNetwork(){ };


    template <typename TLane>
    void set(TLane const& lane){
        this->my_road.push_back(std::make_unique<TLane>(lane));
    }

    void add_straight_lane(std::string _from, std::string _to, StraightLane lane){
        this->graph[_from][_to].push_back(std::move(std::make_unique<StraightLane>(lane)));
    }

    //void add_straight_lane(StraightLane lane){
        //this->my_road.push_back(std::move(std::make_unique<StraightLane>(lane)));
    //}


    void add_sine_lane(SineLane lane){
        this->my_road.push_back(std::move(std::make_unique<SineLane>(lane)));
    }


    std::unique_ptr<AbstractLane> get(int i){
        std::cout <<"FROM CPP:  " << std::endl;
        std::cout <<"cPP:  " << my_road[i]->length << std::endl;
        return std::move(this->my_road[i]->clone());
    }

    // TODO implement catch for missing LaneIndex
    std::unique_ptr<AbstractLane> get_lane2(std::string _from, std::string _to, int num){
        return std::move(this->graph[_from][_to][num]->clone());
    }


    template <typename TLane>
    void add_lane(std::string _from, std::string _to, TLane const& lane){
        /*
        A lane is encoded as an edge in the road network.

        :param _from: the node at which the lane starts.
        :param _to: the node at which the lane ends.
        :param AbstractLane lane: the lane geometry.
        */
        //if _from not in this->graph:
            //this->graph[_from] = {}
        //if _to not in this->graph[_from]:
            //this->graph[_from][_to] = []

        this->graph[_from][_to].push_back(std::make_unique<TLane>(lane));
        //this->graph[_from][_to].push_back(std::unique_ptr<AbstractLane>(lane));
    }


    std::unique_ptr<AbstractLane> get_lane(LaneIndex index){
        /*
        Get the lane geometry corresponding to a given index in the road network.

        :param index: a tuple (origin node, destination node, lane id on the road).
        :return: the corresponding lane geometry.
        */
        //_from, _to, _id = index
        std::string _from;
        std::string _to;
        int _id;
        std::tie(_from, _to, _id) = index;

        //if _id is None and len(self.graph[_from][_to]) == 1:
            //_id = 0

        //return std::unique_ptr<AbstractLane>(this->graph[_from][_to][_id]);
        return std::move(this->graph[_from][_to][_id]);
    }

};

    //def get_closest_lane_index(self, position: np.ndarray, heading: Optional[float] = None) -> LaneIndex:
        //"""
        //Get the index of the lane closest to a world position.

        //:param position: a world position [m].
        //:param heading: a heading angle [rad].
        //:return: the index of the closest lane.
        //"""

        //if not self.lane_indices:
            //for _from, to_dict in self.graph.items():
                //for _to, lanes in to_dict.items():
                    //for _id, l in enumerate(lanes):
                        //self.lane_indices.append((_from, _to, _id))
                        //self.lanes.append(self.get_lane((_from, _to, _id)))

        //cdef float current_smallest = 1e8
        //cdef Tuple [str, str, int] closest_lane_index
        //cdef float curr_dist
        //cdef int i, l_len
        //# cdef AbstractLane lane
        //cdef object lane

        //l_len = len(self.lane_indices)

        //for i in range(l_len):
            //index = self.lane_indices[i]
            //lane = self.lanes[i]
            //curr_dist = lane.distance_with_heading(position, heading)
            //if  curr_dist < current_smallest:
                //current_smallest = curr_dist
                //closest_lane_index = index

        //# print("Done \n\n")

        //return closest_lane_index

    //def next_lane(self, current_index: LaneIndex, route: Route = None, position: np.ndarray = None,
                  //np_random: np.random.RandomState = np.random) -> LaneIndex:
        //"""
        //Get the index of the next lane that should be followed after finishing the current lane.

        //- If a plan is available and matches with current lane, follow it.
        //- Else, pick closest next road
        //- If it has the same number of lanes as current road, stay in the same lane.
        //- Else, pick next road's closest lane.
        //:param current_index: the index of the current lane.
        //:param route: the planned route, if any.
        //:param position: the vehicle position.
        //:param np_random: a source of randomness.
        //:return: the index of the next lane to be followed when current lane is finished.
        //"""
        //_from, _to, _id = current_index
        //next_to = None
        //# Pick next road according to planned route
        //if route:
            //if route[0][:2] == current_index[:2]:  # We just finished the first step of the route, drop it.
                //route.pop(0)
            //if route and route[0][0] == _to:  # Next road in route is starting at the end of current road.
                //_, next_to, _ = route[0]
            //# elif route:
                //# logger.warning("Route {} does not start after current road {}.".format(route[0], current_index))

        //# pick closest next road
        //if not next_to:
            //try:
                //# next_to = list(self.graph[_to].keys())[np.random.randint(len(self.graph[_to]))]

                //lane_candidates = []
                //for next_to in list(self.graph[_to].keys()):
                    //for l in range(len(self.graph[_to][next_to])):
                        //lane_candidates.append((_to, next_to, l))

                //next_lane = min(lane_candidates, key=lambda l: self.get_lane(l).distance(position))
                //return next_lane
            //except KeyError:
                //return current_index

#endif // ROAD_H
