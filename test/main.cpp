#include <iostream>
#include <memory>

#include "../src/lane.h"
#include "../src/road.h"

typedef Eigen::Vector2d Vector;

int main(int argc, char *argv[])
{
    RoadNetwork road = RoadNetwork();

    auto lane1 = StraightLane(Vector(10, 20), Vector(50,20), 4.0, {LineType::CONTINUOUS_LINE, LineType::CONTINUOUS_LINE}, false);
    auto lane2 = StraightLane(Vector(20, 30), Vector(50,30), 4.0, {LineType::CONTINUOUS_LINE, LineType::CONTINUOUS_LINE}, false);

    //road.add_lane<StraightLane>("a", "b", lane1);
    //road.set(lane1);
    //road.set(std::make_unique<StraightLane>(lane1));
    road.add_straight_lane(lane1);
    std::cout << "set lane" << std::endl;

    //auto laneout = road.get_lane({"a", "b", 0}).get();
    auto laneout = road.get(0);

    std::cout << "Test " << laneout->start << std::endl;
    std::cout << "Test " << laneout->end << std::endl;
    //laneout->heading_at(15);


    std::cout << "Calliing distance" << std::endl;
    lane1.distance(Vector(15,25));
    //std::cout << "Calliing width at" << std::endl;
    //lane1.width_at(15);

    auto dist = laneout->distance(Vector(15,25));
    std::cout << dist << std::endl;

}
