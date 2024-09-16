
import numpy as np
import python_example
from python_example import LineType, StraightLane, SineLane, RoadNetwork

start = np.array([10, 20])
end = np.array([90, 20])

linetypes = [python_example.LineType.CONTINUOUS, python_example.LineType. CONTINUOUS]

lane = python_example.StraightLane(start, end, 4., linetypes, False)
lane = python_example.StraightLane(start, end, 4., linetypes)

# print(lane)

# print(lane.length)
# print(lane.start)
# print(lane.end)

# print(lane.heading_at(25))
# print(lane.width_at(25))

# print(lane.local_coordinates(np.array([15, 25])))

def _make_road() -> None:
    """
    Make a road composed of a straight highway-env and a merging lane.

    :return: the road
    """
    net = RoadNetwork()
    DEFAULT_WIDTH = 4

    # Highway lanes
    ends = [150, 80, 80, 150]  # Before, converging, merge, after

    c, s, n = LineType.CONTINUOUS_LINE, LineType.STRIPED, LineType.NONE
    y = [0, DEFAULT_WIDTH]
    line_type = [(c, s), (n, c)]
    line_type_merge = [(c, s), (n, s)]

    # net.add_straight_lane(StraightLane(np.array([0, 1]), np.array([sum(ends[:2]), y[0]]), DEFAULT_WIDTH, line_type[0]))

    # ljk = StraightLane(np.array([0, 6.5 + 4 + 4]), np.array([ends[0], 6.5 + 4 + 4]), DEFAULT_WIDTH, (c, c), forbidden=True)
    # amplitude = 3.25
    # net.add_sine_lane(SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude), amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, DEFAULT_WIDTH, (c, c), forbidden=True))

    # l1 = net.get(0)
    # print(l1)
    # print(l1.length)
    # print(net.get(0).heading)

    for i in range(2):
        # net.add_lane("a", "b", StraightLane(np.array([0, y[i]]), np.array([sum(ends[:2]), y[i]]), DEFAULT_WIDTH, line_type[i]))
        # net.add_lane("b", "c", StraightLane(np.array([sum(ends[:2]), y[i]]), np.array([sum(ends[:3]), y[i]]), DEFAULT_WIDTH, line_type_merge[i]))
        # net.add_lane("c", "d", StraightLane(np.array([sum(ends[:3]), y[i]]), np.array([sum(ends[:4]), y[i]]), DEFAULT_WIDTH, line_type_merge[i]))
        net.add_straight_lane("a", "b", StraightLane(np.array([0, y[i]]), np.array([sum(ends[:2]), y[i]]), DEFAULT_WIDTH, line_type[i]))
        net.add_straight_lane("b", "c", StraightLane(np.array([sum(ends[:2]), y[i]]), np.array([sum(ends[:3]), y[i]]), DEFAULT_WIDTH, line_type_merge[i]))
        net.add_straight_lane("c", "d", StraightLane(np.array([sum(ends[:3]), y[i]]), np.array([sum(ends[:4]), y[i]]), DEFAULT_WIDTH, line_type_merge[i]))

    # Merging lane

    # lkb = SineLane(ljk.position(ends[0], -amplitude), ljk.position(sum(ends[:2]), -amplitude), amplitude, 2 * np.pi / (2*ends[1]), np.pi / 2, DEFAULT_WIDTH, (c, c), forbidden=True)

    # lbc = StraightLane(lkb.position(ends[1], 0), lkb.position(ends[1], 0) + [ends[2], 0], DEFAULT_WIDTH, (n, c), forbidden=False)
    # # #off ramp
    # # lco = StraightLane(lcd.position(ends[2], 0), lcd.position(ends[2]+80, 6.5), line_types=[c, c], forbidden=True)
    # lco = StraightLane(lbc.position(ends[2], 0), lbc.position(ends[2]+80, 6.5), DEFAULT_WIDTH, (c, c), forbidden=True)
    # lou = StraightLane([sum(ends[:3])+80, 6.5+4+4], [sum(ends[:3])+80+70, 6.5+4+4], DEFAULT_WIDTH, (c, c), forbidden=True)


    # net.add_lane("j", "k", ljk)
    # net.add_lane("k", "b", lkb)
    # net.add_lane("b", "c", lbc)
    # net.add_lane("c", "o", lco)
    
    # net.add_lane("o", "u", lou) #off ramp
    # road = Road(network=net, np_random=self.np_random, record_history=self.config["show_trajectories"])
    # self.road = road

    # net.get_lane(("j", "k", 0))
    # print(net.get_lane(("a", "b", 0)).length)
    # print(net.get_lane(("a", "b", 0)))
    # print(net.get_lane(("a", "b", 0)).start)
    # print(net.get_lane(("a", "b", 0)).end)

    next_lane = net.next_lane(("a", "b", 0), np.array([0,0]))
    print(next_lane)
    print(net.get_lane(next_lane).start)

    net.get_closest_lane_index(np.array([0,0]), 0.4)
    net.get_closest_lane_index(np.array([0,0]), 0.4)
    net.get_closest_lane_index(np.array([0,0]), 0.4)
    dist = net.get_closest_lane_index(np.array([240,0]), 0.4)
    print(dist)

_make_road()
