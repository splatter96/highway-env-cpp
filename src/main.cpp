
#include "lane.h"
#include "road.h"
#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

typedef Eigen::Vector2d Vector;

namespace py = pybind11;
PYBIND11_MODULE(python_example, m)
{
    py::enum_<LineType>(m, "LineType", py::arithmetic())
        .value("NONE", LineType::NONE)
        .value("STRIPED", LineType::STRIPED)
        .value("CONTINUOUS", LineType::CONTINUOUS)
        .value("CONTINUOUS_LINE", LineType::CONTINUOUS_LINE)
        .export_values();

    py::class_<AbstractLane>(m, "AbstractLane")
        .def_readwrite("length", &AbstractLane::length)
        .def_readwrite("forbidden", &AbstractLane::forbidden)
        .def_readwrite("start", &AbstractLane::start)
        .def_readwrite("end", &AbstractLane::end)
        .def_readwrite("line_types", &AbstractLane::line_types);

    py::class_<StraightLane, AbstractLane>(m, "StraightLane")
        .def(py::init<Vector, Vector, float, std::array<LineType, 2>, bool>(), py::arg(), py::arg(), py::arg(), py::arg(), py::arg("forbidden")=false)
        .def("position", &StraightLane::position)
        .def("heading_at", &StraightLane::heading_at)
        .def("width_at", &StraightLane::width_at)
        .def("local_coordinates", &StraightLane::local_coordinates)
        .def_readwrite("direction", &StraightLane::direction)
        .def_readwrite("direction_lateral", &StraightLane::direction_lateral)
        .def_readwrite("width", &StraightLane::width)
        .def_readwrite("heading", &StraightLane::heading);

    py::class_<SineLane, AbstractLane>(m, "SineLane")
        .def(py::init<Vector, Vector, float, float, float, float, std::array<LineType, 2>, bool>(), py::arg(), py::arg(), py::arg(), py::arg(), py::arg(), py::arg(), py::arg(),  py::arg("forbidden")=false)
        .def("position", &SineLane::position)
        .def("heading_at", &SineLane::heading_at)
        .def("local_coordinates", &SineLane::local_coordinates)
        .def_readwrite("amplitude", &SineLane::amplitude)
        .def_readwrite("pulsation", &SineLane::pulsation)
        .def_readwrite("phase", &SineLane::phase);

    py::class_<RoadNetwork>(m, "RoadNetwork")
        .def(py::init<>())
        .def("get", &RoadNetwork::get)
        .def("add_straight_lane", &RoadNetwork::add_straight_lane)
        .def("add_sine_lane", &RoadNetwork::add_sine_lane)
        .def("get_lane", &RoadNetwork::get_lane2);
        //.def("add_lane", &RoadNetwork::add_lane);
        //.def("get_lane", &RoadNetwork::get_lane);
}
