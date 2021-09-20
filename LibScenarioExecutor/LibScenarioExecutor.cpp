#include <pybind11/pybind11.h>
#include "ScenarioExecutor.hpp"
  
PYBIND11_MODULE(LibScenarioExecutor, m)
{
    pybind11::class_<SimIniVal>(m, "SimIniVal")
        .def_readwrite("map_id", &SimIniVal::map_id)
        .def_readwrite("wp_id", &SimIniVal::wp_id)
        .def_readwrite("ego_model_id", &SimIniVal::ego_model_id)
        .def_readwrite("ego_color_r", &SimIniVal::ego_color_r)
        .def_readwrite("ego_color_g", &SimIniVal::ego_color_g)
        .def_readwrite("ego_color_b", &SimIniVal::ego_color_b)
        .def_readwrite("ego_location_x", &SimIniVal::ego_location_x)
        .def_readwrite("ego_location_y", &SimIniVal::ego_location_y)
        .def_readwrite("ego_location_z", &SimIniVal::ego_location_z)
        .def_readwrite("ego_rotation_x", &SimIniVal::ego_rotation_x)
        .def_readwrite("ego_rotation_y", &SimIniVal::ego_rotation_y)
        .def_readwrite("ego_rotation_z", &SimIniVal::ego_rotation_z)
        .def_readwrite("pedestrian_count", &SimIniVal::pedestrian_count)
        .def_readwrite("obstacle_count", &SimIniVal::obstacle_count);

    pybind11::class_<PedestrianIniVal>(m, "PedestrianIniVal")
        .def_readwrite("model_id", &PedestrianIniVal::model_id)
        .def_readwrite("location_x", &PedestrianIniVal::location_x)
        .def_readwrite("location_y", &PedestrianIniVal::location_y)
        .def_readwrite("location_z", &PedestrianIniVal::location_z)
        .def_readwrite("rotation_z", &PedestrianIniVal::rotation_z);
 
    pybind11::class_<PedestrianVal>(m, "PedestrianVal")
        .def_readwrite("speed", &PedestrianVal::speed)
        .def_readwrite("direction_x", &PedestrianVal::direction_x)
        .def_readwrite("direction_y", &PedestrianVal::direction_y);

    pybind11::class_<ObstacleIniVal>(m, "ObstacleIniVal")
        .def_readwrite("model_id", &ObstacleIniVal::model_id)
        .def_readwrite("location_x", &ObstacleIniVal::location_x)
        .def_readwrite("location_y", &ObstacleIniVal::location_y)
        .def_readwrite("location_z", &ObstacleIniVal::location_z)
        .def_readwrite("rotation_z", &ObstacleIniVal::rotation_z);

    m.def("attachScenarioExecutor", &attachScenarioExecutor, "");
    m.def("detachScenarioExecutor", &detachScenarioExecutor, "");
    m.def("loadScenario", &loadScenario, "");
    m.def("loadWaypoint", &loadWaypoint, "");
    m.def("getSimIniVal", &getSimIniVal, "");
    m.def("startSimulation", &startSimulation, "");
    m.def("endSimulation", &endSimulation, "");
    m.def("updateEgoStatus", &updateEgoStatus, "");
    m.def("getPedestrianStartPosition", &getPedestrianStartPosition, "");
    m.def("updatePedestrianStatus", &updatePedestrianStatus, "");
    m.def("getObstaclePosition", &getObstaclePosition, "");
}
