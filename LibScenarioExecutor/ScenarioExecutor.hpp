#ifndef ScenarioExecutor_hpp
#define ScenarioExecutor_hpp

#include <string>
#include "ScenarioJsonLoader.hpp"
#include "WaypointJsonLoader.hpp"
#include "ScenarioExecutorCore.hpp"

using namespace std;

#define SCENARIO_EXE_INFO_PARAM_IDX 0
#define EGO_CAR_INFO_PARAM_IDX 1
#define EGO_CAR_CONTROL_PARAM_IDX 7
#define OTHER_CAR_INFO_PARAM_IDX 11
#define OTHER_CAR_INFO_PARAM_LEN 30
 
int attachScenarioExecutor();
void detachScenarioExecutor();
void loadScenario(string path);
void loadWaypoint(string path);
SimIniVal getSimIniVal();
void startSimulation();
void endSimulation();
bool updateEgoStatus(double location_x, double location_y, double location_z,
                     double rotation_x, double rotation_y, double rotation_z,
                     double velocity_x, double velocity_y, double velocity_z,
                     double collision, double delta_time);
PedestrianIniVal getPedestrianStartPosition(int idx);
PedestrianVal updatePedestrianStatus(int idx, double location_x, double location_y, double delta_time);
ObstacleIniVal getObstaclePosition(int idx);

#endif