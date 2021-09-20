#ifndef LaneChangeThread_hpp
#define LaneChangeThread_hpp

#include "ScenarioExecutorCore.hpp"

void createLaneChangeWps(TOtherCarInfo *other_car_info, int new_wps_idx, double lane_offset, double lane_change_time);
void createLaneChangeFineWps(TOtherCarInfo *other_car_info);

#endif