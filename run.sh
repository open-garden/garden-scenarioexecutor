#!/bin/bash
source setup.sh
python ScenarioExecutor.py ./scenarios/highway_cutin.json ./waypoints/highway.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/urban_curvemirror.json ./waypoints/Urban.MAP.500.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.1_1_FreeDriving_TEMPLATE.json ./waypoints/ALKS.Road.Curvatures.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.1_2_SwervingLeadVehicle_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.1_3_SwervingSideVehicle_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.2_1_FullyBlockingTarget_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.2_2_PartiallyBlockingTarget_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.2_3_CrossingPedestrian_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.2_4_MultipleBlockingTargets_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.3_1_FollowLeadVehicleComfortable_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.3_2_FollowLeadVehicleEmergencyBrake_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.4_1_CutInNoCollision_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.4_2_CutInUnavoidableCollision_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.5_1_CutOutFullyBlocking_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.5_2_CutOutMultipleBlockingTargets_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.6_1_ForwardDetectionRange_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
#python ScenarioExecutor.py ./scenarios/ALKS/ALKS_Scenario_4.6_2_LateralDetectionRange_TEMPLATE.json ./waypoints/ALKS.Road.waypoint.json autoware
