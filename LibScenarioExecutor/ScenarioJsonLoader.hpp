#ifndef ScenarioJsonLoader_hpp
#define ScenarioJsonLoader_hpp

#include <string>
#include <vector>

using namespace std;

typedef struct Actor {
    int actor_id;
    string model_id;
    string color;
} TActor;

typedef struct Actors {
    TActor ego;
    vector<TActor> others;
} TActors;

typedef struct Orientation {
    double heading; 
} TOrientation;

typedef struct Position {
    bool exec;
    int actor_id;
    string type;
    string wp_id;
    int wp_idx;
    double wp_offset;
    TOrientation orientation;
    double distance;
    string measure_type;
    double tolerance;
} TPosition;

typedef struct Accel {
    string type;
    double value;
} TAccel;

typedef struct Speed {
    bool exec;
    int actor_id;
    string type;
    int target_actor_id;
    string comparison;
    double value;
    TAccel accel;
} TSpeed;

typedef struct Distance {
    bool exec;
    int actor_id;
    string type;
    int target_actor_id;
    string comparison;
    double value;
    string measure_type;
} TDistance;

typedef struct TraveledDistance {
    bool exec;
    int actor_id;
    string comparison;
    double value;
    string measure_type;
} TTraveledDistance;

typedef struct TimeHeadway {
    bool exec;
    int actor_id;
    string wp_id;
    int wp_idx;
    string comparison;
    double value;
} TTimeHeadway;

typedef struct SimulationTime {
    bool exec;
    string comparison;
    double value;
} TSimulationTime;

typedef struct Collision {
    bool exec;
} TCollision;

typedef struct Exp {
    TPosition position;
    TSpeed speed;
    TDistance distance;
    TTimeHeadway time_headway;
    TSimulationTime simulation_time;
    TCollision collision;
} TExp;

typedef struct Conditions {
    vector<TExp> and_;
    vector<TExp> or_;
} TConditions;

typedef struct LaneChange {
    bool exec;
    int actor_id;
    string type;
    string wp_id;
    double time;
} TLaneChange;

typedef struct LaneOffset {
    bool exec;
    int actor_id;
    string type;
    double value;
    double time;
} TLaneOffset;

typedef struct Route {
    double time;
    string wp_id;
    int wp_idx;
    double wp_offset;
} TRoute;

typedef struct RouteMove {
    bool exec;
    int actor_id;
    string type;
    vector<TRoute> route;
} TRouteMove;

typedef struct LightState {
    bool exec;
    int actor_id;
    int blinker_left;
    int blinker_right;
    int special1;
} TLightState;

typedef struct SoundState {
    bool exec;
    int actor_id;
    int sound;
} TSoundState;

typedef struct HandOver {
    bool exec;
    bool hand_over;
    double throttle;
    double brake;
    double steer;
} THandOver;

typedef struct Action {
    TSpeed speed;
    TTraveledDistance traveled_distance;
    TLaneChange lane_change;
    TLaneOffset lane_offset;
    TRouteMove route_move;
    TLightState light_state;
    TSoundState sound_state;
    THandOver hand_over;
} TAction;

typedef struct OpeningAction {
    int actor_id;
    TPosition start_position;
    TPosition end_position;
    TAction action;
} TOpeningAction;

typedef struct Scene {
    TConditions conditions;
    int scene_id;
    double duration;
    vector<TAction> actions;
    vector<int> next_scenes;
} TScene;

typedef struct OpeningScene {
    TOpeningAction ego;
    vector<TOpeningAction> others;
    vector<int> next_scenes;
} TOpeningScene;

typedef struct Scenario {
   TOpeningScene opening_scene;
   vector<TScene> scenes;
   vector<TScene> ending_scenes; 
} TScenario;

typedef struct ScenarioJson {
    string map_id;
    TActors actors;
    TScenario scenario;
    TConditions stop_conditions;
} TScenarioJson;

int LoadScenarioJson(const string &path, TScenarioJson *scenario);

#endif