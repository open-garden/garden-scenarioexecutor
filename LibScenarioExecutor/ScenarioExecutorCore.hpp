#ifndef ScenarioExecutorCore_hpp
#define ScenarioExecutorCore_hpp

#include <string>
#include <vector>
#include <cmath>
#include "ScenarioJsonLoader.hpp"
#include "WaypointJsonLoader.hpp"

using namespace std;

#define FINE_WAYPOINT_UNIT ((double)0.01)
#define DegToRad(deg) ((double)deg*M_PI/180.0)
#define RadToDeg(rad) ((double)rad*180.0/M_PI)

typedef struct Vector2D {
    double X;
    double Y;
} TVector2D;

typedef struct Vector3D {
    double X;
    double Y;
    double Z;
} TVector3D;

typedef struct Pose {
    TVector3D Location;
    TVector3D Rotation;
} TPose;

typedef struct CarGeo {
    double Length;
    double Width;
    double FrontAxis;
    double WheelBase;
    double WheelDiameter;
} TCarGeo;

typedef struct FineWaypoint {
    TPose Pose;
    double WheelAngle;
} TFineWaypoint;

typedef struct EgoCarInfo {
    int ActorId;
    int BodyColorR;
    int BodyColorG;
    int BodyColorB;
    TCarGeo Geometory;
    TPose Pose;
    TPose PrevPose;
    double CurrentVelocity;
    double PrevVelocity;
    double Accel;
    double Collision;
    double CurrentFineRpIdx;
    double EndFineRpIdx;
    vector<TVector3D> Waypoint;
    vector<TFineWaypoint> FineWaypoint;
    TAction Action;
} TEgoCarInfo;

typedef struct RouteMoveState {
    int RouteIdx;
    double Duration;
} TRouteMoveState;

typedef struct CulcBrake {
    double T;
    double PrevVelocity;
} TCulcBrake;

typedef struct CulcPitch {
    double PrevVelocity;
    double Accel;
    double Pitch;
} TCulcPitch;

// typedef struct CulcRoll {
//     double F;
//     double Roll;
// } TCulcRoll;

typedef struct OtherCarInfo {
    int ActorId;
    string ModelId;
    int BodyColorR;
    int BodyColorG;
    int BodyColorB;
    TPose Pose;
    TPose PrevPose;
    double CurrentVelocity;
    double WorkingVelocity;     // 他車位置計算用の速度(変動が大きいので平均化したものを現在速度とする)
    array<double, 3> WorkingVelocityBuf;
    TCulcBrake CulcBrake;
    TCulcPitch CulcPitch;
    //TCulcRoll CulcRoll;
    TCarGeo Geometory;
    int CurrentWpsIdx;
    double CurrentFineWpIdx;
    double CurrentFineRpIdx;
    vector<TVector3D> Waypoint;
    vector<TFineWaypoint> FineWaypoint;
    double WheelRotation;
    bool BlinkerLeft;
    bool BlinkerRight;
    bool BrakeLamp;
    bool Special1;
    bool Sound;
    TAction Action;
    TRouteMoveState RouteMoveState;
} TOtherCarInfo;

typedef struct PedestrianInfo {
    int ActorId;
    string ModelId;
    TPoint CurrentPoint;
    double Speed;
    double Direction;
    TAction Action;
    TRouteMoveState RouteMoveState;
} TPedestrianInfo;

typedef struct ObstacleInfo {
    int ActorId;
    string ModelId;
    TPoint CurrentPoint;
} TObstacleInfo;

typedef struct CarModel {
    string Id;
    TCarGeo Geo;
} TCarModel;

class SimIniVal {
public:
    string map_id;
    string wp_id;
    string ego_model_id;
    int ego_color_r;
    int ego_color_g;
    int ego_color_b;
    double ego_location_x;
    double ego_location_y;
    double ego_location_z;
    double ego_rotation_x;
    double ego_rotation_y;
    double ego_rotation_z;
    int pedestrian_count;
    int obstacle_count;
};

class PedestrianIniVal {
public:
    string model_id;
    double location_x;
    double location_y;
    double location_z;
    double rotation_z;
};

class PedestrianVal {
public:
    double speed;
    double direction_x;
    double direction_y;
};

class ObstacleIniVal {
public:
    string model_id;
    double location_x;
    double location_y;
    double location_z;
    double rotation_z;
};

int getClosestWaypoint(vector<TVector3D> &waypoint, int current_wp_idx, TVector3D current_location, int search_len);
TFineWaypoint getOtherCarPose(int other_idx);
double calcWheelAngle(TPose current_pose, double front_axis, TVector3D org, double r);
void tick(vector<TVector3D> &waypoint, int *current_wp_idx, TPose *current_pose, double current_velocity, TVector3D *org, double *r);
void createFineWaypoint(vector<TVector3D> &waypoint, vector<TFineWaypoint> *fine_waypoint, double v, TCarGeo geometory);
void _loadScenario(string path);
void _loadWaypoint(string path);
SimIniVal _getSimIniVal();
void _startSimulation();
bool _updateEgoStatus(double location_x, double location_y, double location_z,
                      double rotation_x, double rotation_y, double rotation_z,
                      double velocity_x, double velocity_y, double velocity_z,
                      double collision, double delta_time);
PedestrianIniVal _getPedestrianStartPosition(int idx);
PedestrianVal _updatePedestrianStatus(int idx, double location_x, double location_y, double delta_time);
ObstacleIniVal _getObstaclePosition(int idx);

#endif