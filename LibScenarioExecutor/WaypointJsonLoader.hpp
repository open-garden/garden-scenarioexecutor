#ifndef WaypointJsonLoader_hpp
#define WaypointJsonLoader_hpp

#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include "./picojson/picojson.h"

using namespace std;

typedef struct Point {
    double x;
    double y;
    double z;
    double v;
} TPoint;

typedef struct Waypoint {
    string id;
    vector<TPoint> points;
} TWaypoint;

typedef struct WaypointJson {
    string id;
    vector<TWaypoint> waypoints;
} TWaypointJson;

int LoadWaypointJson(const string &path, TWaypointJson *waypoint);

#endif