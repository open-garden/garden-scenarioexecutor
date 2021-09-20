#include "WaypointJsonLoader.hpp"

void setPoints(vector<TPoint> *points, picojson::array points_) {
    for (picojson::value point_: points_) {
        TPoint point;
        point.x = point_.get<picojson::object>()["x"].get<double>();
        point.y = -point_.get<picojson::object>()["z"].get<double>();
        point.z = point_.get<picojson::object>()["y"].get<double>();
        point.v = point_.get<picojson::object>()["v"].get<double>();
        (*points).push_back(point);
    }
}

void setWaypoints(vector<TWaypoint> *waypoints, picojson::array waypoints_) {
    for (picojson::value waypoint_: waypoints_) {
        TWaypoint waypoint;
        waypoint.id = waypoint_.get<picojson::object>()["id"].get<string>();
        setPoints(&waypoint.points, waypoint_.get<picojson::object>()["points"].get<picojson::array>());
        (*waypoints).push_back(waypoint);
    }
}

int LoadWaypointJson(const string &path, TWaypointJson *waypoint) {
    ifstream ifs(path, ios::in);
    if (ifs.fail()) {
        return 0;
    }
    const string json((istreambuf_iterator<char>(ifs)), istreambuf_iterator<char>());
    ifs.close();

    picojson::value v;
    const string err = parse(v, json);
    if (!err.empty()) {
        return 0;
    }

    picojson::object root = v.get<picojson::object>();

    waypoint->id = root["id"].get<string>();

    setWaypoints(&waypoint->waypoints, root["waypoints"].get<picojson::array>());

    cout << path << endl;

    return 1;
}