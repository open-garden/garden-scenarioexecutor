#include <vector>
#include "LaneChangeThread.hpp"

extern TWaypointJson waypointJson;

int getClosestWaypointJson(vector<TPoint> &waypoint, TVector3D current_location) {
    double closest_wp_idx = 0;

    double min_len = sqrt(pow(current_location.X - waypoint[0].x, 2.0) + 
                          pow(current_location.Y - waypoint[0].y, 2.0) +
                          pow(current_location.Z - waypoint[0].z, 2.0));

    for (int i = 1; i < waypoint.size(); i++) {
        double len = sqrt(pow(current_location.X - waypoint[i].x, 2.0) + 
                          pow(current_location.Y - waypoint[i].y, 2.0) +
                          pow(current_location.Z - waypoint[i].z, 2.0));
        if (len <= min_len) {
            min_len = len;
            closest_wp_idx = i;
        }
    }

    return closest_wp_idx;
}

void createLaneChangeWps(TOtherCarInfo *other_car_info, int new_wps_idx, double lane_offset, double lane_change_time) {
    // 1秒進んだところをレーンチェンジの起点にする
    int l = (int)(1.0 * other_car_info->CurrentVelocity);

    if (l < 4) {
        l = 4;
    }

    TVector3D current_location = other_car_info->FineWaypoint[(int)other_car_info->CurrentFineWpIdx].Pose.Location;
    
    int current_wp_idx = getClosestWaypoint(other_car_info->Waypoint, 0, current_location, other_car_info->Waypoint.size()) + l;

    int new_wp_idx = getClosestWaypointJson(waypointJson.waypoints[new_wps_idx].points, current_location) + 1 + l;

    // レーンチェンジ元の最寄りのウェイポイントで切る
    other_car_info->Waypoint.resize(current_wp_idx + 1);

    // レーンチェンジ先のウェイポイントを継ぎ足す
    TVector3D next_wp, wp;
    next_wp.X = waypointJson.waypoints[new_wps_idx].points[new_wp_idx].x;
    next_wp.Y = waypointJson.waypoints[new_wps_idx].points[new_wp_idx].y;
    next_wp.Z = waypointJson.waypoints[new_wps_idx].points[new_wp_idx].z;

    for (int i = new_wp_idx + 1; i < waypointJson.waypoints[new_wps_idx].points.size(); i++) {
        wp = next_wp;
        next_wp.X = waypointJson.waypoints[new_wps_idx].points[i].x;
        next_wp.Y = waypointJson.waypoints[new_wps_idx].points[i].y;
        next_wp.Z = waypointJson.waypoints[new_wps_idx].points[i].z;

        double direction = atan2(next_wp.Y - wp.Y, next_wp.X - wp.X);

        if (lane_offset <= 0.0) {
            direction -= DegToRad(90.0);
        } else {
            direction += DegToRad(90.0);
        }

        wp.X += abs(lane_offset) * cos(direction);
        wp.Y += abs(lane_offset) * sin(direction);

        other_car_info->Waypoint.push_back(wp);
    }

    // レーンチェンジ元とレーンチェンジ先を斜めにつなぐ
    l = (int)(other_car_info->CurrentVelocity * (lane_change_time - 1.0));

    if (l < 4) {
        l = 4;
    }

    int connect_end_wp_idx = current_wp_idx + l;
    if (connect_end_wp_idx >= other_car_info->Waypoint.size()) {
        connect_end_wp_idx = other_car_info->Waypoint.size() - 1;
    }

    // 進行方向
    double hdg = atan2(other_car_info->Waypoint[current_wp_idx].Y - other_car_info->Waypoint[current_wp_idx - 1].Y,
                       other_car_info->Waypoint[current_wp_idx].X - other_car_info->Waypoint[current_wp_idx - 1].X);

    // レーンチェンジ先の方向
    double dir = atan2(other_car_info->Waypoint[current_wp_idx + 1].Y - other_car_info->Waypoint[current_wp_idx].Y,
                       other_car_info->Waypoint[current_wp_idx + 1].X - other_car_info->Waypoint[current_wp_idx].X);

    // 進行方向から見たレーンチェンジ先の方向
    if (sin(dir - hdg) >= 0.0) {
        dir = -1.0;
    } else {
        dir = 1.0;
    }

    double div_t = sqrt(pow(other_car_info->Waypoint[current_wp_idx + 1].X - other_car_info->Waypoint[current_wp_idx].X, 2) + 
                        pow(other_car_info->Waypoint[current_wp_idx + 1].Y - other_car_info->Waypoint[current_wp_idx].Y, 2));
    div_t /= (double)l;

    double div_z = other_car_info->Waypoint[connect_end_wp_idx].Z - other_car_info->Waypoint[current_wp_idx].Z;
    div_z /= (double)l;

    double prev_x = other_car_info->Waypoint[connect_end_wp_idx].X;
    double prev_y = other_car_info->Waypoint[connect_end_wp_idx].Y;

    double t = 0;
    double z = 0;

    for (int i = connect_end_wp_idx - 1; i > current_wp_idx; i--) {
        double a = atan2(prev_y - other_car_info->Waypoint[i].Y, prev_x - other_car_info->Waypoint[i].X);
        a += dir * DegToRad(90.0);
        t += div_t;
        z -= div_z;

        prev_x = other_car_info->Waypoint[i].X;
        prev_y = other_car_info->Waypoint[i].Y;

        other_car_info->Waypoint[i].X += t * cos(a);
        other_car_info->Waypoint[i].Y += t * sin(a);
        other_car_info->Waypoint[i].Z += z;
    }
}

void createLaneChangeFineWps(TOtherCarInfo *other_car_info) {
    createFineWaypoint(other_car_info->Waypoint, &other_car_info->FineWaypoint, other_car_info->CurrentVelocity, other_car_info->Geometory);
}
