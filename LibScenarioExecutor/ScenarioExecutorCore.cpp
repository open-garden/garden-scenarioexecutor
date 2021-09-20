#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include <cfloat>
#include "ScenarioExecutorCore.hpp"
#include "LaneChangeThread.hpp"

TScenarioJson scenario;
TWaypointJson waypointJson;
TEgoCarInfo egoCarInfo;
vector<TOtherCarInfo> otherCarInfo;
vector<TPedestrianInfo> pedestrianInfo;
vector<TObstacleInfo> obstacleInfo;
TScene currentScene;
chrono::system_clock::time_point simStartTime;

vector<TCarModel> carModel = {
    //ModelID                            Length Width FrontAxis WheelBase WheelDiameter
    {"vehicle.audi.a2",                 {4.54,  1.76, 0.92,     2.70,     0.66}},
    {"vehicle.audi.etron",              {4.54,  1.76, 0.92,     2.70,     0.77}},
    {"vehicle.audi.tt",                 {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.bmw.grandtourer",         {4.54,  1.76, 0.92,     1.93,     0.73}},
    {"vehicle.chevrolet.impala",        {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.charger2020.charger2020", {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.garden.gambulancejp",     {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.garden.gpolicecarjp",     {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.garden.gpumperjp",        {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.lincoln2020.mkz2020",     {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.mercedesccc.mercedesccc", {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.mini.cooperst",           {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.nissan.micra",            {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.tesla.model3",            {4.54,  1.76, 0.92,     2.70,     0.76}},
    {"vehicle.toyota.prius",            {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.volkswagen.t2",           {4.54,  1.76, 0.92,     2.70,     0.71}},
    {"vehicle.kawasaki.ninja",          {2.00,  0.80, 0.30,     1.70,     0.61}},
};

double getElapsedSimTime() {
    chrono::system_clock::time_point now = chrono::system_clock::now();
    std::chrono::duration<float> elapsed = now - simStartTime;
    int t = elapsed.count() * 1000.0;
    return (double)t / 1000.0;
}

int getClosestWaypoint(vector<TVector3D> &waypoint, int current_wp_idx, TVector3D current_location, int search_len) {
    double closest_wp_idx = current_wp_idx;

    double min_len = sqrt(pow(current_location.X - waypoint[current_wp_idx].X, 2.0) + 
                          pow(current_location.Y - waypoint[current_wp_idx].Y, 2.0) +
                          pow(current_location.Z - waypoint[current_wp_idx].Z, 2.0));

    for (int i = current_wp_idx + 1; i < waypoint.size(); i++) {
        double len = sqrt(pow(current_location.X - waypoint[i].X, 2.0) + 
                          pow(current_location.Y - waypoint[i].Y, 2.0) +
                          pow(current_location.Z - waypoint[i].Z, 2.0));
        if (len <= min_len) {
            min_len = len;
            closest_wp_idx = i;
        } else if (i > current_wp_idx + search_len) {
            break;
        }
    }

    return closest_wp_idx;
}

double getClosestFineRefPoint(TVector3D location, double rp_idx) {
    for (int i = rp_idx; i < egoCarInfo.FineWaypoint.size(); i++) {
        double car_angle = atan2(location.Y - egoCarInfo.FineWaypoint[i].Pose.Location.Y,
                                 location.X - egoCarInfo.FineWaypoint[i].Pose.Location.X);
        double x = cos(car_angle - egoCarInfo.FineWaypoint[i].Pose.Rotation.Z);

        if (x <= 0.0) {
            rp_idx = (double)i + x;
            break;
        }
    }

    if (rp_idx < 0.0) {
        rp_idx = 0.0;
    }

    return rp_idx;
}

double getClosestFineWpFromRp(int other_idx, double rp_idx) {
    double wp_idx = otherCarInfo[other_idx].CurrentFineWpIdx;

    for (int i = wp_idx; i < otherCarInfo[other_idx].FineWaypoint.size(); i++) {
        double wp_angle = atan2(otherCarInfo[other_idx].FineWaypoint[i].Pose.Location.Y - egoCarInfo.FineWaypoint[(int)rp_idx].Pose.Location.Y,
                                otherCarInfo[other_idx].FineWaypoint[i].Pose.Location.X - egoCarInfo.FineWaypoint[(int)rp_idx].Pose.Location.X);
        double x = cos(wp_angle - egoCarInfo.FineWaypoint[(int)rp_idx].Pose.Rotation.Z);

        if (x >= 0.0) {
            wp_idx = (double)i + modf(rp_idx, &rp_idx);
            break;
        }
    }

    return wp_idx;
}

double getClosestFineRpFromWp(int other_idx, double wp_idx) {
    double rp_idx = otherCarInfo[other_idx].CurrentFineRpIdx;

    for (int i = rp_idx; i < egoCarInfo.FineWaypoint.size(); i++) {
        double rp_angle = atan2(egoCarInfo.FineWaypoint[i].Pose.Location.Y - otherCarInfo[other_idx].FineWaypoint[(int)wp_idx].Pose.Location.Y,
                             egoCarInfo.FineWaypoint[i].Pose.Location.X - otherCarInfo[other_idx].FineWaypoint[(int)wp_idx].Pose.Location.X);
        double x = cos(rp_angle - otherCarInfo[other_idx].FineWaypoint[(int)wp_idx].Pose.Rotation.Z);

        if (x >= 0.0) {
            rp_idx = (double)i + modf(wp_idx, &wp_idx);
            break;
        }
    }

    return rp_idx;
}

TFineWaypoint getOtherCarPose(int other_idx) {
    TCarGeo geo = otherCarInfo[other_idx].Geometory;
    double rear_offset = (geo.FrontAxis + geo.WheelBase) - geo.Length / 2.0;

    // 小数部分を取得
    double i;
    double mod = modf(otherCarInfo[other_idx].CurrentFineWpIdx, &i) * FINE_WAYPOINT_UNIT;

    // 後輪の軸部分のウェイポイントを取得
    double rear_wp_idx = otherCarInfo[other_idx].CurrentFineWpIdx - rear_offset / FINE_WAYPOINT_UNIT;
    if (rear_wp_idx < 0.0) {
        rear_wp_idx = 0.0;
    }
    TFineWaypoint fine_wp = otherCarInfo[other_idx].FineWaypoint[(int)rear_wp_idx];

    // 前輪の軸部分のウェイポイントを取得
    double front_wp_idx = rear_wp_idx + geo.WheelBase / FINE_WAYPOINT_UNIT;
    if (front_wp_idx >= otherCarInfo[other_idx].FineWaypoint.size()) {
        front_wp_idx = otherCarInfo[other_idx].FineWaypoint.size() - 1;
    }
    TFineWaypoint front_wp = otherCarInfo[other_idx].FineWaypoint[(int)front_wp_idx];

    // 前輪と後輪のZ座標の差からpitchを求める
    double dz = front_wp.Pose.Location.Z - fine_wp.Pose.Location.Z;
    fine_wp.Pose.Rotation.Y = atan2(dz, geo.WheelBase);

    // 車の中心の座標を求める
    fine_wp.Pose.Location.X += (rear_offset + mod) * cos(fine_wp.Pose.Rotation.Z);
    fine_wp.Pose.Location.Y += (rear_offset + mod) * sin(fine_wp.Pose.Rotation.Z);
    fine_wp.Pose.Location.Z += dz / 2.0;

    return fine_wp;
}

void calcTrajectry(TPose current_pose, TVector3D target_point, TVector3D *org, double *r) {
    double l = sqrt(pow((target_point.Y - current_pose.Location.Y), 2.0) +
                    pow((target_point.X - current_pose.Location.X), 2.0)) / 2.0;

    double target_angle = atan2(target_point.Y - current_pose.Location.Y, target_point.X - current_pose.Location.X);

    double a = sin(target_angle - current_pose.Rotation.Z);

    if (abs(a) < 0.000000001) {
        *r = 0.0;
    } else {
        *r = l / a;
    }

    double org_angle = current_pose.Rotation.Z - DegToRad(90.0);

    org->X = current_pose.Location.X - (*r) * cos(org_angle);
    org->Y = current_pose.Location.Y - (*r) * sin(org_angle);
}

void tick(vector<TVector3D> &waypoint, int *current_wp_idx, TPose *current_pose, double current_velocity, TVector3D *org, double *r) {
    int len = (int)(1.0 * current_velocity);

    if (len < 4) {
        len = 4;
    }

    *current_wp_idx = getClosestWaypoint(waypoint, *current_wp_idx, current_pose->Location, len);

    int target_wp_idx = *current_wp_idx + len;
    if (target_wp_idx >= waypoint.size()) {
        target_wp_idx = waypoint.size() - 1;
    }

    TVector3D target_wp = waypoint[target_wp_idx];

    calcTrajectry(*current_pose, target_wp, org, r);

    double diameter = 2.0 * (*r) * M_PI;

    if ((*r) == 0.0) {
        current_pose->Location.X += cos(current_pose->Rotation.Z) * FINE_WAYPOINT_UNIT;
        current_pose->Location.Y += sin(current_pose->Rotation.Z) * FINE_WAYPOINT_UNIT;
    } else {
        double angle = (DegToRad(360.0) / diameter) * FINE_WAYPOINT_UNIT;

        if ((*r) > 0.0) {
            angle = atan2(current_pose->Location.Y - org->Y, current_pose->Location.X - org->X) + angle;
        } else {
            angle = atan2(org->Y - current_pose->Location.Y, org->X - current_pose->Location.X) + angle;
        }

        double car_x = (*r) * cos(angle);
        double car_y = (*r) * sin(angle);

        current_pose->Location.X = org->X + car_x;
        current_pose->Location.Y = org->Y + car_y;
        current_pose->Rotation.Z = angle + DegToRad(90.0);
    }

    // 高さを求める
    TVector3D current_wp = waypoint[*current_wp_idx];
    TVector3D next_wp = waypoint[*current_wp_idx + 1];
    double a1 = atan2(next_wp.Y - current_wp.Y, next_wp.X - current_wp.X);
    double a2 = atan2(current_pose->Location.Y - current_wp.Y, current_pose->Location.X - current_wp.X);
    double a3 = a2 - a1;
    double b = sqrt(pow(current_pose->Location.X - current_wp.X, 2.0) + pow(current_pose->Location.Y - current_wp.Y, 2.0));

    double l = cos(a3) * b;
    double L = sqrt(pow(next_wp.X - current_wp.X, 2.0) + pow(next_wp.Y - current_wp.Y, 2.0));
    double H = next_wp.Z - current_wp.Z;
    double h = l / L * H;

    current_pose->Location.Z = current_wp.Z + h;
}

int getWpsIdx(string wp_id) {
    for (int wps_idx = 0; wps_idx < waypointJson.waypoints.size(); wps_idx++) {
        if (waypointJson.waypoints[wps_idx].id == wp_id) {
            return wps_idx;
        }
    }
    return 0;
}

int getCarModelIdx(string model_id) {
    for (int i = 0; i < carModel.size(); i++) {
        if (model_id == carModel[i].Id) {
            return i;
        }
    }
    return -1;
}

double normalizeAngle(double angle) {
    double new_angle = RadToDeg(angle);
    while (new_angle <= -180.0) new_angle = new_angle + 360.0;
    while (new_angle > 180.0) new_angle = new_angle - 360.0;
    return DegToRad(new_angle);
}

double calcWheelAngle(TPose current_pose, double front_axis, TVector3D org, double r) {
    if (r != 0.0) {
        double front_x = current_pose.Location.X + front_axis * cos(current_pose.Rotation.Z);
        double front_y = current_pose.Location.Y + front_axis * sin(current_pose.Rotation.Z);

        double angle = atan2(front_y - org.Y, front_x - org.X);

        if (r < 0.0) {
            angle = angle - DegToRad(90.0);
        } else {
            angle = angle + DegToRad(90.0);
        }

        return normalizeAngle(current_pose.Rotation.Z - angle);
    }

    return 0;
}

bool isPedestrianModelIdx(string model_id) {
    if (model_id == "walker.pedestrian.0001") {
        return true;
    } else {
        return false;
    }
}

TPose getWpPose(vector<TVector3D> &waypoint, int wp_idx) {
    TPose pose;
    pose.Location = waypoint[wp_idx];

    int next_wp_idx;
    if (wp_idx < waypoint.size() - 1) {
        next_wp_idx = wp_idx + 1;
    } else {
        next_wp_idx = wp_idx;
        wp_idx = wp_idx - 1;
    }

    pose.Rotation.X = 0.0;
    pose.Rotation.Y = 0.0;
    pose.Rotation.Z = atan2(waypoint[next_wp_idx].Y - waypoint[wp_idx].Y, waypoint[next_wp_idx].X - waypoint[wp_idx].X);

    return pose;
}

int getOtherCarInfoIdx(int actor_id) {
    int info_idx = -1; 
    for (int i = 0; i < otherCarInfo.size(); i++) {
        if (actor_id == otherCarInfo[i].ActorId) {
            info_idx = i;
            break;
        }
    }
    return info_idx;
}

int getPedestrianInfoIdx(int actor_id) {
    int info_idx = -1; 
    for (int i = 0; i < pedestrianInfo.size(); i++) {
        if (actor_id == pedestrianInfo[i].ActorId) {
            info_idx = i;
            break;
        }
    }
    return info_idx;
}

int getObstacleInfoIdx(int actor_id) {
    int info_idx = -1; 
    for (int i = 0; i < obstacleInfo.size(); i++) {
        if (actor_id == obstacleInfo[i].ActorId) {
            info_idx = i;
            break;
        }
    }
    return info_idx;
}

void getEgoStartPosition() {
    int wp_idx = scenario.scenario.opening_scene.ego.start_position.wp_idx;

    TPose pose = getWpPose(egoCarInfo.Waypoint, wp_idx);

    egoCarInfo.CurrentFineRpIdx = getClosestFineRefPoint(pose.Location, egoCarInfo.CurrentFineRpIdx);

    egoCarInfo.PrevPose = egoCarInfo.Pose = pose;
}

void getOtherStartPosition() {
    for (TOpeningAction opening_action: scenario.scenario.opening_scene.others) {
        int info_idx = getOtherCarInfoIdx(opening_action.actor_id);
        if (info_idx < 0) {
            continue;
        }
        if (opening_action.start_position.type == "distance") {
            double distance = scenario.scenario.opening_scene.others[info_idx].start_position.distance;

            if (opening_action.start_position.measure_type == "surface") {
                if (distance >= 0.0) {
                    distance += egoCarInfo.Geometory.Length / 2.0 + otherCarInfo[info_idx].Geometory.Length / 2.0;
                } else {
                    distance -= egoCarInfo.Geometory.Length / 2.0 + otherCarInfo[info_idx].Geometory.Length / 2.0;
                }
            }

            int rp_idx = (int)(egoCarInfo.CurrentFineRpIdx + distance / FINE_WAYPOINT_UNIT);

            otherCarInfo[info_idx].CurrentFineRpIdx = rp_idx;
            otherCarInfo[info_idx].CurrentFineWpIdx = getClosestFineWpFromRp(info_idx, rp_idx);
        } else {
            int wp_idx = opening_action.start_position.wp_idx;

            TPose other_pose = getWpPose(otherCarInfo[info_idx].Waypoint, wp_idx);

            otherCarInfo[info_idx].CurrentFineRpIdx = getClosestFineRefPoint(other_pose.Location, 0);
            otherCarInfo[info_idx].CurrentFineWpIdx = (double)wp_idx / FINE_WAYPOINT_UNIT;
        }

        otherCarInfo[info_idx].PrevPose = otherCarInfo[info_idx].Pose = getOtherCarPose(info_idx).Pose;
    }
}

TScene getScene(int scene_id) {
    TScene scene;
    for (TScene scene: scenario.scenario.scenes) {
        if (scene_id == scene.scene_id) {
            return scene;
        }
    }

    for (TScene scene: scenario.scenario.ending_scenes) {
        if (scene_id == scene.scene_id) {
            return scene;
        }
    }
    return scene;
}

bool checkCondition(TExp exp) {
    if (exp.speed.exec) {
        double current_veloicty;
        if (exp.speed.actor_id == egoCarInfo.ActorId) {
            current_veloicty = egoCarInfo.CurrentVelocity;
        } else {
            int other_idx = getOtherCarInfoIdx(exp.speed.actor_id);
            current_veloicty = otherCarInfo[other_idx].CurrentVelocity;
        }
        double velocity = exp.speed.value / 3.6;
        if (exp.speed.comparison == ">=") {
            if (current_veloicty >= velocity) {
                cout << getElapsedSimTime() << " [condition] speed " << current_veloicty * 3.6 << " >= " << velocity * 3.6 << endl;
                return true;
            }
        }
    } else if (exp.distance.exec) {
        double actor_rp_idx, target_actor_rp_idx;
        double actor_length, target_actor_length;

        if (exp.distance.actor_id == egoCarInfo.ActorId) {
            actor_rp_idx = egoCarInfo.CurrentFineRpIdx;
            actor_length = egoCarInfo.Geometory.Length;
        } else {
            int other_idx = getOtherCarInfoIdx(exp.distance.actor_id);
            if (other_idx >= 0) {
                actor_rp_idx = otherCarInfo[other_idx].CurrentFineRpIdx;
                actor_length = otherCarInfo[other_idx].Geometory.Length;
            } else {
                int pedestrian_idx = getPedestrianInfoIdx(exp.distance.actor_id);
                TVector3D wp;
                wp.X = pedestrianInfo[pedestrian_idx].CurrentPoint.x;
                wp.Y = pedestrianInfo[pedestrian_idx].CurrentPoint.y;
                wp.Z = 0.0;
                actor_rp_idx = getClosestFineRefPoint(wp, egoCarInfo.CurrentFineRpIdx);
                actor_length = 0.0;
            }
        }

        if (exp.distance.target_actor_id == egoCarInfo.ActorId) {
            target_actor_rp_idx = (int)egoCarInfo.CurrentFineRpIdx;
            target_actor_length = egoCarInfo.Geometory.Length;
        } else {
            int other_idx = getOtherCarInfoIdx(exp.distance.target_actor_id);
            if (other_idx >= 0) {
                target_actor_rp_idx = (int)otherCarInfo[other_idx].CurrentFineRpIdx;
                target_actor_length = otherCarInfo[other_idx].Geometory.Length;
            } else {
                int pedestrian_idx = getPedestrianInfoIdx(exp.distance.target_actor_id);
                TVector3D wp;
                wp.X = pedestrianInfo[pedestrian_idx].CurrentPoint.x;
                wp.Y = pedestrianInfo[pedestrian_idx].CurrentPoint.y;
                wp.Z = 0.0;
                target_actor_rp_idx = getClosestFineRefPoint(wp, egoCarInfo.CurrentFineRpIdx);
                target_actor_length = 0.0;
            }
        }

        double distance = (target_actor_rp_idx - actor_rp_idx) * FINE_WAYPOINT_UNIT;

        if (exp.distance.measure_type == "surface") {
            if (exp.distance.value >= 0.0) {
                distance -= actor_length / 2.0 + target_actor_length / 2.0;
            } else {
                distance += actor_length / 2.0 + target_actor_length / 2.0;
             }
        }

        if (exp.distance.value >= 0.0 && distance >= 0.0) {
            if (exp.distance.comparison == "<=") {
                if (distance <= exp.distance.value) {
                    cout << getElapsedSimTime() << " [condition] distance " << exp.distance.measure_type << " " << distance << " <= " << exp.distance.value << endl;
                    return true;
                }
            } else if (exp.distance.comparison == "<") {
                if (distance < exp.distance.value) {
                    cout << getElapsedSimTime() << " [condition] distance " << exp.distance.measure_type << " " << distance << " < " << exp.distance.value << endl;
                    return true;
                }
            } else if (exp.distance.comparison == ">=") {
                if (distance >= exp.distance.value) {
                    cout << getElapsedSimTime() << " [condition] distance " << exp.distance.measure_type << " " << distance << " >= " << exp.distance.value << endl;
                    return true;
                }
            } else if (exp.distance.comparison == ">") {
                if (distance > exp.distance.value) {
                    cout << getElapsedSimTime() << " [condition] distance " << exp.distance.measure_type << " " << distance << " > " << exp.distance.value << endl;
                    return true;
                }
            }
        } else if (exp.distance.value < 0.0 && distance <= 0.0) {
            distance = abs(distance);
            double value = abs(exp.distance.value);
            if (exp.distance.comparison == "<=") {
                if (distance <= value) {
                    cout << getElapsedSimTime() << " [condition] distance " << exp.distance.measure_type << " " << distance << " <= " << value << endl;
                    return true;
                }
            } else if (exp.distance.comparison == "<") {
                if (distance < value) {
                    cout << getElapsedSimTime() << " [condition] distance " << exp.distance.measure_type << " " << distance << " < " << value << endl;
                    return true;
                }
            } else if (exp.distance.comparison == ">=") {
                if (distance >= value) {
                    cout << getElapsedSimTime() << " [condition] distance " << exp.distance.measure_type << " " << distance << " >= " << value << endl;
                    return true;
                }
            } else if (exp.distance.comparison == ">") {
                if (distance > value) {
                    cout << getElapsedSimTime() << " [condition] distance " << exp.distance.measure_type << " " << distance << " > " << value << endl;
                    return true;
                }
            }
        }
    } else if (exp.position.exec) {
        double distance;
        Vector3D location, prev_location, wp;

        if (exp.position.actor_id == egoCarInfo.ActorId) {
            location = egoCarInfo.Pose.Location;
            prev_location = egoCarInfo.PrevPose.Location;
            wp = egoCarInfo.Waypoint[exp.position.wp_idx];
        } else {
            int other_idx = getOtherCarInfoIdx(exp.position.actor_id);
            location = otherCarInfo[other_idx].Pose.Location;
            prev_location = otherCarInfo[other_idx].PrevPose.Location;
            wp = otherCarInfo[other_idx].Waypoint[exp.position.wp_idx];
        }

        double d1 = sqrt(pow(wp.X - location.X, 2.0) + pow(wp.Y - location.Y, 2.0));
        double d2 = DBL_MAX;

        // 高速で走行している場合、目標位置を飛び越えている場合を考慮する(前回位置と現在位置を結ぶ線分が目標の半径内を通過しているかを調べる)
        if (prev_location.X != location.X || prev_location.Y != location.Y) {
            double a1, a2;
            a1 = atan2(location.Y - prev_location.Y, location.X - prev_location.X);
            a2 = atan2(wp.Y - prev_location.Y, wp.X - prev_location.X);
            double x1 = cos(a2 - a1);

            a1 = atan2(prev_location.Y - location.Y, prev_location.X - location.X);
            a2 = atan2(wp.Y - location.Y, wp.X - location.X);
            double x2 = cos(a2 - a1);

            if (x1 >= 0 && x2 >= 0) {
                d2 = abs(sin(a2 - a1) * d1);
            }
        }

        distance = d1 < d2 ? d1 : d2;

        distance = sqrt(pow(distance, 2.0) + pow(wp.Z - location.Z, 2.0));

        if (distance <= exp.position.tolerance) {
            cout << getElapsedSimTime() << " [condition] position reach " << "waypoint " << exp.position.wp_idx << endl;
            return true;
        }
    } else if (exp.time_headway.exec) {
        if (exp.time_headway.actor_id == egoCarInfo.ActorId) {
            int rp_idx1 = (int)(egoCarInfo.CurrentFineRpIdx + (egoCarInfo.Geometory.Length / 2.0) / FINE_WAYPOINT_UNIT);

            int wps_idx = getWpsIdx(exp.time_headway.wp_id);
            TVector3D wp;
            wp.X = waypointJson.waypoints[wps_idx].points[exp.time_headway.wp_idx].x;
            wp.Y = waypointJson.waypoints[wps_idx].points[exp.time_headway.wp_idx].y;
            wp.Z = 0.0;
            int rp_idx2 = getClosestFineRefPoint(wp, egoCarInfo.CurrentFineRpIdx);
            
            double d = (rp_idx2 - rp_idx1) * FINE_WAYPOINT_UNIT;

            if (egoCarInfo.CurrentVelocity > 0.0) {
                double t = d / egoCarInfo.CurrentVelocity;

                if (exp.time_headway.comparison == "<=") {
                    if (t <= exp.time_headway.value) {
                        cout << getElapsedSimTime() << " [condition] time_headway " << t << "<=" << exp.time_headway.value << endl;
                        return true;
                    }
                }
            }
        }
    } else if (exp.simulation_time.exec) {
        if (getElapsedSimTime() >= exp.simulation_time.value) {
            cout << getElapsedSimTime() << " [condition] simulation_time " << exp.simulation_time.value << endl;
            return true;
        }
    } else if (exp.collision.exec) {
        if (egoCarInfo.Collision) {
            cout << getElapsedSimTime() << " [condition] collision " << endl;
            return true;
        }
    }

    return false;
}

bool checkStopConditions() {
    if (scenario.stop_conditions.and_.size() > 0) {
        int true_count = 0;
        for (TExp exp: scenario.stop_conditions.and_) {
            if (checkCondition(exp)) {
                true_count++;
            }
        }

        if (true_count == scenario.stop_conditions.and_.size()) {
            return true;
        }
    } else if (scenario.stop_conditions.or_.size() > 0) {
        for (TExp exp: scenario.stop_conditions.or_) {
            if (checkCondition(exp)) {
                return true;
            }
        }
    }

    return false;
}

void getNextSceneId(vector<int> &next_scenes) {
    TScene scene;
    bool scene_change = false;

    for (int next_scene_id: next_scenes) {
        scene = getScene(next_scene_id);

        if (scene.conditions.and_.size() > 0) {
            int true_count = 0;
            for (TExp exp: scene.conditions.and_) {
                if (checkCondition(exp)) {
                    true_count++;
                }
            }

            if (true_count == scene.conditions.and_.size()) {
                scene_change = true;
                break;
            }
        } else if (scene.conditions.or_.size() > 0) {
            for (TExp exp: scene.conditions.or_) {
                if (checkCondition(exp)) {
                    scene_change = true;
                    break;
                }
            }
        } else {
            scene_change = true;
            break;
        }
    }

    if (scene_change) {
        cout << getElapsedSimTime() << " [scene] id " << currentScene.scene_id << " -> " << scene.scene_id << " duration " << scene.duration << endl;
        for (TAction action: scene.actions) {
            if (action.traveled_distance.exec) {
                int other_idx = getOtherCarInfoIdx(action.traveled_distance.actor_id);
                otherCarInfo[other_idx].Action.traveled_distance = action.traveled_distance;
                otherCarInfo[other_idx].Action.speed.exec = false;
                cout << getElapsedSimTime() << " [action] traveled_distance " << action.traveled_distance.measure_type << " " << action.traveled_distance.value << endl;
            } else if (action.speed.exec) {
                int other_idx = getOtherCarInfoIdx(action.speed.actor_id);
                otherCarInfo[other_idx].Action.speed = action.speed;
                otherCarInfo[other_idx].Action.traveled_distance.exec = false;
                cout << getElapsedSimTime() << " [action] speed " << action.speed.type << " " << action.speed.value << endl;
            } else if (action.lane_change.exec) {
                int other_idx = getOtherCarInfoIdx(action.lane_change.actor_id);
                otherCarInfo[other_idx].Action.lane_change = action.lane_change;
                cout << getElapsedSimTime() << " [action] lane_change " << action.lane_change.wp_id << endl;
            } else if (action.lane_offset.exec) {
                int other_idx = getOtherCarInfoIdx(action.lane_offset.actor_id);
                otherCarInfo[other_idx].Action.lane_offset = action.lane_offset;
                cout << getElapsedSimTime() << " [action] lane_offset " << action.lane_offset.value << endl;
            } else if (action.route_move.exec) {
                int info_idx = getOtherCarInfoIdx(action.route_move.actor_id);
                if (info_idx >= 0) {
                    otherCarInfo[info_idx].Action.route_move = action.route_move;
                    otherCarInfo[info_idx].RouteMoveState.RouteIdx = 0;
                    otherCarInfo[info_idx].RouteMoveState.Duration = 0.0;
                } else {
                    info_idx = getPedestrianInfoIdx(action.route_move.actor_id);
                    if (info_idx >= 0) {
                        pedestrianInfo[info_idx].Action.route_move = action.route_move;
                        pedestrianInfo[info_idx].RouteMoveState.RouteIdx = 0;
                        pedestrianInfo[info_idx].RouteMoveState.Duration = 0.0;
                    }
                }
                cout << getElapsedSimTime() << " [action] route_move" << endl;
            } else if (action.light_state.exec) {
                int other_idx = getOtherCarInfoIdx(action.light_state.actor_id);
                otherCarInfo[other_idx].Action.light_state = action.light_state;
                cout << getElapsedSimTime() << " [action] light_state";
                if (action.light_state.blinker_left == 1)
                    cout << " blinker_left on";
                else if (action.light_state.blinker_left == 0)
                    cout << " blinker_left off";
                if (action.light_state.blinker_right == 1)
                    cout << " blinker_right on";
                else if (action.light_state.blinker_right == 0)
                    cout << " blinker_right off";
                if (action.light_state.special1 == 1)
                    cout << " special1 on";
                else if (action.light_state.special1 == 0)
                    cout << " special1 off";
                cout << endl;
            } else if (action.sound_state.exec) {
                int other_idx = getOtherCarInfoIdx(action.sound_state.actor_id);
                otherCarInfo[other_idx].Action.sound_state = action.sound_state;
                cout << getElapsedSimTime() << " [action] sound_state";
                if (action.sound_state.sound == 1)
                    cout << " on";
                else if (action.sound_state.sound == 0)
                    cout << " off";
                cout << endl;
            } else if (action.hand_over.exec) {
                egoCarInfo.Action.hand_over = action.hand_over;
                egoCarInfo.Action.hand_over.exec = false;
                cout << getElapsedSimTime() << " [action] hand_over";
                if (action.hand_over.hand_over) {
                    cout << " on";
                    cout << " throttle " << action.hand_over.throttle;
                    cout << " brake " << action.hand_over.brake;
                    cout << " steer " << action.hand_over.steer;
                } else {
                    cout << " off";
                }
                cout << endl;
            }
        }
        currentScene = scene;
    }
}

void getOtherCarPoseByDistance(int other_idx, double target_distance, double delta_time) {
    double current_distance = (otherCarInfo[other_idx].CurrentFineRpIdx - egoCarInfo.CurrentFineRpIdx) * FINE_WAYPOINT_UNIT;
    double s = target_distance - current_distance;
    double rel_v = egoCarInfo.CurrentVelocity - otherCarInfo[other_idx].WorkingVelocity;
    double a = 1.0 * delta_time;
    double ego_a = abs(egoCarInfo.Accel * delta_time);

    // 通常は加速度1m/s^2とするが、自車がそれを超えていた場合、自車に合わせる
    if (ego_a > a) {
        a = ego_a;
    }

    // 初速度0、加速度aで運動している物体がs進んだときの速度vを求める
    double v = sqrt(2 * a * abs(s));

    if (s > 0.0) {
        v = -v;
    }

    // 目標の車間距離sまで加速して詰める場合、今この瞬間からs区間減速した未来の速度vが自車との相対速度rel_vを下回るなら加速(目標の車間距離まで減速して詰める場合はその逆)
    if (v < rel_v) {
        otherCarInfo[other_idx].WorkingVelocity += a;
    } else if (v > rel_v) {
        otherCarInfo[other_idx].WorkingVelocity -= a;
    }

    if (otherCarInfo[other_idx].WorkingVelocity < 0) {
        otherCarInfo[other_idx].WorkingVelocity = 0;
    }

    double d = (otherCarInfo[other_idx].WorkingVelocity * delta_time) / FINE_WAYPOINT_UNIT;

    double current_fine_wp_idx = otherCarInfo[other_idx].CurrentFineWpIdx + d;
    if ((int)current_fine_wp_idx >= otherCarInfo[other_idx].FineWaypoint.size()) {
        current_fine_wp_idx = otherCarInfo[other_idx].FineWaypoint.size() - 1;
    }
    otherCarInfo[other_idx].CurrentFineWpIdx = current_fine_wp_idx;
    otherCarInfo[other_idx].CurrentFineRpIdx = getClosestFineRpFromWp(other_idx, current_fine_wp_idx);
}

void getOtherCarPoseByVelocity(int other_idx, double target_velocity, double accel, bool god_hands, double delta_time) {
    if (god_hands) {
        otherCarInfo[other_idx].WorkingVelocity = target_velocity;
    } else {
        if (target_velocity > otherCarInfo[other_idx].WorkingVelocity) {
            otherCarInfo[other_idx].WorkingVelocity += accel * delta_time;

            if (otherCarInfo[other_idx].WorkingVelocity > target_velocity) {
                otherCarInfo[other_idx].WorkingVelocity = target_velocity;
            }
        } else if (target_velocity < otherCarInfo[other_idx].WorkingVelocity) {
            otherCarInfo[other_idx].WorkingVelocity -= accel * delta_time;

            if (otherCarInfo[other_idx].WorkingVelocity < target_velocity) {
                otherCarInfo[other_idx].WorkingVelocity = target_velocity;
            }
        }
    }

    double d = (otherCarInfo[other_idx].WorkingVelocity * delta_time) / FINE_WAYPOINT_UNIT;

    double current_fine_wp_idx = otherCarInfo[other_idx].CurrentFineWpIdx + d;
    if ((int)current_fine_wp_idx >= otherCarInfo[other_idx].FineWaypoint.size()) {
        current_fine_wp_idx = otherCarInfo[other_idx].FineWaypoint.size() - 1;
    }
    otherCarInfo[other_idx].CurrentFineWpIdx = current_fine_wp_idx;
    otherCarInfo[other_idx].CurrentFineRpIdx = getClosestFineRpFromWp(other_idx, current_fine_wp_idx);
}

void createFineWaypoint(vector<TVector3D> &waypoint_, vector<TFineWaypoint> *fine_waypoint, double v, TCarGeo geometory) {
    // ウェイポイントを1m間隔にする
    vector<TVector3D> waypoint;
    TVector3D point, next_point;
    point = waypoint_[0];
    for (int i = 1; i < waypoint_.size(); i++) {
        next_point = waypoint_[i];
        int l = (int)sqrt(pow(next_point.X - point.X, 2) + pow(next_point.Y - point.Y, 2));
        if (l > 0) {
            double angle = atan2(next_point.Y - point.Y, next_point.X - point.X);
            double angle_z = atan2(next_point.Z - point.Z, l);
            while (l > 0) {
                waypoint.push_back(point);
                // 次のウェイポイントの方向に1m進める
                point.X += cos(angle);
                point.Y += sin(angle);
                point.Z += sin(angle_z);
                l--;
            }
        }
    }

    // z座標を均す
    std::array<double, 8> avg_z;
    for (int i = 0; i < avg_z.size() && i < waypoint.size(); i++) {
        avg_z[i] = waypoint[i].Z;
    }

    for (int i = 0; i < waypoint.size(); i++) {
        double z = 0;
        for (int j = 0; j < avg_z.size(); j++) {
            z += avg_z[j];
        }
        waypoint[i].Z = z / avg_z.size();
        int j = 0;
        while (j + 1 < avg_z.size()) {
            avg_z[j] = avg_z[j + 1];
            j++;
        }
        if (i + avg_z.size() < waypoint.size()) {
            avg_z[j] = waypoint[i + avg_z.size()].Z;
        } else {
            avg_z[j] = waypoint[waypoint.size() - 1].Z;
        }
    }

    TPose pose;
    TVector3D org;
    double r;
    TFineWaypoint fine_wp;

    pose.Location = waypoint[0];
    pose.Rotation.X = 0;
    pose.Rotation.Y = 0;
    pose.Rotation.Z = atan2(waypoint[1].Y - waypoint[0].Y, waypoint[1].X - waypoint[0].X);

    int current_wp_idx = 0;
    int fine_wp_count = 0;

    while (current_wp_idx + 2 < waypoint.size() - 2) {
        tick(waypoint, &current_wp_idx, &pose, v, &org, &r);

        fine_wp.Pose = pose;
        fine_wp.WheelAngle = calcWheelAngle(pose, geometory.FrontAxis, org, r);

        if (fine_wp_count < fine_waypoint->size()) {
            (*fine_waypoint)[fine_wp_count] = fine_wp;
        } else {
            fine_waypoint->push_back(fine_wp);
        }
        fine_wp_count++;
    }
    fine_waypoint->resize(fine_wp_count);
}

void traveledDistanceAction(int other_idx, TTraveledDistance traveled_distance, double delta_time) {
    double d = traveled_distance.value;

    if (traveled_distance.measure_type == "surface") {
        if (d >= 0.0) {
            d += egoCarInfo.Geometory.Length / 2.0 + otherCarInfo[other_idx].Geometory.Length / 2.0;
        } else {
            d -= egoCarInfo.Geometory.Length / 2.0 + otherCarInfo[other_idx].Geometory.Length / 2.0;
        }
    }
    getOtherCarPoseByDistance(other_idx, d, delta_time);
}

void speedAction(int other_idx, TSpeed speed, double delta_time) {
    double v;

    if (speed.type == "relative") {
        v = egoCarInfo.CurrentVelocity + speed.value / 3.6;
    } else {
        v = speed.value / 3.6;
    }

    double a = speed.accel.value;

    bool god_hands;
    if (speed.accel.type == "god_hands") {
        god_hands = true;
    } else {
        god_hands = false;
    }

    getOtherCarPoseByVelocity(other_idx, v, a, god_hands, delta_time);
}

void laneChageAction(int other_idx, TLaneChange lane_change) {
    int new_wps_idx = getWpsIdx(lane_change.wp_id);

    createLaneChangeWps(&otherCarInfo[other_idx], new_wps_idx, 0, lane_change.time);

    thread lane_change_thread(createLaneChangeFineWps, &otherCarInfo[other_idx]);
    lane_change_thread.detach();

    otherCarInfo[other_idx].CurrentWpsIdx = new_wps_idx;
}

void laneOffsetAction(int other_idx, TLaneOffset lane_offset) {
    createLaneChangeWps(&otherCarInfo[other_idx], otherCarInfo[other_idx].CurrentWpsIdx, lane_offset.value, lane_offset.time);

    thread lane_change_thread(createLaneChangeFineWps, &otherCarInfo[other_idx]);
    lane_change_thread.detach();
}

void otherCarRouteMoveAction(int other_idx, TRouteMove route_move, double delta_time) {
    if (otherCarInfo[other_idx].RouteMoveState.Duration > 0.0) {
        otherCarInfo[other_idx].RouteMoveState.Duration -= delta_time;
        if (otherCarInfo[other_idx].RouteMoveState.Duration > 0.0) {
            return;
        }
        otherCarInfo[other_idx].RouteMoveState.RouteIdx++;
        if (otherCarInfo[other_idx].RouteMoveState.RouteIdx == route_move.route.size()) {
            otherCarInfo[other_idx].Action.route_move.exec = false;
            return;
        }
    }

    int route_idx = otherCarInfo[other_idx].RouteMoveState.RouteIdx;

    cout << getElapsedSimTime() << " [root move action] wp_offset=" << route_move.route[route_idx].wp_offset << endl;

    createLaneChangeWps(&otherCarInfo[other_idx], otherCarInfo[other_idx].CurrentWpsIdx,
                        route_move.route[route_idx].wp_offset, route_move.route[route_idx].time);

    thread lane_change_thread(createLaneChangeFineWps, &otherCarInfo[other_idx]);
    lane_change_thread.detach();

    otherCarInfo[other_idx].RouteMoveState.Duration = route_move.route[route_idx].time;
}

void lightStateAction(int other_idx, TLightState light_state) {
    if (light_state.blinker_left != -1) {
        otherCarInfo[other_idx].BlinkerLeft = (bool)light_state.blinker_left;
    }
    if (light_state.blinker_right != -1) {
        otherCarInfo[other_idx].BlinkerRight = (bool)light_state.blinker_right;
    }
    if (light_state.special1 != -1) {
        otherCarInfo[other_idx].Special1 = (bool)light_state.special1;
    }
}

void soundStateAction(int other_idx, TSoundState sound_state) {
    if (sound_state.sound != -1) {
        otherCarInfo[other_idx].Sound = (bool)sound_state.sound;
    }
}

void pedestrianRouteMoveAction(TRouteMove route_move, int pedestrian_idx, double delta_time) {
    TRouteMoveState *route_move_state = &pedestrianInfo[pedestrian_idx].RouteMoveState;
    if (route_move_state->Duration > 0.0) {
        route_move_state->Duration -= delta_time;
        if (route_move_state->Duration > 0.0) {
            return;
        }
        route_move_state->RouteIdx++;
        if (route_move_state->RouteIdx == route_move.route.size()) {
            pedestrianInfo[pedestrian_idx].Speed = 0.0;
            pedestrianInfo[pedestrian_idx].Action.route_move.exec = false;
            return;
        }
    }

    int route_idx = route_move_state->RouteIdx;

    cout << getElapsedSimTime() << " [root move action] wp_idx=" << route_move.route[route_idx].wp_idx << endl;

    int wps_idx = getWpsIdx(route_move.route[route_idx].wp_id);

    int wp_idx1 = route_move.route[route_idx].wp_idx;
    if (wp_idx1 >= waypointJson.waypoints[wps_idx].points.size() - 1) {
        wp_idx1 = waypointJson.waypoints[wps_idx].points.size() - 2;
    }
    int wp_idx2 = wp_idx1 + 1;

    TPoint p1 = waypointJson.waypoints[wps_idx].points[wp_idx1];
    TPoint p2 = waypointJson.waypoints[wps_idx].points[wp_idx2];

    double direction = atan2(p2.y - p1.y, p2.x - p2.x);

    if (route_move.route[route_idx].wp_offset <= 0.0) {
        direction -= DegToRad(90.0);
    } else {
        direction += DegToRad(90.0);
    }

    TPoint target = waypointJson.waypoints[wps_idx].points[route_move.route[route_idx].wp_idx];

    target.x += abs(route_move.route[route_idx].wp_offset) * cos(direction);
    target.y += abs(route_move.route[route_idx].wp_offset) * sin(direction);

    double dx = target.x - pedestrianInfo[pedestrian_idx].CurrentPoint.x;
    double dy = target.y - pedestrianInfo[pedestrian_idx].CurrentPoint.y;
    double l = sqrt(pow(dx, 2) + pow(dy, 2));
    pedestrianInfo[pedestrian_idx].Speed = l / route_move.route[route_idx].time;
    pedestrianInfo[pedestrian_idx].Direction = atan2(dy, dx);

    pedestrianInfo[pedestrian_idx].RouteMoveState.Duration = route_move.route[route_idx].time;
}

vector<TAction> getActions() {
    vector<TAction> actions;
    if (currentScene.scene_id == 0) {
        for (TOpeningAction opening_action: scenario.scenario.opening_scene.others) {
            actions.push_back(opening_action.action);
        }
    } else {
        TScene scene = getScene(currentScene.scene_id);
        actions = scene.actions;
    }
    return actions;
}

void _loadScenario(string path) {
    LoadScenarioJson(path, &scenario);

    currentScene.scene_id = 0;
    currentScene.next_scenes = scenario.scenario.opening_scene.next_scenes;

    egoCarInfo.ActorId = scenario.actors.ego.actor_id;
    int model_idx = getCarModelIdx(scenario.actors.ego.model_id);
    egoCarInfo.Geometory = carModel[model_idx].Geo;
    string r = scenario.actors.ego.color.substr(0, 2);
    string g = scenario.actors.ego.color.substr(2, 2);
    string b = scenario.actors.ego.color.substr(4, 2);
    egoCarInfo.BodyColorR = stoi(r, nullptr, 16);
    egoCarInfo.BodyColorG = stoi(g, nullptr, 16);
    egoCarInfo.BodyColorB = stoi(b, nullptr, 16);
    egoCarInfo.CurrentVelocity = 0.0;
    egoCarInfo.Action.hand_over.hand_over = false;

    for (int other_idx = 0; other_idx < scenario.scenario.opening_scene.others.size(); other_idx++) {
        int model_idx = getCarModelIdx(scenario.actors.others[other_idx].model_id);
        if (model_idx >= 0) {
            TOtherCarInfo other_car_info;
            other_car_info.ActorId = scenario.actors.others[other_idx].actor_id;
            other_car_info.ModelId = scenario.actors.others[other_idx].model_id;
            other_car_info.Geometory = carModel[model_idx].Geo;
            r = scenario.actors.others[other_idx].color.substr(0, 2);
            g = scenario.actors.others[other_idx].color.substr(2, 2);
            b = scenario.actors.others[other_idx].color.substr(4, 2);
            other_car_info.BodyColorR = stoi(r, nullptr, 16);
            other_car_info.BodyColorG = stoi(g, nullptr, 16);
            other_car_info.BodyColorB = stoi(b, nullptr, 16);
            other_car_info.CurrentFineRpIdx = 0.0;
            other_car_info.CurrentFineWpIdx = 0.0;
            other_car_info.CurrentVelocity = 0.0;
            other_car_info.WorkingVelocity = 0.0;
            for (int i = 0; i < other_car_info.WorkingVelocityBuf.size(); i++) {
                other_car_info.WorkingVelocityBuf[i] = 0.0;
            }
            other_car_info.CulcBrake.T = 0.0;
            other_car_info.CulcBrake.PrevVelocity = 0.0;
            other_car_info.CulcPitch.PrevVelocity = 0.0;
            other_car_info.CulcPitch.Accel = 0.0;
            other_car_info.CulcPitch.Pitch = 0.0;
            other_car_info.BlinkerLeft = false;
            other_car_info.BlinkerRight = false;
            other_car_info.BrakeLamp = false;
            other_car_info.Special1 = false;
            other_car_info.Sound = false;
            other_car_info.Action = scenario.scenario.opening_scene.others[other_idx].action;
            otherCarInfo.push_back(other_car_info);
        } else if (isPedestrianModelIdx(scenario.actors.others[other_idx].model_id)) {
            TPedestrianInfo pedestrian_info;
            pedestrian_info.ActorId = scenario.actors.others[other_idx].actor_id;
            pedestrian_info.ModelId = scenario.actors.others[other_idx].model_id;
            pedestrian_info.Action = scenario.scenario.opening_scene.others[other_idx].action;
            pedestrianInfo.push_back(pedestrian_info);
        } else {
            TObstacleInfo obstacle_info;
            obstacle_info.ActorId = scenario.actors.others[other_idx].actor_id;
            obstacle_info.ModelId = scenario.actors.others[other_idx].model_id;
            obstacleInfo.push_back(obstacle_info);
        }
    }
}

void _loadWaypoint(string path) {
    LoadWaypointJson(path, &waypointJson);

    int wps_idx = getWpsIdx(scenario.scenario.opening_scene.ego.start_position.wp_id);

    for (TPoint point: waypointJson.waypoints[wps_idx].points) {
        TVector3D waypoint;
        waypoint.X = point.x;
        waypoint.Y = point.y;
        waypoint.Z = point.z;
        egoCarInfo.Waypoint.push_back(waypoint);
    }

    createFineWaypoint(egoCarInfo.Waypoint, &egoCarInfo.FineWaypoint, 30.0 / 3.6, egoCarInfo.Geometory);

    for (int other_idx = 0; other_idx < otherCarInfo.size(); other_idx++) {
        int wps_idx = getWpsIdx(scenario.scenario.opening_scene.others[other_idx].start_position.wp_id);

        TVector3D next_wp, wp;
        next_wp.X = waypointJson.waypoints[wps_idx].points[0].x;
        next_wp.Y = waypointJson.waypoints[wps_idx].points[0].y;
        next_wp.Z = waypointJson.waypoints[wps_idx].points[0].z;

        double wp_offset = scenario.scenario.opening_scene.others[other_idx].start_position.wp_offset;

        for (int i = 1; i < waypointJson.waypoints[wps_idx].points.size(); i++) {
            wp = next_wp;
            next_wp.X = waypointJson.waypoints[wps_idx].points[i].x;
            next_wp.Y = waypointJson.waypoints[wps_idx].points[i].y;
            next_wp.Z = waypointJson.waypoints[wps_idx].points[i].z;

            double direction = atan2(next_wp.Y - wp.Y, next_wp.X - wp.X);
            if (wp_offset <= 0.0) {
                direction -= DegToRad(90.0);
            } else {
                direction += DegToRad(90.0);
            }
            wp.X += abs(wp_offset) * cos(direction);
            wp.Y += abs(wp_offset) * sin(direction);

            otherCarInfo[other_idx].Waypoint.push_back(wp);
        }

        createFineWaypoint(otherCarInfo[other_idx].Waypoint, &otherCarInfo[other_idx].FineWaypoint, 30.0 / 3.6, otherCarInfo[other_idx].Geometory);

        otherCarInfo[other_idx].CurrentWpsIdx = wps_idx;
    }
}

SimIniVal _getSimIniVal() {
    getEgoStartPosition();

    getOtherStartPosition();

    int rp_idx = int(egoCarInfo.CurrentFineRpIdx);

    SimIniVal sim_ini_val;
    sim_ini_val.map_id = scenario.map_id;
    sim_ini_val.wp_id = scenario.scenario.opening_scene.ego.start_position.wp_id;
    sim_ini_val.ego_model_id = scenario.actors.ego.model_id;
    sim_ini_val.ego_color_r = egoCarInfo.BodyColorR;
    sim_ini_val.ego_color_g = egoCarInfo.BodyColorG;
    sim_ini_val.ego_color_b = egoCarInfo.BodyColorB;
    sim_ini_val.ego_location_x = egoCarInfo.FineWaypoint[rp_idx].Pose.Location.X;
    sim_ini_val.ego_location_y = egoCarInfo.FineWaypoint[rp_idx].Pose.Location.Y;
    sim_ini_val.ego_location_z = egoCarInfo.FineWaypoint[rp_idx].Pose.Location.Z;
    sim_ini_val.ego_rotation_x = egoCarInfo.FineWaypoint[rp_idx].Pose.Rotation.X;
    sim_ini_val.ego_rotation_y = egoCarInfo.FineWaypoint[rp_idx].Pose.Rotation.Y;
    sim_ini_val.ego_rotation_z = egoCarInfo.FineWaypoint[rp_idx].Pose.Rotation.Z;
    sim_ini_val.pedestrian_count = pedestrianInfo.size();
    sim_ini_val.obstacle_count = obstacleInfo.size();

    cout << "vehicle " << otherCarInfo.size() << endl;
    cout << "pedestrian " << pedestrianInfo.size() << endl;
    cout << "obstacle " << obstacleInfo.size() << endl;

    return sim_ini_val;
}

void _startSimulation() {
    simStartTime = chrono::system_clock::now();
    cout << getElapsedSimTime() << " [scene] id " << currentScene.scene_id << endl;
    getNextSceneId(currentScene.next_scenes);
}

bool _updateEgoStatus(double ego_location_x, double ego_location_y, double ego_location_z,
                      double ego_rotation_x, double ego_rotation_y, double ego_rotation_z,
                      double ego_velocity_x, double ego_velocity_y, double ego_velocity_z,
                      double collision, double delta_time) {
    egoCarInfo.PrevPose = egoCarInfo.Pose;

    egoCarInfo.Pose.Location.X = ego_location_x;
    egoCarInfo.Pose.Location.Y = ego_location_y;
    egoCarInfo.Pose.Location.Z = ego_location_z;

    egoCarInfo.Pose.Rotation.X = ego_rotation_x;
    egoCarInfo.Pose.Rotation.Y = ego_rotation_y;
    egoCarInfo.Pose.Rotation.Z = ego_rotation_z;

    egoCarInfo.CurrentVelocity = sqrt(pow(ego_velocity_x, 2.0) + pow(ego_velocity_y, 2.0) + pow(ego_velocity_z, 2.0));

    egoCarInfo.Accel = (egoCarInfo.CurrentVelocity - egoCarInfo.PrevVelocity) / delta_time;

    egoCarInfo.PrevVelocity = egoCarInfo.CurrentVelocity;

    egoCarInfo.Collision = collision;

    egoCarInfo.CurrentFineRpIdx = getClosestFineRefPoint(egoCarInfo.Pose.Location, egoCarInfo.CurrentFineRpIdx);

    if (checkStopConditions()) {
        return true;
    }

    if (currentScene.next_scenes.size() == 0) {
        return true;
    }

    for (int other_idx = 0; other_idx < otherCarInfo.size(); other_idx++) {
        double prev_fine_wp_idx = otherCarInfo[other_idx].CurrentFineWpIdx;
        TAction *action = &otherCarInfo[other_idx].Action;
        if (action->traveled_distance.exec) {
            traveledDistanceAction(other_idx, action->traveled_distance, delta_time);
        }
        if (action->speed.exec) {
            speedAction(other_idx, action->speed, delta_time);
        }
        if (action->lane_change.exec) {
            laneChageAction(other_idx, action->lane_change);
            action->lane_change.exec = false;
        }
        if (action->lane_offset.exec) {
            laneOffsetAction(other_idx, action->lane_offset);
            action->lane_offset.exec = false;
        }
        if (action->route_move.exec && action->route_move.type == "wp_offset") {
            otherCarRouteMoveAction(other_idx, action->route_move, delta_time);
        }
        if (action->light_state.exec) {
            lightStateAction(other_idx, action->light_state);
            action->light_state.exec = false;
        }
        if (action->sound_state.exec) {
            soundStateAction(other_idx, action->sound_state);
            action->sound_state.exec = false;
        }

        // 現在位置
        otherCarInfo[other_idx].PrevPose = otherCarInfo[other_idx].Pose;
        otherCarInfo[other_idx].Pose = getOtherCarPose(other_idx).Pose;

        // 現在速度(移動平均)
        double v = 0.0;
        int i = 0;
        while (i < otherCarInfo[other_idx].WorkingVelocityBuf.size() - 1) {
            otherCarInfo[other_idx].WorkingVelocityBuf[i] = otherCarInfo[other_idx].WorkingVelocityBuf[1 + 1];
            v += otherCarInfo[other_idx].WorkingVelocityBuf[i];
            i++;
        }
        otherCarInfo[other_idx].WorkingVelocityBuf[i] = otherCarInfo[other_idx].WorkingVelocity;
        v += otherCarInfo[other_idx].WorkingVelocityBuf[i];
        otherCarInfo[other_idx].CurrentVelocity = v / otherCarInfo[other_idx].WorkingVelocityBuf.size();

        // タイヤの回転角
        double d = (otherCarInfo[other_idx].CurrentFineWpIdx - prev_fine_wp_idx) * FINE_WAYPOINT_UNIT;
        double rot_count = d / (otherCarInfo[other_idx].Geometory.WheelDiameter * M_PI);
        double rot = otherCarInfo[other_idx].WheelRotation - DegToRad(360.0 * rot_count);
        while (rot <= -DegToRad(360.0)) rot += DegToRad(360.0);
        otherCarInfo[other_idx].WheelRotation = rot;

        // ブレーキランプ
        otherCarInfo[other_idx].CulcBrake.T += delta_time;
        if (otherCarInfo[other_idx].CulcBrake.T > 0.5) {
            double accel = otherCarInfo[other_idx].CurrentVelocity - otherCarInfo[other_idx].CulcBrake.PrevVelocity;
            // 減速度が1.3m/s^2を超えたらブレーキランプ点灯
            if (accel <= -1.3) {
                otherCarInfo[other_idx].BrakeLamp = true;
            } else {
                otherCarInfo[other_idx].BrakeLamp = false;
            }
            otherCarInfo[other_idx].CulcBrake.T = 0.0;
            otherCarInfo[other_idx].CulcBrake.PrevVelocity = otherCarInfo[other_idx].CurrentVelocity;
        }

        // Pitch
        otherCarInfo[other_idx].CulcPitch.Accel = (otherCarInfo[other_idx].CurrentVelocity - otherCarInfo[other_idx].CulcPitch.PrevVelocity) * 1.5;
        if (otherCarInfo[other_idx].CulcPitch.Pitch < otherCarInfo[other_idx].CulcPitch.Accel - 1.0 * delta_time) {
            otherCarInfo[other_idx].CulcPitch.Pitch += 1.0 * delta_time;
        } else if (otherCarInfo[other_idx].CulcPitch.Pitch > otherCarInfo[other_idx].CulcPitch.Accel + 1.0 * delta_time) {
            otherCarInfo[other_idx].CulcPitch.Pitch -= 1.0 * delta_time;
        }
        otherCarInfo[other_idx].CulcPitch.PrevVelocity = otherCarInfo[other_idx].CurrentVelocity;

        // Roll
        // int wp_idx = otherCarInfo[other_idx].CurrentFineWpIdx;
        // TFineWaypoint fine_wp = otherCarInfo[other_idx].FineWaypoint[wp_idx];
        // double r = otherCarInfo[other_idx].Geometory.WheelBase / tan(fine_wp.WheelAngle);
        // otherCarInfo[other_idx].CulcRoll.F = -(pow(otherCarInfo[other_idx].CurrentVelocity, 2) / r) * 1.5;
        // if (otherCarInfo[other_idx].CulcRoll.Roll < otherCarInfo[other_idx].CulcRoll.F - 1.0 * delta_time) {
        //     otherCarInfo[other_idx].CulcRoll.Roll += 1.0 * delta_time;
        // } else if (otherCarInfo[other_idx].CulcRoll.Roll > otherCarInfo[other_idx].CulcRoll.F + 1.0 * delta_time) {
        //     otherCarInfo[other_idx].CulcRoll.Roll -= 1.0 * delta_time;
        // }
    }

    if (currentScene.duration > 0.0) {
        currentScene.duration -= delta_time;
    } else {
        getNextSceneId(currentScene.next_scenes);
    }

    return false;
}

PedestrianIniVal _getPedestrianStartPosition(int idx) {
    PedestrianIniVal pedestrian_ini_val = {"", 0.0, 0.0, 0.0, 0.0};

    for (TOpeningAction opening_action: scenario.scenario.opening_scene.others) {
        if (pedestrianInfo[idx].ActorId != opening_action.actor_id) {
            continue;
        }

        int wps_idx = getWpsIdx(opening_action.start_position.wp_id);

        int wp_idx1 = opening_action.start_position.wp_idx;
        if (wp_idx1 >= waypointJson.waypoints[wps_idx].points.size() - 1) {
            wp_idx1 = waypointJson.waypoints[wps_idx].points.size() - 2;
        }
        int wp_idx2 = wp_idx1 + 1;

        TPoint p1 = waypointJson.waypoints[wps_idx].points[wp_idx1];
        TPoint p2 = waypointJson.waypoints[wps_idx].points[wp_idx2];

        double direction = atan2(p2.y - p1.y, p2.x - p2.x);
        
        if (opening_action.start_position.wp_offset <= 0) {
            direction -= DegToRad(90.0);
        } else {
            direction += DegToRad(90.0);
        }

        pedestrianInfo[idx].CurrentPoint = waypointJson.waypoints[wps_idx].points[opening_action.start_position.wp_idx];
        pedestrianInfo[idx].CurrentPoint.x += abs(opening_action.start_position.wp_offset) * cos(direction);
        pedestrianInfo[idx].CurrentPoint.y += abs(opening_action.start_position.wp_offset) * sin(direction);
        pedestrianInfo[idx].Speed = 0.0;
        pedestrianInfo[idx].Direction = direction;
    
        pedestrian_ini_val.model_id = pedestrianInfo[idx].ModelId;
        pedestrian_ini_val.location_x = pedestrianInfo[idx].CurrentPoint.x;
        pedestrian_ini_val.location_y = pedestrianInfo[idx].CurrentPoint.y;
        pedestrian_ini_val.location_z = pedestrianInfo[idx].CurrentPoint.z;
        pedestrian_ini_val.rotation_z = pedestrianInfo[idx].Direction;
    }

    return pedestrian_ini_val;
}

PedestrianVal _updatePedestrianStatus(int idx, double location_x, double location_y, double delta_time) {
    pedestrianInfo[idx].CurrentPoint.x = location_x;
    pedestrianInfo[idx].CurrentPoint.y = location_y;

    TAction action = pedestrianInfo[idx].Action;
    if (action.route_move.exec && action.route_move.type == "waypoint") {
        pedestrianRouteMoveAction(action.route_move, idx, delta_time);
    }

    PedestrianVal pedestrian_val;
    pedestrian_val.speed = pedestrianInfo[idx].Speed;
    pedestrian_val.direction_x = cos(pedestrianInfo[idx].Direction);
    pedestrian_val.direction_y = sin(pedestrianInfo[idx].Direction);

    return pedestrian_val;
}

ObstacleIniVal _getObstaclePosition(int idx) {
    ObstacleIniVal obstacle_ini_val = {"", 0.0, 0.0, 0.0, 0.0};

    for (TOpeningAction opening_action: scenario.scenario.opening_scene.others) {
        if (obstacleInfo[idx].ActorId != opening_action.actor_id) {
            continue;
        }

        int wps_idx = getWpsIdx(opening_action.start_position.wp_id);

        int wp_idx1 = opening_action.start_position.wp_idx;
        if (wp_idx1 >= waypointJson.waypoints[wps_idx].points.size() - 1) {
            wp_idx1 = waypointJson.waypoints[wps_idx].points.size() - 2;
        }
        int wp_idx2 = wp_idx1 + 1;

        TPoint p1 = waypointJson.waypoints[wps_idx].points[wp_idx1];
        TPoint p2 = waypointJson.waypoints[wps_idx].points[wp_idx2];

        double direction = atan2(p2.y - p1.y, p2.x - p2.x);

        if (opening_action.start_position.wp_offset <= 0.0) {
            direction -= DegToRad(90.0);
        } else {
            direction += DegToRad(90.0);
        }

        obstacleInfo[idx].CurrentPoint = waypointJson.waypoints[wps_idx].points[opening_action.start_position.wp_idx];
        obstacleInfo[idx].CurrentPoint.x += abs(opening_action.start_position.wp_offset) * cos(direction);
        obstacleInfo[idx].CurrentPoint.y += abs(opening_action.start_position.wp_offset) * sin(direction);
    
        obstacle_ini_val.model_id = obstacleInfo[idx].ModelId;
        obstacle_ini_val.location_x = obstacleInfo[idx].CurrentPoint.x;
        obstacle_ini_val.location_y = obstacleInfo[idx].CurrentPoint.y;
        obstacle_ini_val.location_z = obstacleInfo[idx].CurrentPoint.z;
    }

    return obstacle_ini_val;
}
