#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <string>
#include <sys/shm.h>
#include <sys/stat.h>
#include <sys/ipc.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>
#include <float.h>
#include <iomanip>
#include "ScenarioExecutor.hpp"

extern TEgoCarInfo egoCarInfo;
extern vector<TOtherCarInfo> otherCarInfo;
extern vector<TCarModel> carModel;
extern TWaypointJson waypointJson;
extern TScene currentScene;

string key_file = "/garden.shm";
int segmentId;
int *sharedMemory;
string shmFilePath;
std::ofstream ofs;
double timestamp;
int currentWpIdx[100][100];

int getClosestWaypoint1(vector<TPoint> &waypoint, int current_wp_idx, TVector3D current_location) {
    double min_len = sqrt(pow(current_location.X - waypoint[current_wp_idx].x, 2.0) + 
                          pow(current_location.Y - waypoint[current_wp_idx].y, 2.0));
    double closest_wp = current_wp_idx;

    for (int i = current_wp_idx + 1; i < waypoint.size(); i++) {
        double len = sqrt(pow(current_location.X - waypoint[i].x, 2.0) + 
                          pow(current_location.Y - waypoint[i].y, 2.0));
        if (len > min_len) {
            break;
        }
        min_len = len;
        closest_wp = i;
    }

    return closest_wp;
}

int getClosestWaypoint(int actor_id, TVector3D current_location) {
    int min_wps_idx = 0;
    double min_len = DBL_MAX;

    for (int wps_idx = 0; wps_idx < waypointJson.waypoints.size(); wps_idx++) {
        int closest_wp_idx = getClosestWaypoint1(waypointJson.waypoints[wps_idx].points, currentWpIdx[actor_id][wps_idx], current_location);

        double len = sqrt(pow(waypointJson.waypoints[wps_idx].points[closest_wp_idx].x - current_location.X, 2) +
                          pow(waypointJson.waypoints[wps_idx].points[closest_wp_idx].y - current_location.Y, 2));

        if (len < min_len) {
            min_wps_idx = wps_idx;
            min_len = len;
        }

        currentWpIdx[actor_id][wps_idx] = closest_wp_idx;
    }

    return min_wps_idx;
}

void writeEgoCarPose(double location_x, double location_y, double location_z,
                     double rotation_x, double rotation_y, double rotation_z) {
    int i = EGO_CAR_INFO_PARAM_IDX;
    sharedMemory[i++] = (int)(location_x * 10000.0);
    sharedMemory[i++] = (int)(location_y * 10000.0);
    sharedMemory[i++] = (int)(location_z * 10000.0);
    sharedMemory[i++] = (int)(rotation_x * 10000.0);
    sharedMemory[i++] = (int)(rotation_y * 10000.0);
    sharedMemory[i++] = (int)(rotation_z * 10000.0);
}

void writeEgoCarControl() {
    int i = EGO_CAR_CONTROL_PARAM_IDX;
    sharedMemory[i++] = (int)egoCarInfo.Action.hand_over.hand_over;
    sharedMemory[i++] = (int)(egoCarInfo.Action.hand_over.throttle * 1000.0);
    sharedMemory[i++] = (int)(egoCarInfo.Action.hand_over.brake * 1000.0);
    sharedMemory[i++] = (int)(egoCarInfo.Action.hand_over.steer * 1000.0);
}

void writeOtherCarInfo(int other_idx) {
    int i = OTHER_CAR_INFO_PARAM_IDX + OTHER_CAR_INFO_PARAM_LEN * other_idx;
    sharedMemory[i] = 1;
    char *model_id = (char*)&sharedMemory[i + 1];
    int j = 0;
    while (j < otherCarInfo[other_idx].ModelId.size() && j < 63) {
        model_id[j] = otherCarInfo[other_idx].ModelId[j];
        j++;
    }
    while (j < 64) {
        model_id[j] = '\0';
        j++;
    }
    sharedMemory[i + 17] = otherCarInfo[other_idx].BodyColorR;
    sharedMemory[i + 18] = otherCarInfo[other_idx].BodyColorG;
    sharedMemory[i + 19] = otherCarInfo[other_idx].BodyColorB;
    sharedMemory[i + 20] = (otherCarInfo[other_idx].Geometory.WheelDiameter / 2.0) * 10000.0;
}

void writeOtherCarPose(int other_idx) {
    TFineWaypoint fine_wp = getOtherCarPose(other_idx);
    int special1 = (int)otherCarInfo[other_idx].Special1;
    int blinker_left = (int)otherCarInfo[other_idx].BlinkerLeft;
    int blinker_right = (int)otherCarInfo[other_idx].BlinkerRight;
    int brake_lamp = (int)otherCarInfo[other_idx].BrakeLamp;
    int sound = (int)otherCarInfo[other_idx].Sound;
    int i = OTHER_CAR_INFO_PARAM_IDX + OTHER_CAR_INFO_PARAM_LEN * other_idx + 21;
    sharedMemory[i++] = (int)(fine_wp.Pose.Location.X * 10000.0);
    sharedMemory[i++] = (int)(-fine_wp.Pose.Location.Y * 10000.0);
    sharedMemory[i++] = (int)(fine_wp.Pose.Location.Z * 10000.0);
    sharedMemory[i++] = (int)(RadToDeg(fine_wp.Pose.Rotation.X) * 10000.0);
    sharedMemory[i++] = (int)((RadToDeg(fine_wp.Pose.Rotation.Y) + otherCarInfo[other_idx].CulcPitch.Pitch) * 10000.0);
    sharedMemory[i++] = (int)(RadToDeg(-fine_wp.Pose.Rotation.Z) * 10000.0);
    sharedMemory[i++] = (int)(RadToDeg(fine_wp.WheelAngle) * 10000.0);
    sharedMemory[i++] = (int)(RadToDeg(otherCarInfo[other_idx].WheelRotation) * 10000.0);
    sharedMemory[i++] = (sound << 4) | (special1 << 3) | (blinker_left << 2) | (blinker_right << 1) | brake_lamp;
}

void writeOtherCarInfoEnd() {
    int i = OTHER_CAR_INFO_PARAM_IDX + OTHER_CAR_INFO_PARAM_LEN * otherCarInfo.size();
    sharedMemory[i] = 0;
}

void setScenarioExeInfoParam(int index, int value) {
    sharedMemory[SCENARIO_EXE_INFO_PARAM_IDX + index] = value;
}

void createLogFile(string path) {
    struct stat st;
    // logフォルダがなければ作成
    if (stat("log", &st) != 0) {
        mkdir("log", 0775);
    }

    // シナリオファイル名の拡張子を".csv"に変更
    string ext = path.substr(path.size() - 5, 5);
    if (ext == ".json") {
        path = path.substr(0, path.size() - 5);
    }
    path += ".csv";

    // パスからシナリオファイル名のみ取得
    char* c_path = new char[path.size() + 1];
    std::char_traits<char>::copy(c_path, path.c_str(), path.size() + 1);
    string log_file_name(basename(c_path));
    delete[] c_path;

    ofs.open("./log/" + log_file_name);

    // 現在の日時を取得
    time_t t = time(nullptr);
    const tm* localTime = localtime(&t);
    char date_time[70];
    std::strftime(date_time, sizeof(date_time) - 1, "%Y-%m-%d-%H-%M-%S", localTime);
    ofs << date_time << endl;

    ofs << "timestamp,scene_id,ego_x,ego_y,ego_z,ego_roll,ego_pitch,ego_yaw,ego_lane_id,ego_velocity,collision";

    int obs_count = 0;

    for (int other_idx = 0; other_idx < otherCarInfo.size(); other_idx++) {
        ofs << ",obs" << obs_count << "_x";
        ofs << ",obs" << obs_count << "_y";
        ofs << ",obs" << obs_count << "_z";
        ofs << ",obs" << obs_count << "_roll";
        ofs << ",obs" << obs_count << "_pitch";
        ofs << ",obs" << obs_count << "_yaw";
        ofs << ",obs" << obs_count << "_lane_id";
        obs_count++;
    }
    ofs << endl;

    timestamp = 0.0;
}

void writeLogFile(double delta_time) {
    timestamp += delta_time;

    ofs << timestamp;
    ofs << "," << currentScene.scene_id;
    ofs << "," << egoCarInfo.Pose.Location.X;
    ofs << "," << egoCarInfo.Pose.Location.Y;
    ofs << "," << egoCarInfo.Pose.Location.Z;
    ofs << "," << egoCarInfo.Pose.Rotation.X;
    ofs << "," << egoCarInfo.Pose.Rotation.Y;
    ofs << "," << egoCarInfo.Pose.Rotation.Z;

    TVector3D current_location = {egoCarInfo.Pose.Location.X, egoCarInfo.Pose.Location.Y, egoCarInfo.Pose.Location.Z};
    int wps_idx = getClosestWaypoint(egoCarInfo.ActorId, current_location);

    ofs << "," <<  wps_idx;
    ofs << "," << egoCarInfo.CurrentVelocity;
    ofs << "," << egoCarInfo.Collision;

    for (int other_idx = 0; other_idx < otherCarInfo.size(); other_idx++) {
        TPose pose = otherCarInfo[other_idx].FineWaypoint[otherCarInfo[other_idx].CurrentFineWpIdx].Pose;
        ofs << "," << pose.Location.X;
        ofs << "," << pose.Location.Y;
        ofs << "," << pose.Location.Z;
        ofs << "," << pose.Rotation.X;
        ofs << "," << pose.Rotation.Y;
        ofs << "," << pose.Rotation.Z;

        TVector3D current_location = {pose.Location.X, pose.Location.Y, pose.Location.Z};
        int wps_idx = getClosestWaypoint(otherCarInfo[other_idx].ActorId, current_location);

        ofs << "," <<  wps_idx;
    }

    ofs << endl;
}

void closeLogFile() {
    ofs.close();
}

int attachScenarioExecutor() {
    string home = getenv("HOME");
    shmFilePath = home + key_file;

    ofstream ofs(shmFilePath, ios::app);

    auto key = ftok(shmFilePath.c_str(), 1);
    if(key == -1) {
        return EXIT_FAILURE;
    }

    printf("Shared memory ftok: %s -> %d\n", shmFilePath.c_str(), key);

    segmentId = shmget(key, OTHER_CAR_INFO_PARAM_IDX + OTHER_CAR_INFO_PARAM_LEN * sizeof(int) * 100, IPC_CREAT | IPC_EXCL | S_IRUSR | S_IWUSR);
    if(segmentId == -1) {
        return EXIT_FAILURE;
    }

    sharedMemory = reinterpret_cast<int*>(shmat(segmentId, 0, 0));

    ofs << segmentId << endl;
    ofs << sharedMemory << endl;

    printf("Shared memory attached at address %p.\n", sharedMemory);

    return EXIT_SUCCESS;
}

void detachScenarioExecutor() {
    shmctl(segmentId, IPC_RMID, 0);
    shmdt(sharedMemory);

    printf("Shared memory detached at address %p.\n", sharedMemory);

    remove(shmFilePath.c_str());
}

void loadScenario(string path) {
    _loadScenario(path);

    // ログファイルを作成
    createLogFile(path);
}

void loadWaypoint(string path) {
    _loadWaypoint(path);
}

SimIniVal getSimIniVal() {
    SimIniVal sim_ini_val = _getSimIniVal();
    sim_ini_val.ego_rotation_x = RadToDeg(sim_ini_val.ego_rotation_x);
    sim_ini_val.ego_rotation_y = RadToDeg(sim_ini_val.ego_rotation_y);
    sim_ini_val.ego_rotation_z = RadToDeg(-sim_ini_val.ego_rotation_z);

    return sim_ini_val;
}

void startSimulation() {
    _startSimulation();

    for (int other_idx = 0; other_idx < otherCarInfo.size(); other_idx++) {
        writeOtherCarInfo(other_idx);
        writeOtherCarPose(other_idx);
        cout << otherCarInfo[other_idx].ModelId << " spawn_point";
        cout << " " << otherCarInfo[other_idx].FineWaypoint[otherCarInfo[other_idx].CurrentFineWpIdx].Pose.Location.X;
        cout << " " << otherCarInfo[other_idx].FineWaypoint[otherCarInfo[other_idx].CurrentFineWpIdx].Pose.Location.Y;
        cout << " " << otherCarInfo[other_idx].FineWaypoint[otherCarInfo[other_idx].CurrentFineWpIdx].Pose.Location.Z;
        cout << endl;
    }

    writeOtherCarInfoEnd();

    setScenarioExeInfoParam(0, 1);
}

void endSimulation() {
    setScenarioExeInfoParam(0, 0);

    closeLogFile();
}

bool updateEgoStatus(double location_x, double location_y, double location_z,
                     double rotation_x, double rotation_y, double rotation_z,
                     double velocity_x, double velocity_y, double velocity_z,
                     double collision, double delta_time) {

    writeEgoCarPose(location_x, location_y, location_z, rotation_x, rotation_y, rotation_z);

    writeEgoCarControl();

    bool end_flag = _updateEgoStatus(location_x, -location_y, location_z,
                                     DegToRad(rotation_x), DegToRad(rotation_y), DegToRad(-rotation_z),
                                     velocity_x, velocity_y, velocity_z,
                                     collision, delta_time);

    for (int other_idx = 0; other_idx < otherCarInfo.size(); other_idx++) {
        writeOtherCarPose(other_idx);
    }
    
    if (end_flag) {
        setScenarioExeInfoParam(0, 0);
    }

    writeLogFile(delta_time);

    return end_flag;
}

PedestrianIniVal getPedestrianStartPosition(int idx) {
    PedestrianIniVal pedestrian_ini_val = _getPedestrianStartPosition(idx);

    pedestrian_ini_val.location_y = -pedestrian_ini_val.location_y;
    pedestrian_ini_val.rotation_z = RadToDeg(-pedestrian_ini_val.rotation_z);

    return pedestrian_ini_val;
}

PedestrianVal updatePedestrianStatus(int idx, double location_x, double location_y, double delta_time) {
    PedestrianVal pedestrian_val = _updatePedestrianStatus(idx, location_x, -location_y, delta_time);

    pedestrian_val.direction_y = -pedestrian_val.direction_y;

    return pedestrian_val;
}

ObstacleIniVal getObstaclePosition(int idx) {
    ObstacleIniVal obstacle_ini_val = _getObstaclePosition(idx);

    obstacle_ini_val.location_y = -obstacle_ini_val.location_y;
    obstacle_ini_val.rotation_z = RadToDeg(-obstacle_ini_val.rotation_z);

    return obstacle_ini_val;
}
