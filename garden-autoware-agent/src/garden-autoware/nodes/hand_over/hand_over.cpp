#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <carla_msgs/CarlaEgoVehicleInfo.h>
#include <carla_msgs/CarlaEgoVehicleControl.h>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fstream>

#define EGO_CAR_CONTROL_PARAM_IDX 7

std::string key_file = "/garden.shm";

ros::Subscriber vehicle_info_sub;
ros::Publisher manual_override_pub;
ros::Publisher control_command_pub;

bool sim_start = false;
int *sharedMemory;

bool connectScenarioExecuter() {
    std::string home = getenv("HOME");
    const std::string file_path(home + key_file);

    std::ifstream ifs(file_path);

    if (!ifs.is_open()) {
        return false;
  	}

    const auto key = ftok(file_path.c_str(), 1);
    if(key == -1) {
        return false;
    }

    const auto segment_id = shmget(key, 0, 0);
    if(segment_id == -1) {
        return false;
    }

    sharedMemory = reinterpret_cast<int*>(shmat(segment_id, 0, 0));

    return true;
}

void disconnectScenarioExecuter() {
    shmdt(sharedMemory);
}

int getEgoCarControlParam(int index) {
    return sharedMemory[EGO_CAR_CONTROL_PARAM_IDX + index];
}

void vehicle_info_callback(carla_msgs::CarlaEgoVehicleInfo msg) {
    sim_start = true;
    vehicle_info_sub.shutdown();
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hand_over");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::Rate loop_rate(10);

    std::string role_name = "ego_vehicle";
    pnh.getParam("role_name", role_name);
    
    vehicle_info_sub = nh.subscribe("/carla/" + role_name + "/vehicle_info", 10, vehicle_info_callback);

    manual_override_pub = nh.advertise<std_msgs::Bool>("/carla/" + role_name + "/vehicle_control_manual_override", 1);
    control_command_pub = nh.advertise<carla_msgs::CarlaEgoVehicleControl>("/carla/" + role_name + "/vehicle_control_cmd_manual", 1);

    bool connected = false;
    bool override_state = false;

    while (ros::ok()) {
        loop_rate.sleep();

        ros::spinOnce();

        if (!sim_start) {
            continue;
        }

        if (!connected) {
            connected = connectScenarioExecuter();
            continue;
        }
  
        bool override = (bool)getEgoCarControlParam(0);

        if (override) {
             std_msgs::Bool override_msg;
             override_msg.data = true;
             manual_override_pub.publish(override_msg);

             double throttle = (double)getEgoCarControlParam(1) / 1000.0;
             double brake = (double)getEgoCarControlParam(2) / 1000.0;
             double steer = (double)getEgoCarControlParam(3) / 1000.0;

             carla_msgs::CarlaEgoVehicleControl control_msg;
             control_msg.throttle = throttle;
             control_msg.steer = steer;
             control_msg.brake = brake;

             control_command_pub.publish(control_msg);
        } else {
            if (override_state) {
                std_msgs::Bool override_msg;
                override_msg.data = false;
                manual_override_pub.publish(override_msg);
            }
        }

        override_state = override;
    }

    disconnectScenarioExecuter();

    return 0;
}
