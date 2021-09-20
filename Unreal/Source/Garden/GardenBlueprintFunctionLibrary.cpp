// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#include "CarlaUE4.h"
#include "GardenBlueprintFunctionLibrary.h"
#include <stdio.h>
#include <stdlib.h>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>
#include <sys/shm.h>
#include <sys/stat.h>
#include <fstream>

DEFINE_LOG_CATEGORY(LogGarden);

std::string key_file = "/garden.shm";
bool connected = false;
int *sharedMemory;
int newVehicleID = 0;

void UGardenBlueprintFunctionLibrary::initializeScenarioExecutor()
{
    std::string home = getenv("HOME");
    const std::string file_path(home + key_file);

    std::ifstream ifs(file_path);

    if (ifs.is_open()) {
        std::string segment_id_str;
        getline(ifs, segment_id_str);
        if (segment_id_str != "") {
            int segment_id = stoi(segment_id_str);
            shmctl(segment_id, IPC_RMID, 0);
        }

        std::string shared_memory_str;
        getline(ifs, shared_memory_str);
        if (shared_memory_str != "") {
            int *shared_memory;
            shared_memory = (int*)strtol(shared_memory_str.c_str(), NULL, 16);
            shmdt(shared_memory);
        }

        remove(file_path.c_str());
    }

    newVehicleID = 0;
}

bool UGardenBlueprintFunctionLibrary::connectScenarioExecutor()
{
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

    FString s = file_path.c_str();
    UE_LOG(LogGarden, Log, TEXT("Shared memory ftok: %s -> %d"), *s, key);

    const auto segment_id = shmget(key, 0, 0);
    if(segment_id == -1) {
        return false;
    }

    sharedMemory = reinterpret_cast<int*>(shmat(segment_id, 0, 0));

    UE_LOG(LogGarden, Log, TEXT("Shared memory attached at address %p."), sharedMemory);

    connected = true;

    return true;
}

void UGardenBlueprintFunctionLibrary::disconnectScenarioExecutor()
{
    shmdt(sharedMemory);

    UE_LOG(LogGarden, Log, TEXT("Shared memory detached at address %p."), sharedMemory);

    connected = false;
}

bool UGardenBlueprintFunctionLibrary::isConnected()
{
    return connected;
}

int UGardenBlueprintFunctionLibrary::getParamLen()
{
    return OTHER_CAR_INFO_PARAM_LEN;
}

int UGardenBlueprintFunctionLibrary::getScenarioExeInfoParam(int index)
{
	return sharedMemory[SCENARIO_EXE_INFO_PARAM_IDX + index];
}

int UGardenBlueprintFunctionLibrary::getEgoCarInfoParam(int index)
{
	return sharedMemory[EGO_CAR_INFO_PARAM_IDX + index];
}

int UGardenBlueprintFunctionLibrary::getOtherCarInfoParam(int index)
{
	return sharedMemory[OTHER_CAR_INFO_PARAM_IDX + index];
}

void UGardenBlueprintFunctionLibrary::setNewVehicleID(int id)
{
	newVehicleID = id;
}

int UGardenBlueprintFunctionLibrary::getNewVehicleID()
{
	return newVehicleID;
}

FString UGardenBlueprintFunctionLibrary::getModelId(int index)
{
    std::string model_id = "";

    char *c = (char*)&sharedMemory[OTHER_CAR_INFO_PARAM_IDX + index];

    for (int i = 0; i < 256; i++) {
        model_id += c[i];
        if (c[i] == '\0') {
            break;
        }
    }

	return model_id.c_str();
}
