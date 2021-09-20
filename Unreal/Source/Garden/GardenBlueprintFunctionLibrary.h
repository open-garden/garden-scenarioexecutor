// Copyright (c) 2017 Computer Vision Center (CVC) at the Universitat Autonoma de Barcelona (UAB). This work is licensed under the terms of the MIT license. For a copy, see <https://opensource.org/licenses/MIT>.

#pragma once

#include "CoreMinimal.h"
#include "Kismet/BlueprintFunctionLibrary.h"
#include "GardenBlueprintFunctionLibrary.generated.h"
	
DECLARE_LOG_CATEGORY_EXTERN(LogGarden, Log, All);

#define SCENARIO_EXE_INFO_PARAM_IDX 0
#define EGO_CAR_INFO_PARAM_IDX 1
#define OTHER_CAR_INFO_PARAM_IDX 11
#define OTHER_CAR_INFO_PARAM_LEN 30

UCLASS()
class CARLAUE4_API UGardenBlueprintFunctionLibrary : public UBlueprintFunctionLibrary
{
	GENERATED_BODY()
	
	UFUNCTION(BlueprintCallable, Category = "Garden")
		static void initializeScenarioExecutor();
	
	UFUNCTION(BlueprintCallable, Category = "Garden")
		static bool connectScenarioExecutor();

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static void disconnectScenarioExecutor();

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static bool isConnected();

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static int getParamLen();

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static int getScenarioExeInfoParam(int index);

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static int getEgoCarInfoParam(int index);

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static int getOtherCarInfoParam(int index);

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static void setNewVehicleID(int id);

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static int getNewVehicleID();

	UFUNCTION(BlueprintCallable, Category = "Garden")
		static FString getModelId(int index);

};
