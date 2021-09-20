cmake_minimum_required(VERSION 3.2)
project(ScenarioExecutor)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PYBIND11_PYTHON_VERSION 2.7)

set(THREADS_PREFER_PTHREAD_FLAG ON)              

find_package(Threads REQUIRED)                   

add_subdirectory(pybind11)

add_library(_scenario_executor
    ScenarioExecutor.cpp
    ScenarioExecutorCore.cpp
    ScenarioJsonLoader.cpp
    WaypointJsonLoader.cpp
    LaneChangeThread.cpp
)

pybind11_add_module(LibScenarioExecutor LibScenarioExecutor.cpp)
target_link_libraries(LibScenarioExecutor PRIVATE _scenario_executor pthread)

