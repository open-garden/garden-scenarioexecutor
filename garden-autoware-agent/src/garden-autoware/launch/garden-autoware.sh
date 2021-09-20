#!/bin/bash
SCENARIO_EXECUTOR_ROOT=$(pwd)
gnome-terminal --tab -- bash -c "bash $SCENARIO_EXECUTOR_ROOT/garden-autoware-agent/src/garden-autoware/launch/autoware.sh $1 $2 $3; bash"
gnome-terminal --tab -- bash $SCENARIO_EXECUTOR_ROOT/garden-autoware-agent/src/garden-autoware/launch/rviz.sh