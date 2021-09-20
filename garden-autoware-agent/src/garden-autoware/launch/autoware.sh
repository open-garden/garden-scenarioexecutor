#!/bin/bash
SCENARIO_EXECUTOR_ROOT=$(pwd)
source $SCENARIO_EXECUTOR_ROOT/setup.sh
source $SCENARIO_EXECUTOR_ROOT/garden-autoware-agent/devel/setup.bash
. ~/autoware.ai/install/setup.bash
roslaunch garden-autoware garden-autoware.launch $1 $2 $3
