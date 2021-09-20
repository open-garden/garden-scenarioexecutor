#!/bin/bash
SCENARIO_EXECUTER_ROOT=$(pwd)
source ${SCENARIO_EXECUTER_ROOT}/setup.sh
python ${CARLA_ROOT}/PythonAPI/examples/manual_control.py --filter $1 --res 1280x960
