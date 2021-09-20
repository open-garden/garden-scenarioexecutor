#!/bin/bash
source ./setup.sh

echo UE4 Contentのシンボリックリンクを作成します
if [ ! -d $CARLA_ROOT/Unreal/CarlaUE4/Content ]; then
  echo "ERROR directory $CARLA_ROOT/Unreal/CarlaUE4/Content does not exist."
  exit 1
fi
unlink $CARLA_ROOT/Unreal/CarlaUE4/Content/Garden 2>/dev/null
echo ln -s $PWD/Unreal/Content/Garden $CARLA_ROOT/Unreal/CarlaUE4/Content
ln -s $PWD/Unreal/Content/Garden $CARLA_ROOT/Unreal/CarlaUE4/Content 2>/dev/null

echo
echo UE4 Sourceのシンボリックリンクを作成します
if [ ! -d $CARLA_ROOT/Unreal/CarlaUE4/Source/CarlaUE4 ]; then
  echo "ERROR directory $CARLA_ROOT/Unreal/CarlaUE4/Source/CarlaUE4 does not exist."
  exit 1
fi
unlink $CARLA_ROOT/Unreal/CarlaUE4/Source/CarlaUE4/Garden 2>/dev/null
echo ln -s $PWD/Unreal/Source/Garden $CARLA_ROOT/Unreal/CarlaUE4/Source/CarlaUE4
ln -s $PWD/Unreal/Source/Garden $CARLA_ROOT/Unreal/CarlaUE4/Source/CarlaUE4 2>/dev/null
