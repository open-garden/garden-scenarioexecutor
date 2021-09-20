#!/usr/bin/python
# -*- coding: utf-8 -*-
import sys
import subprocess
import math
import time
import os
import carla

sys.path.append('./LibScenarioExecutor/build')

import LibScenarioExecutor

collision_impulse = 0.0

def get_ego_vehicle(world, role_name):
    actor_list = world.get_actors()
    for actor in actor_list:
        if actor.type_id.startswith('vehicle'):
            if actor.attributes.get('role_name') == role_name:
                return actor
    return None

def on_collision(event):
    global collision_impulse
    impulse = event.normal_impulse
    collision_impulse = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)

def spawn_pedestrian(world, transform, model_id, i):
    bp = world.get_blueprint_library().find(model_id.encode('utf-8'))
    bp.set_attribute('role_name', 'pedestrian{}'.format(i))
    actor = world.try_spawn_actor(bp, transform)
    return actor

def spawn_obstacle(world, transform, model_id, i):
    bp = world.get_blueprint_library().find(model_id.encode('utf-8'))
    bp.set_attribute('role_name', 'obstacle{}'.format(i))
    actor = world.try_spawn_actor(bp, transform)
    return actor

def main(world, ego_vehicle, sim_ini_val):
    global collision_impulse

    collision_sensor = None
    pedestrian_list = []
    obstacle_list = []

    try:
        bp = world.get_blueprint_library().find('sensor.other.collision')
        collision_sensor = world.spawn_actor(bp, carla.Transform(), attach_to=ego_vehicle)
        collision_sensor.listen(lambda event: on_collision(event))

        # 歩行者をスポーンする
        for i in range(sim_ini_val.pedestrian_count):
            pedestrian_ini_val = LibScenarioExecutor.getPedestrianStartPosition(i)
            transform = carla.Transform(
                carla.Location(pedestrian_ini_val.location_x, pedestrian_ini_val.location_y, pedestrian_ini_val.location_z + 2),
                carla.Rotation(0.0, pedestrian_ini_val.rotation_z, 0.0))
            pedestrian = spawn_pedestrian(world, transform, pedestrian_ini_val.model_id, i)
            pedestrian_control = pedestrian.get_control()
            pedestrian_control.speed = 0.0
            pedestrian_control.jump = False
            pedestrian.apply_control(pedestrian_control)
            pedestrian_list.append(pedestrian)

        # 障害物をスポーンする
        for i in range(sim_ini_val.obstacle_count):
            obstacle_ini_val = LibScenarioExecutor.getObstaclePosition(i)
            transform = carla.Transform(
                carla.Location(obstacle_ini_val.location_x, obstacle_ini_val.location_y, obstacle_ini_val.location_z),
                carla.Rotation(0.0, obstacle_ini_val.rotation_z, 0.0))
            obstacle = spawn_obstacle(world, transform, obstacle_ini_val.model_id, i)
            obstacle_list.append(obstacle)

        # UE4とScenarioExecutorを接続する
        LibScenarioExecutor.attachScenarioExecutor()

        LibScenarioExecutor.startSimulation()

        time.sleep(2.0)

        collision = False

        while True:
            timestamp = world.wait_for_tick()
            delta_seconds = timestamp.delta_seconds

            transform = ego_vehicle.get_transform()
            velocity = ego_vehicle.get_velocity()

            if collision_impulse > 0.0:
                collision = True

            if LibScenarioExecutor.updateEgoStatus(
                transform.location.x, transform.location.y, transform.location.z,
                transform.rotation.roll, transform.rotation.pitch, transform.rotation.yaw,
                velocity.x, velocity.y, velocity.z,
                collision_impulse, delta_seconds):
                break
            
            for i in range(len(pedestrian_list)):
                location = pedestrian_list[i].get_location()
                pedestrian_val = LibScenarioExecutor.updatePedestrianStatus(i, location.x, location.y, delta_seconds)
                pedestrian_control.speed = pedestrian_val.speed
                pedestrian_control.direction = carla.Vector3D(pedestrian_val.direction_x, pedestrian_val.direction_y, 0.0)
                pedestrian_list[i].apply_control(pedestrian_control)

            if collision:
                collision_impulse = 0.0

    except (KeyboardInterrupt):
        pass
    finally:
        LibScenarioExecutor.endSimulation()

        LibScenarioExecutor.detachScenarioExecutor()

        if collision_sensor is not None:
            collision_sensor.destroy()

        for pedestrian in pedestrian_list:
            pedestrian.destroy()

        for obstacle in obstacle_list:
            obstacle.destroy()

if __name__ == '__main__':
    try:
        if len(sys.argv) >= 3:
            scenario = sys.argv[1]
            waypoint = sys.argv[2]
            autoware = ''

        autoware = False

        if len(sys.argv) >= 4 and sys.argv[3].lower() == 'autoware':
            autoware = True

        LibScenarioExecutor.loadScenario(scenario)
        LibScenarioExecutor.loadWaypoint(waypoint)

        sim_ini_val = LibScenarioExecutor.getSimIniVal()

        sim_ini_val.ego_location_z += 1.0

        print("town " + sim_ini_val.map_id)
        print("ego model " + sim_ini_val.ego_model_id)
        print("ego waypoint " + sim_ini_val.wp_id)
        print('ego spawn_point {},{},{},{},{},{}'.format(
            sim_ini_val.ego_location_x,
            sim_ini_val.ego_location_y,
            sim_ini_val.ego_location_z,
            sim_ini_val.ego_rotation_y,
            sim_ini_val.ego_rotation_z,
            sim_ini_val.ego_rotation_x))

        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(2.0)

        ego_vehicle = None

        if autoware:
            world = client.get_world()

            wp_file_name = '{}.{}.csv'.format(os.path.splitext(os.path.basename(waypoint))[0], sim_ini_val.wp_id)
            print("autoware waypoint_file " + wp_file_name)

            arg_town = 'town:={}'.format(sim_ini_val.map_id)
            arg_spawn_point = 'spawn_point:={},{},{},{},{},{}'.format(
                sim_ini_val.ego_location_x,
                sim_ini_val.ego_location_y,
                sim_ini_val.ego_location_z,
                sim_ini_val.ego_rotation_x,
                sim_ini_val.ego_rotation_y,
                -sim_ini_val.ego_rotation_z,
                )
            arg_waypoint = 'waypoint:={}'.format(wp_file_name)

            # ROS-bridge + Autowareを起動する
            subprocess.call(['gnome-terminal', '--', 'bash', './garden-autoware-agent/src/garden-autoware/launch/garden-autoware.sh',
                arg_town, arg_spawn_point, arg_waypoint])

            for i in range(10):
                time.sleep(1.0)
                ego_vehicle = get_ego_vehicle(world, 'ego_vehicle')
                if ego_vehicle is not None:
                    break
        else:
            world = client.load_world(sim_ini_val.map_id.encode('utf-8'))

            # manual_controlを起動する
            subprocess.call(['gnome-terminal', '--', 'bash', './util/manual_control/manual_control.sh', sim_ini_val.ego_model_id])

            for i in range(10):
                time.sleep(1.0)
                ego_vehicle = get_ego_vehicle(world, 'hero')
                if ego_vehicle is not None:
                    break

            ego_vehicle.set_transform(
                carla.Transform(carla.Location(
                    sim_ini_val.ego_location_x,
                    sim_ini_val.ego_location_y,
                    sim_ini_val.ego_location_z),
                carla.Rotation(
                    sim_ini_val.ego_rotation_y,
                    sim_ini_val.ego_rotation_z,
                    sim_ini_val.ego_rotation_x)))

        main(world, ego_vehicle, sim_ini_val)

    except (KeyboardInterrupt):
        pass
    finally:
        print('done.\n')
