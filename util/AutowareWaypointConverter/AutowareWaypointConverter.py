#! /usr/bin/python
# -*- coding: utf-8 -*-
import sys
import json
import math
import os

base_file_name = os.path.splitext(os.path.basename(sys.argv[1]))[0]

v = -1.0
if len(sys.argv) >= 3:
    v = float(sys.argv[2])

f = open(sys.argv[1], 'r')
waypoint_json = json.load(f)

for waypoints in waypoint_json['waypoints']:
    file_name = '{}.{}.{}'.format(base_file_name, waypoints['id'], 'csv')
    print(file_name)
    with open(file_name, mode='w') as f:
        f.write('x,y,z,yaw,velocity,change_flag\n')
        wp_x = float(waypoints['points'][0]['x'])
        wp_y = -float(waypoints['points'][0]['z'])
        wp_z = float(waypoints['points'][0]['y'])
        for i in range(len(waypoints['points'])):
            next_wp_x = float(waypoints['points'][i]['x'])
            next_wp_y = -float(waypoints['points'][i]['z'])
            next_wp_z = float(waypoints['points'][i]['y'])
            l = int(math.sqrt((next_wp_x - wp_x)**2 + (next_wp_y - wp_y)**2))
            if l > 0:
                angle = math.atan2(next_wp_y - wp_y, next_wp_x - wp_x)
                angle_z = math.atan2(next_wp_z - wp_z, l)
                while l > 0:
                    if v < 0.0:
                        v = float(waypoints['points'][i]['v'])
                    f.write('{},{},{},{},{},{}\n'.format(wp_x, wp_y, wp_z, angle, v, 0))
                    wp_x += math.cos(angle)
                    wp_y += math.sin(angle)
                    wp_z += math.sin(angle_z)
                    l -= 1
