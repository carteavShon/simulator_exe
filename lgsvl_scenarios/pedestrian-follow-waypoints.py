#!/usr/bin/env python3
#
# Copyright (c) 2019-2021 LG Electronics, Inc.
#
# This software contains code licensed as described in LICENSE.
#

import math
from environs import Env
import lgsvl
import os
import time
import sys


print("Python API Quickstart #16: Pedestrian following waypoints")
env = Env()

LGSVL__SIMULATOR_HOST = env.str("LGSVL__SIMULATOR_HOST", "127.0.0.1")
LGSVL__SIMULATOR_PORT = env.int("LGSVL__SIMULATOR_PORT", 8181)
BRIDGE_HOST = os.environ.get("LGSVL__AUTOPILOT_0_HOST", "127.0.0.1")
BRIDGE_PORT = int(os.environ.get("LGSVL__AUTOPILOT_0_PORT", 9090))

sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))
if sim.current_scene == "AutonomouStuff":
    sim.reset()
else:
    sim.load("AutonomouStuff")

spawns = sim.get_spawn()

forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])

state = lgsvl.AgentState()
state.transform = spawns[0]

ego = sim.add_agent("4a4f4856-446c-416d-96b2-d4aa53dc9c42", lgsvl.AgentType.EGO, state)

# An EGO will not connect to a bridge unless commanded to
print("Bridge connected:", ego.bridge_connected)

# The EGO is now looking for a bridge at the specified IP and port
ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST", lgsvl.wise.SimulatorSettings.bridge_host), env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port))

print("Waiting for connection...")

while not ego.bridge_connected:
    time.sleep(1)

print("Bridge connected:", ego.bridge_connected)

#ego.connect_bridge(BRIDGE_HOST, BRIDGE_PORT)
# This will create waypoints in a circle for the pedestrian to follow
radius = 4
count = 8
wp = []
for i in range(count):
    x = radius * math.cos(i * 2 * math.pi / count)
    z = radius * math.sin(i * 2 * math.pi / count)
    # idle is how much time the pedestrian will wait once it reaches the waypoint
    idle = 1 if i < count // 2 else 0
    wp.append(
        lgsvl.WalkWaypoint(spawns[0].position + x * right + (z + 8) * forward, idle)
    )

state = lgsvl.AgentState()
state.transform = spawns[0]
state.transform.position = wp[0].position

p = sim.add_agent("Pamela", lgsvl.AgentType.PEDESTRIAN, state)

def on_waypoint(agent, index):
    print("Waypoint {} reached".format(index))


p.on_waypoint_reached(on_waypoint)

# This sends the list of waypoints to the pedestrian. The bool controls whether or not the pedestrian will continue walking (default false)
p.follow(wp, True)

input("Press Enter to walk in circle for 600 seconds")

sim.run(600)