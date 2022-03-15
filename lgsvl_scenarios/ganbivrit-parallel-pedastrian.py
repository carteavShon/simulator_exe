#!/usr/bin/env python3

import math
from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
from scenarios_ros_thread import ScenariosRosThread
import simulator_types
import time
from datetime import datetime

from permutation_params import PermutationParams

now = datetime.now()
current_time = now.strftime("%H:%M:%S")

scenario_num = 20
scenario_name = "FROM RECORDING"
scene = simulator_types.map_types.GanBIvrit

print('Scenrio #' + str(scenario_num)+ '. ' +scenario_name)

vehicle=simulator_types.ego_types.DefaultType
print('Vehicle: ' + vehicle.name + ' - ' + vehicle.value)

params = PermutationParams(scenario_num)
randoms = params.input_params()

ros_thread = ScenariosRosThread()
ros_thread.run(scenario_name)


env = Env()
sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))

if sim.current_scene == scene.value:
    sim.reset()
else:
    sim.load(scene.value)

ego_state = lgsvl.AgentState()
ego_state.transform = Transform(position = lgsvl.Vector(904.27,0,4756.42), 
                                    rotation = lgsvl.Vector(0,-76.4,0))

forward = lgsvl.utils.transform_to_forward(ego_state)
right = lgsvl.utils.transform_to_right(ego_state)

ego = sim.add_agent(vehicle.value,lgsvl.AgentType.EGO,ego_state)

ego.connect_bridge(env.str("LGSVL_AUTOPILOT_0_HOST",lgsvl.wise.SimulatorSettings.bridge_host),env.int("LGSVL_AUTOPILOT_0_PORT",lgsvl.wise.SimulatorSettings.bridge_port))

print("Waiting For Connection ...")

while not ego.bridge_connected:
    time.sleep(1)

print("Bridge connected:", ego.bridge_connected)

sim.add_random_agents(lgsvl.AgentType.PEDESTRIAN)

ros_thread.send_drive_to_point(
    31.9634879,
    34.8258068,
    31.9651310,
    34.8254362
    )

def on_collision(ego, ped, contact):    
    name2 = "STATIC OBSTACLE" if ped is None else ped.name
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    hit_angle = ego.transform.rotation.y - math.degrees(math.atan2(ped.transform.position.x - ego.transform.position.x , ped.transform.position.z - ego.transform.position.z))%360
    if(hit_angle > 180 ):
        hit_angle -= 360
    print("\n---------- Collisin Detected : -----------------/\n")
    print("Hit Angle: "+ str(hit_angle))
    print("Cart collided with "+name2+ "at "+str(contact)+" Collision TimeStamp: " + current_time)

ego.on_collision(on_collision)


sim.run(600)