#!/usr/bin/env python3

from ossaudiodev import control_labels
from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import random
import time

from permutation_params import PermutationParams
import simulator_types
from scenarios_ros_thread import ScenariosRosThread 

cordinates = simulator_types.ganBivrit_spawn_cordinates

scenario_num = 18
scenario_name = "Walk In The Park"
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

if not randoms:
    randoms = [0.0,0.0,0.0,0.0,1.0]

for index in range(params.loop_count):
    if params.random_ego_start_pos:
        randoms[0]= random.uniform(-0.2,0.2)
        randoms[1]= random.uniform(-1,1)
    if params.random_object_start_pos:
        randoms[2] = random.uniform(-3,0)
        randoms[3] = random.uniform(-2,3)
    if params.random_object_speed:
        randoms[4] = random.uniform(0.75,1.5)

    if params.source == 'ranmdom':
        params.save_scenario_randoms(randoms)
    
    if sim.current_scene == scene.value:
        sim.reset()
    else:
        sim.load(scene.value)

spawns = sim.get_spawn()
forward = lgsvl.utils.transform_to_forward(spawns[3])
right = lgsvl.utils.transform_to_right(spawns[3])

zero_pose = spawns[3].position

ego_state = lgsvl.AgentState()
ego_state.transform = Transform(position = zero_pose + randoms[0] * right + randoms[1] * forward, 
                                    rotation = lgsvl.Vector(0,180,0))

forward = lgsvl.utils.transform_to_forward(ego_state)
right = lgsvl.utils.transform_to_right(ego_state)

ego = sim.add_agent(vehicle.value,lgsvl.AgentType.EGO,ego_state)

ego.connect_bridge(env.str("LGSVL_AUTOPILOT_0_HOST",lgsvl.wise.SimulatorSettings.bridge_host),env.int("LGSVL_AUTOPILOT_0_PORT",lgsvl.wise.SimulatorSettings.bridge_port))

print("Waiting For Connection ...")

while not ego.bridge_connected:
    time.sleep(1)

print("Bridge connected:", ego.bridge_connected)

sim.add_random_agents(lgsvl.AgentType.PEDESTRIAN)


def on_collision(ego, ped, contact):    
    name1 = "STATIC OBSTACLE" if ego is None else ego.name
    name2 = "STATIC OBSTACLE" if ped is None else ped.name
    print("{} collided with {} at {}".format(name1, name2, contact))

sim.run(2)

ego.on_collision(on_collision)

# Drive from Parking 1 to West Gate 
ros_thread.send_drive_to_point(
    31.9651178,
    34.8265818,
    31.9656669,
    34.8260782
)

while not ros_thread.scenarios_node.check_ended():
    sim.run(5)

   

print(" Drive from Parking 1 to West Gate ")
# drive from West Gate to cafe Gan Sipur
ros_thread.send_drive_to_point(
    31.9656669,
    34.8260782,
    31.9640471,
    34.8267438
)

print("Drive from West Gate to cafe Gan Sipur")
print("Ros Thread ID:" + str(ros_thread.ros_thread_id))
while not ros_thread.scenarios_node.check_ended():
    sim.run(3)


# drive from West Gate to cafe Gan Sipur
ros_thread.send_drive_to_point(
    31.9640471,
    34.8267438,
    31.9633875,
    34.8268809
)

print("Drive from 'Gan Sipur' to 'Back Park Yard' ")

while not ros_thread.scenarios_node.check_ended():
    sim.run(3)

sim.stop() 
ros_thread.stop() 



