#!/usr/bin/env python3

from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import random
import time

from permutation_params import PermutationParams
import simulator_types
from scenarios_ros_thread import ScenariosRosThread 

import sim_pois

scenario_num = 17
scenario_name = "Vehicle rides against WG-Bells."
scene = simulator_types.map_types.GanBIvrit

print('Scenario #' + str(scenario_num) + '. '  + scenario_name)
print(scene.name)

vehicle = simulator_types.ego_types.DefaultType
print('Vehicle: ' + vehicle.name + ' - ' + vehicle.value)

params = PermutationParams(scenario_num)
randoms = params.input_params()

if not randoms:
    randoms = [0.0,0.0,0.0,0.0,1.0]

ros_thread = ScenariosRosThread()
ros_thread.run(scenario_name)

env = Env()
sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))

for loop_index in range(params.loop_count):

    if params.loop_count > 1:       
        print(str(str(loop_index + 1)) + '.')

    if params.random_ego_start_pos:
        randoms[0] = random.uniform(-0.2,0.2) #right
        randoms[1] = random.uniform(-1,1) #forward
    if params.random_object_start_pos:
        randoms[2] = random.uniform(-3,0) #right
        randoms[3] = random.uniform(-2,3) #forward  
    if params.random_object_speed:
        randoms[4] = random.uniform(0.75,1.5) #speed factor

    if params.source == 'random': 
        params.save_scenario_randoms(randoms)

    if sim.current_scene == scene.value:
        sim.reset()
    else:
        sim.load(scene.value)

    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])

    zero_pose = simulator_types.lat_lon_2_map_ground(sim, 31.964901917347795, 34.82459038726001) # WG

    # add ego-vehicle
    state_ego = lgsvl.AgentState()
    state_ego.transform = Transform(position = zero_pose + randoms[0] * right + randoms[1] * forward, 
                                    rotation = lgsvl.Vector(0, 20, 0))

    ego = sim.add_agent(vehicle.value, lgsvl.AgentType.EGO, state_ego)

    # An EGO will not connect to a bridge unless commanded to
    print("Bridge connected:", ego.bridge_connected)

    # The EGO is now looking for a bridge at the specified IP and port
    ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST", lgsvl.wise.SimulatorSettings.bridge_host), env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port))

    print("Waiting for connection...")

    while not ego.bridge_connected:
        time.sleep(1)

    print("Bridge connected:", ego.bridge_connected)

    # add npcs

    # points in lgsvl-coordinates
    wp = sim_pois.WG_Bells_Path(sim)
    wp.reverse()
    wp = sim_pois.get_drive_points(wp, 1.8*randoms[4], state_ego.rotation.y)

    for i in range(6):   
        first_point_index = i * 10 
        if first_point_index < len(wp):
            npc_state = lgsvl.AgentState(
                    transform = Transform(
                        rotation = Vector( x = 0, y = wp[first_point_index].angle.y, z = 0), 
                        position = wp[first_point_index].position))  

            npc = simulator_types.add_random_npc(sim, npc_state)  

            if npc: 
                npc.follow(wp[first_point_index:])

    # points in carteav-db coordinates
    ros_thread.send_drive_to_point(
        31.965590, #31.96554412108018, # WG
        34.8247096, #34.82465359876652,
        31.963650, # MAZE
        34.825965)    

    while not ros_thread.scenarios_node.check_ended():
        sim.run(5)
    
sim.stop() 
ros_thread.stop()  