#!/usr/bin/env python3

from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import random
import time

from permutation_params import PermutationParams
import simulator_types
from scenarios_ros_thread import ScenariosRosThread 

scenario_num = 15
scenario_name = "Several pedestrian crossing WG-Bells."
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

    # add pedestrian

    ped_velocity = 0.9*randoms[4]

    # points in lgsvl-coordinates
    lat_lon_list = [
	simulator_types.lat_lon_2_map_ground(sim, 31.963948, 34.826225),
	simulator_types.lat_lon_2_map_ground(sim, 31.963815, 34.826267),
	simulator_types.lat_lon_2_map_ground(sim, 31.963687, 34.826267),
	simulator_types.lat_lon_2_map_ground(sim, 31.963653, 34.826296),
	simulator_types.lat_lon_2_map_ground(sim, 31.963687, 34.826331),
	simulator_types.lat_lon_2_map_ground(sim, 31.963831, 34.826321)]

    wp = [lgsvl.WalkWaypoint(lat_lon, 0, 0, ped_velocity) for lat_lon in lat_lon_list]    
    
    ped = sim.add_agent(simulator_types.pedestrian_types[0], lgsvl.AgentType.PEDESTRIAN, 
        lgsvl.AgentState(transform = Transform(position = lat_lon_list[0])))
    ped.follow([wp[0], wp[4], wp[0]], True)

    ped = sim.add_agent(simulator_types.pedestrian_types[1], lgsvl.AgentType.PEDESTRIAN, 
        lgsvl.AgentState(transform = Transform(position = lat_lon_list[1])))
    ped.follow([wp[1], wp[3], wp[1]], True)

    ped = sim.add_agent(simulator_types.pedestrian_types[2], lgsvl.AgentType.PEDESTRIAN, 
        lgsvl.AgentState(transform = Transform(position = lat_lon_list[2])))
    ped.follow([wp[2], wp[5], wp[2]], True)

    ped = sim.add_agent(simulator_types.pedestrian_types[3], lgsvl.AgentType.PEDESTRIAN, 
        lgsvl.AgentState(transform = Transform(position = lat_lon_list[3])))
    ped.follow([wp[3], wp[5], wp[3]], True)

    ped = sim.add_agent(simulator_types.pedestrian_types[4], lgsvl.AgentType.PEDESTRIAN, 
        lgsvl.AgentState(transform = Transform(position = lat_lon_list[4])))
    ped.follow([wp[4], wp[1], wp[4]], True)

    ped = sim.add_agent(simulator_types.pedestrian_types[5], lgsvl.AgentType.PEDESTRIAN, 
        lgsvl.AgentState(transform = Transform(position = lat_lon_list[5])))
    ped.follow([wp[5], wp[1], wp[5]], True)

    sim.run(2)

    # ped = sim.add_agent(simulator_types.pedestrian_types[6], lgsvl.AgentType.PEDESTRIAN, 
    #     lgsvl.AgentState(transform = Transform(position = lat_lon_list[0])))
    # ped.follow([wp[0], wp[1], wp[0]], True)

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