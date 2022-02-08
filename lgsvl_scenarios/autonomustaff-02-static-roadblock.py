#!/usr/bin/env python3

from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import random
import time

from permutation_params import PermutationParams
import simulator_types
from scenarios_ros_thread import ScenariosRosThread 

scenario_num = 2
scenario_name = "Static pedestrians road block"
scene = simulator_types.map_types.AutonomouStuff

print('Scenario #' + str(scenario_num) + '. '  + scenario_name)
print(scene.name)

vehicle = simulator_types.ego_types.PilotcaDefaultTyperPC4_With2D
print('Vehicle: ' + vehicle.name + ' - ' + vehicle.value)

params = PermutationParams(scenario_num)
randoms = params.input_params()

ros_thread = ScenariosRosThread()
ros_thread.run(scenario_name)

env = Env()
sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))

if not randoms:
    randoms = [0.0,0.0]

# N pedestriants
N = 7

for loop_index in range(params.loop_count):

    if params.loop_count > 1:       
        print(str(str(loop_index + 1)) + '.')    

    for i in  range(N*2):
        randoms.append(0.0)

    if params.random_ego_start_pos:
        randoms[0] = random.uniform(-0.2,0.2) #right
        randoms[1] = random.uniform(-1,1) #forward
    if params.random_object_start_pos:
        for i in range(2,N*2+2,2):
            randoms[i] = random.uniform(-0.2,0.2) #right
            randoms[i+1] = random.uniform(-1,1) #forward

    if params.source == 'random': 
        params.save_scenario_randoms(randoms)

    if sim.current_scene == scene.value:
        sim.reset()
    else:
        sim.load(scene.value)

    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])

    zero_pose = simulator_types.lat_lon_pose_2_map_ground(sim, simulator_types.positions.start_lat_lon_unity)

    # add ego-vehicle
    state_ego = lgsvl.AgentState()
    state_ego.transform = Transform(position = zero_pose + randoms[0] * right + randoms[1] * forward, 
                                    rotation = simulator_types.positions.start_rotation_unity)

    ego = sim.add_agent(vehicle.value, lgsvl.AgentType.EGO, state_ego)

    # An EGO will not connect to a bridge unless commanded to
    print("Bridge connected:", ego.bridge_connected)

    # The EGO is now looking for a bridge at the specified IP and port
    ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST", lgsvl.wise.SimulatorSettings.bridge_host), env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port))

    print("Waiting for connection...")

    while not ego.bridge_connected:
        time.sleep(1)

    print("Bridge connected:", ego.bridge_connected)

    # add pedestrians
    forward = lgsvl.utils.transform_to_forward(state_ego)
    right = lgsvl.utils.transform_to_right(state_ego)

    pedestriant_pos_arr = [Transform(rotation = state_ego.rotation * i, 
    position = zero_pose + 8.5*forward - 1.2*right*(i-1) + randoms[2+i] * right + randoms[2+i+1] * forward)
        for i in range(N)]

    def add_agent(pedestrian_type, transform):
        try: 
            sim.add_agent( pedestrian_type, lgsvl.AgentType.PEDESTRIAN, lgsvl.AgentState(transform = transform)) 
        except Exception as e:
            print('failed to add pedestrian ' + str(pedestrian_type) + ': ' + str(e))

    try:
        pedestriant_arr = [
            add_agent( random.choice(simulator_types.pedestrian_types), pedestriant_pos_arr[i]) 
            for i in range(N)]
    except Exception as e:
        print(e)      

    ros_thread.send_drive_to_point(
        simulator_types.positions.start_lat_lon_unity.latitude, 
        simulator_types.positions.start_lat_lon_unity.longitude, 
        simulator_types.positions.dest_lat_lon_unity.latitude, 
        simulator_types.positions.dest_lat_lon_unity.longitude)    

    while not ros_thread.scenarios_node.check_ended():
        sim.run(5)
    
sim.stop() 
ros_thread.stop()  