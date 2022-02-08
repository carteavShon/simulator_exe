#!/usr/bin/env python3

from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import random
import time

from permutation_params import PermutationParams
import simulator_types
from scenarios_ros_thread import ScenariosRosThread 

scenario_num = 12
scenario_name = "Blocked path at “Letters”"
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

    if params.source == 'random': 
        params.save_scenario_randoms(randoms)

    if sim.current_scene == scene.value:
        sim.reset()
    else:
        sim.load(scene.value)

    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])

#    zero_pose = spawns[0].position # simulator_types.lat_lon_pose_2_map_ground(sim, simulator_types.positions.start_lat_lon_ganheb)
    zero_pose = simulator_types.lat_lon_2_map_ground(sim, 31.964114539658521, 34.826100375706545)

    # add ego-vehicle
    state_ego = lgsvl.AgentState()
    state_ego.transform = Transform(position = zero_pose + randoms[0] * right + randoms[1] * forward, 
                                    rotation = simulator_types.positions.start_rotation_ganheb_back)

    # set directions by ego
    # forward = lgsvl.utils.transform_to_forward(state_ego)
    # right = lgsvl.utils.transform_to_right(state_ego)

    ego = sim.add_agent(vehicle.value, lgsvl.AgentType.EGO, state_ego)

    # An EGO will not connect to a bridge unless commanded to
    print("Bridge connected:", ego.bridge_connected)

    # The EGO is now looking for a bridge at the specified IP and port
    ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST", lgsvl.wise.SimulatorSettings.bridge_host), env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port))

    print("Waiting for connection...")

    while not ego.bridge_connected:
        time.sleep(1)

    print("Bridge connected:", ego.bridge_connected)

    # add npc

    # points in lgsvl-coordinates
    block_at_pos = simulator_types.lat_lon_2_map_ground(sim, 31.9642068, 34.82615)

    npc_state = lgsvl.AgentState(
            transform = Transform(
                rotation = Vector( x= state_ego.rotation.x, y = state_ego.rotation.y - 180, z= state_ego.rotation.z), 
                position = block_at_pos))    

    npc = simulator_types.add_random_npc(sim, npc_state)

    sim.run(2)

    # points in carteav-db coordinates
    ros_thread.send_drive_to_point(
        31.96482510432022,
        34.826237722448006, 
        31.96510477783623, 
        34.82641025929133)    

    while not ros_thread.scenarios_node.check_ended():
        sim.run(5)
    
sim.stop() 
ros_thread.stop()  