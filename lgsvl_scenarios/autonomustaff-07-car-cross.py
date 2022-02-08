#!/usr/bin/env python3

from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import random
import time

from permutation_params import PermutationParams
import simulator_types
from simulator_types import lat_lon_2_map_ground, lat_lon_pose_2_map_ground
from scenarios_ros_thread import ScenariosRosThread 

scenario_num = 7
scenario_name = "Car crosses, turns to the left or to the right"
scene = simulator_types.map_types.AutonomouStuff

print('Scenario #' + str(scenario_num) + '. '  + scenario_name)
print(scene.name)

vehicle = simulator_types.ego_types.DefaultType
print('Vehicle: ' + vehicle.name + ' - ' + vehicle.value)

params = PermutationParams(scenario_num)
randoms = params.input_params()

class Trajectory():
    def __init__(self):
        self.angles = [Vector(0.0,0.0,0.0), 
            Vector(0.0,0.0,0.0), 
            Vector(0.0,0.0,0.0)]
        self.positions = [Vector(0.0,0.0,0.0), 
            Vector(0.0,0.0,0.0), 
            Vector(0.0,0.0,0.0)]   

if not randoms:
    randoms = [0.0,0.0,0.0,0.0,1.0,0]

ros_thread = ScenariosRosThread()
ros_thread.run(scenario_name)

env = Env()
sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))

N = 6 # num of trajectories         

for loop_index in range(params.loop_count):
 
    if params.loop_count > 1:       
        print(str(str(loop_index + 1)) + '.')

    if params.random_ego_start_pos:
        randoms[0] = random.uniform(-1,1) #right
        randoms[1] = random.uniform(-1,1) #forward
    if params.random_object_start_pos:
        randoms[2] = random.uniform(-1,1) #right
        randoms[3] = random.uniform(-1,1) #forward 
    if params.random_object_speed:
        randoms[4] = random.uniform(0.5,2) #speed factor 
    if params.random_object_trajectory:
        randoms[5] = random.randint(0,N) #trajectory index 

    if params.source == 'random': 
        params.save_scenario_randoms(randoms)

    if sim.current_scene == scene.value:
        sim.reset()
    else:
        sim.load(scene.value)

    spawns = sim.get_spawn()
    forward = lgsvl.utils.transform_to_forward(spawns[0])
    right = lgsvl.utils.transform_to_right(spawns[0])

    zero_pose = lat_lon_pose_2_map_ground(sim, simulator_types.positions.start_lat_lon_unity)

    # add ego-vehicle
    state_ego = lgsvl.AgentState()
    state_ego.transform = Transform(position = zero_pose + randoms[0] * right + randoms[1] * forward, 
                                    rotation = simulator_types.positions.start_rotation_unity)

    ego_rotation = state_ego.transform.rotation

    # set directions by ego
    forward = lgsvl.utils.transform_to_forward(state_ego)
    right = lgsvl.utils.transform_to_right(state_ego)

    #state_ego.velocity = 5 * forward

    ego = sim.add_agent(vehicle.value, lgsvl.AgentType.EGO, state_ego)

    # An EGO will not connect to a bridge unless commanded to
    print("Bridge connected:", ego.bridge_connected)

    # The EGO is now looking for a bridge at the specified IP and port
    ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST", lgsvl.wise.SimulatorSettings.bridge_host), env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port))

    print("Waiting for connection...")

    while not ego.bridge_connected:
        time.sleep(1)

    print("Bridge connected:", ego.bridge_connected)

    trajectories = [Trajectory(),Trajectory(),Trajectory(),Trajectory(),Trajectory(),Trajectory()]

    # front to right
    trajectories[0].angles = [
        Vector( x= ego_rotation.x, y = ego_rotation.y + 180, z= ego_rotation.z),
        Vector( x= ego_rotation.x, y = ego_rotation.y + 180, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 180, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 135, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 90,  z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 90,  z= ego_rotation.z)]

    trajectories[0].positions = [
        lat_lon_2_map_ground(sim, 37.380981, -121.9095008) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.380912, -121.9094772) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.380868, -121.9094595) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808491, -121.9094418) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808425, -121.9093899) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808421, -121.9092399) + randoms[2] * right + randoms[3] * forward]

    # left to front
    trajectories[1].angles = [
        Vector( x= ego_rotation.x, y = ego_rotation.y + 90, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 90, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 90, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 45, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 0,  z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 0,  z= ego_rotation.z)]

    trajectories[1].positions = [
        lat_lon_2_map_ground(sim, 37.3808078, -121.9095841) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808318, -121.9094713) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.380844, -121.9094318) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808627, -121.9094147) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808927, -121.9094126) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3809997, -121.9094424) + randoms[2] * right + randoms[3] * forward]

    # right to back
    trajectories[2].angles = [
        Vector( x= ego_rotation.x, y = ego_rotation.y - 90, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y - 90, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y - 130, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y - 170, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 180,  z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 180,  z= ego_rotation.z)]

    trajectories[2].positions = [
        lat_lon_2_map_ground(sim, 37.3808862, -121.9093179) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808852, -121.9094005) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808679, -121.9094383) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808566, -121.9094489) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808210, -121.9094554) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3807328, -121.9094330) + randoms[2] * right + randoms[3] * forward]   

    # right to front
    trajectories[3].angles = [
        Vector( x= ego_rotation.x, y = ego_rotation.y - 90, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y - 40, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y - 10, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 0,  z= ego_rotation.z)]

    trajectories[3].positions = [
        lat_lon_2_map_ground(sim, 37.3808862, -121.9093179) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808852, -121.9094005) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.380896, -121.9094129) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3810592, -121.9094678) + randoms[2] * right + randoms[3] * forward]  

    # cross from left
    trajectories[4].angles = [
        Vector( x= ego_rotation.x, y = ego_rotation.y + 90, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y + 90, z= ego_rotation.z),
        Vector( x= ego_rotation.x, y = ego_rotation.y + 90, z= ego_rotation.z)]

    trajectories[4].positions = [
        lat_lon_2_map_ground(sim, 37.3808078, -121.9095841) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808425, -121.9093899) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.3808421, -121.9092399) + randoms[2] * right + randoms[3] * forward]

    # cross from right
    trajectories[5].angles = [
        Vector( x= ego_rotation.x, y = ego_rotation.y - 90, z= ego_rotation.z), 
        Vector( x= ego_rotation.x, y = ego_rotation.y - 90, z= ego_rotation.z)]

    trajectories[5].positions = [
        lat_lon_2_map_ground(sim, 37.3808862, -121.9093179) + randoms[2] * right + randoms[3] * forward,
        lat_lon_2_map_ground(sim, 37.380844, -121.9095876) + randoms[2] * right + randoms[3] * forward]

    speed = 6 * randoms[4]

    # if only one trajectory
    # random_trajectory = trajectories[randoms[5]]
    random_trajectory = trajectories[random.randint(0,N-1)]
    # # trajectories = [random_trajectory]
    trajectories.append(random_trajectory)

    ros_thread.send_drive_to_point(
        simulator_types.positions.start_lat_lon_unity.latitude, 
        simulator_types.positions.start_lat_lon_unity.longitude, 
        simulator_types.positions.dest_lat_lon_unity.latitude, 
        simulator_types.positions.dest_lat_lon_unity.longitude)    

    for trajectory in trajectories:
        if (trajectory.positions == None):
            continue

        #build waypoint list
        waypoints = []
        for i in range(len(trajectory.positions)):

            # Raycast the points onto the ground because BorregasAve is not flat
            # pose = simulator_types.pose_2_ground(sim, trajectory.positions[i])
            # wp = lgsvl.DriveWaypoint(pose, speed, trajectory.angles[i], 0, 0)

            wp = lgsvl.DriveWaypoint(trajectory.positions[i], speed, trajectory.angles[i], 0, 0)
            
            if wp:
                waypoints.append(wp)
            else:
                print("convert to DriveWaypoint failed: " + str(trajectory.positions[i]))

        # add vehicle
        state = lgsvl.AgentState(transform = Transform(position = trajectory.positions[0]))

        npc = simulator_types.add_random_npc(sim, state)  

        if npc: 
            npc.follow(waypoints)
            sim.run(3)

    while not ros_thread.scenarios_node.check_ended():
        sim.run(5)
    
sim.stop()
ros_thread.stop()   



