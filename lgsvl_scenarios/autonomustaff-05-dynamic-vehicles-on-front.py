#!/usr/bin/env python3

#!/usr/bin/env python3

from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import random
import time

from permutation_params import PermutationParams
import simulator_types
from scenarios_ros_thread import ScenariosRosThread 

scenario_num = 5
scenario_name = "Dynamic vehicles on front"
scene = simulator_types.map_types.AutonomouStuff

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
        randoms[0] = random.uniform(-0.2,0.2) #rights
        randoms[1] = random.uniform(-1,1) #forward
    if params.random_object_start_pos:
        randoms[2] = random.uniform(-1,1) #right
        randoms[3] = random.uniform(-1,1) #forward  
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

    zero_pose = simulator_types.lat_lon_pose_2_map_ground(sim, simulator_types.positions.start_lat_lon_unity)

    # add ego-vehicle
    state_ego = lgsvl.AgentState()
    state_ego.transform = Transform(position = zero_pose + randoms[0] * right + randoms[1] * forward, 
                                    rotation = simulator_types.positions.start_rotation_unity)

    # set directions by ego
    forward = lgsvl.utils.transform_to_forward(state_ego)
    right = lgsvl.utils.transform_to_right(state_ego)

    # state_ego.velocity = 4 * forward

    ego = sim.add_agent(vehicle.value, lgsvl.AgentType.EGO, state_ego)

    # An EGO will not connect to a bridge unless commanded to
    print("Bridge connected:", ego.bridge_connected)

    # The EGO is now looking for a bridge at the specified IP and port
    ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST", lgsvl.wise.SimulatorSettings.bridge_host), env.int("LGSVL__AUTOPILOT_0_PORT", lgsvl.wise.SimulatorSettings.bridge_port))

    print("Waiting for connection...")

    while not ego.bridge_connected:
        time.sleep(1)

    print("Bridge connected:", ego.bridge_connected)

    def add_vehicle_on_front(state_ego):

        forward = lgsvl.utils.transform_to_forward(state_ego)
        right = lgsvl.utils.transform_to_right(state_ego)

        point = state_ego.position + (40 * forward) - (2.5 * right) + randoms[2] * right + randoms[3] * forward # sim.map_point_on_lane(point)

        state = lgsvl.AgentState(
            transform = Transform(
                rotation = Vector( x= state_ego.rotation.x, y = state_ego.rotation.y - 180, z= state_ego.rotation.z), 
                position = point))

        speed = 5 * randoms[4]

        npc = simulator_types.add_random_npc(sim, state)  

        # if npc: 
        #     npc.follow_closest_lane(True, speed)
        #     return True
        # else:
        #     return False

        # npc.on_waypoint_reached(on_waypoint)
        rotation = state_ego.transform.rotation.y - 180.0
        wp = [lgsvl.DriveWaypoint(position=state.position, speed=speed/2, angle=Vector(0.0,rotation,0.0)), 
                lgsvl.DriveWaypoint(position=state.position - 80*forward - 1.2*right, speed=speed, angle=Vector(0.0,rotation,0.0))]
        npc.follow(wp)

        return True

    def drive_to_point():
        ros_thread.send_drive_to_point(
            simulator_types.positions.start_lat_lon_unity.latitude, 
            simulator_types.positions.start_lat_lon_unity.longitude, 
            simulator_types.positions.dest_lat_lon_unity.latitude, 
            simulator_types.positions.dest_lat_lon_unity.longitude)    

        prev_state = None
        end = False
        for n in range(15):
            if (prev_state != ego.state):
                if add_vehicle_on_front(ego.state):
                    print(str(n+1) + " vehicle added on front")
                else:
                    break
            sim.run(5)
            end = ros_thread.scenarios_node.check_ended()
            if end:
                break

        while not ros_thread.scenarios_node.check_ended() :
        # while True:    
            sim.run(5)       

    def run_endless():
        prev_state = None
        n = 0
        while True:
            if (prev_state != ego.state):
                if add_vehicle_on_front(ego.state):
                    print(str(n+1) + " vehicle added on front")
                    n = n+1
                else:
                    break
            sim.run(10)    

    drive_to_point()
    # run_endless()

    sim.stop() 

ros_thread.stop()  



