#!/usr/bin/env python3

from environs import Env
import lgsvl
import math
from lgsvl.geometry import Transform
import random
import time
from permutation_params import PermutationParams
import simulator_types
from scenarios_ros_thread import ScenariosRosThread 
from datetime import datetime




now = datetime.now()
current_time = now.strftime("%H:%M:%S")

scenario_num = 19
scenario_name = "Randome Padestrain Angle of Attack"
scene = simulator_types.map_types.GanBIvrit

print("Scenerio Name: "+scenario_name+" Number #"+str(scenario_num))

vehicle = simulator_types.ego_types.DefaultType

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
forward = lgsvl.utils.transform_to_forward(spawns[0])
right = lgsvl.utils.transform_to_right(spawns[0])

zero_pose = spawns[0].position

ego_state = lgsvl.AgentState()
ego_state.transform = Transform(position = zero_pose + randoms[0] * right + randoms[1] * forward, 
                                    rotation = simulator_types.positions.start_rotation_ganheb)

forward = lgsvl.utils.transform_to_forward(ego_state)
right = lgsvl.utils.transform_to_right(ego_state)

ego = sim.add_agent(vehicle.value ,lgsvl.AgentType.EGO , ego_state)

print("Connecting To Bridge ...")

ego.connect_bridge(env.str("LGSVL__AUTOPILOT_0_HOST",lgsvl.wise.SimulatorSettings.bridge_host) ,env.int("LGSVL__AUTOPILOT_0_PORT",lgsvl.wise.SimulatorSettings.bridge_port))

while not ego.bridge_connected:
    time.sleep(1)

print("Bridge Connected :"+ str(ego.bridge_connected))

radius = 10


def create_ped(ego_state,angle):

    # pedestrain spwan positions is in a given radius in a circule around the Cart  

    ped_position = simulator_types.position_from_cart(ego_state,angle,radius)

    # pedestrian facing the cart 

    ped_rotation = simulator_types.azimuth_to_cart(ego_state,ped_position)
    ped_state = lgsvl.AgentState()
    ped_state.transform = Transform(position = ped_position , rotation = ped_rotation)
    ped = sim.add_agent(random.choice(simulator_types.pedestrian_types),lgsvl.AgentType.PEDESTRIAN,ped_state)
    walk_speed = 2*randoms[4]

    # pedestrian calculated Vector for colision with moving Cart 

    vector_infront_cart = lgsvl.Vector(x = ego_state.position.x + (math.sin(math.radians(ego_state.rotation.y))*4) ,y=0 ,z = ego_state.position.z + (math.cos(math.radians(ego.state.rotation.y))*4))
    wp = [lgsvl.WalkWaypoint( vector_infront_cart ,0,0,walk_speed),lgsvl.WalkWaypoint(ped_position + (10*forward) , 0,0,walk_speed) ]
    ped.follow(wp)
    print("Pedestrain name: "+ ped.name +". Pedestrian angle from car: " + str(ped.state.rotation.y))
    return ped_position

def drive_to_point():
    ros_thread.send_drive_to_point(
        simulator_types.positions.start_lat_lon_ganheb.latitude, 
        simulator_types.positions.start_lat_lon_ganheb.longitude, 
        simulator_types.positions.dest_lat_lon_ganheb.latitude, 
        simulator_types.positions.dest_lat_lon_ganheb.longitude
    )
    for num in range(20):
        if(ego.state.speed >2 ):
            angle = random.randint(int(ego.state.rotation.y -50), int(ego.state.rotation.y + 50)) 
        else:
            angle = random.randint(int(ego.state.rotation.y -130), int(ego.state.rotation.y + 130))

        if create_ped(ego.state,angle):
            print("Cart Speed in M/S: "+str(ego.state.speed)) 
        else:
            break
        sim.run(6)

 

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
drive_to_point()
sim.run(600)
