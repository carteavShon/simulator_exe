#!/usr/bin/env python3

from environs import Env
from matplotlib import container
from numpy import add, record
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
import random
import time
import rclpy
from recording_ros_tread import RecordedScenerioThread
from scenarios_ros_thread import ScenariosRosThread
from tracking_node import TrackingObjectsNode
from permutation_params import PermutationParams
import simulator_types



scenario_num = 20
scenario_name = "FROM RECORDING"
scene = simulator_types.map_types.GanBIvrit

print('Scenrio #' + str(scenario_num)+ '. ' +scenario_name)

vehicle=simulator_types.ego_types.DefaultType
print('Vehicle: ' + vehicle.name + ' - ' + vehicle.value)

params = PermutationParams(scenario_num)
randoms = params.input_params()

# sec_ros_thread = ScenariosRosThread()
# sec_ros_thread.run(scenario_name)


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
forward = lgsvl.utils.transform_to_forward(spawns[1])
right = lgsvl.utils.transform_to_right(spawns[1])

zero_pose = spawns[1].position

ego_state = lgsvl.AgentState()
ego_state.transform = Transform(position = lgsvl.Vector(670.87,0,4779.94), 
                                    rotation = lgsvl.Vector(0,-90.24,0))

forward = lgsvl.utils.transform_to_forward(ego_state)
right = lgsvl.utils.transform_to_right(ego_state)

ego = sim.add_agent(vehicle.value,lgsvl.AgentType.EGO,ego_state)

ego.connect_bridge(env.str("LGSVL_AUTOPILOT_0_HOST",lgsvl.wise.SimulatorSettings.bridge_host),env.int("LGSVL_AUTOPILOT_0_PORT",lgsvl.wise.SimulatorSettings.bridge_port))

print("Waiting For Connection ...")

while not ego.bridge_connected:
    time.sleep(1)

print("Bridge connected:", ego.bridge_connected)

## CONECTING TO MSG FROM ROS BAG

ros_thread = RecordedScenerioThread()
ros_thread.run()
padastrians_list = [] 


## CHECK FOR DUPLICATED PEDESTRIANS POSIBILITY 

def containes_pedestrian(pedestrian1,pedestrian2):
    if round(pedestrian1.position.z) == round(pedestrian2.transform.position.z) and round(pedestrian1.position.x) == round(pedestrian2.transform.position.x):
        return True
    return False

## ADING PEDESTRIAN TO SIMULATOR

def add_pedastrian(pedastrian):
    global padastrians_list
    ped_state = lgsvl.AgentState()
    ped_state.transform = Transform(position=pedastrian.position , rotation=lgsvl.Vector(0,270,0))
    ped = sim.add_agent(random.choice(simulator_types.pedestrian_types),lgsvl.AgentType.PEDESTRIAN,ped_state)
    padastrians_list.append(ped)
    print("Created pedastrain at: Z: " +str(pedastrian.position.z)+"--- X: " +str(pedastrian.position.x))

## CHECK TO SEE IF THE PEDESTRIANS LIST FROM ROS MSG HAS GROWN COMPARE TO UNITY PEDESTRIANS

def pedestrians_list_changed(padastrians,msg_pedestrians):
    if padastrians.__len__() != msg_pedestrians.__len__():
        return True

## HANDLE THE DATA COMING FROM OBJECT TRACKING NODE  

def handle_pedestrians():
    for ped in ros_thread.scenarios_node.padastrians:
        contains = False
        if padastrians_list:
            for temp_ped in padastrians_list:   
                if containes_pedestrian(ped,temp_ped):
                    contains =True
        else:
            add_pedastrian(ped)
            contains =True 
        if not contains:
            add_pedastrian(ped)


while True:
    if pedestrians_list_changed(padastrians_list,ros_thread.scenarios_node.padastrians):
        handle_pedestrians()
    else:
        sim.run(0.5)



        
