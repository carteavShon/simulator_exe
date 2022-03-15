#!/usr/bin/env python3

import datetime
import random
import lgsvl
import recording_data
from lgsvl.geometry import Transform
import simulator_types
from sim_start import LounchSimulator

## FOR TUSDAY:
    # function to add time to current timeStamp
    # get pedestrians in a While Loop 
    # get Cart Path 
    # get Pedestrians Path  

def convert(date_time):
    format = '%H:%M:%S' # The format
    datetime_str = datetime.datetime.strptime(date_time, format) 
    return datetime_str


time_stamp = convert("10:05:50")

cart_position = recording_data.get_cartlocation_at_timestamp(time_stamp.time())
cart_rotation = recording_data.get_cart_yaw_at_timestamp(time_stamp.time())

print(str(cart_position))
print(str(cart_rotation))

pedestrians_created =[]



simulator = LounchSimulator("Test",simulator_types.map_types.GanBIvrit,cart_position,lgsvl.Vector(0,70,0))
while True:
    pedestrian_list = recording_data.get_pedestrains_from_timestamp(time_stamp.time(),(time_stamp + datetime.timedelta(0,1)).time())
    for msg_ped in pedestrian_list:
        if not pedestrians_created.__contains__(msg_ped):
            ped_state = lgsvl.AgentState()
            ped_state.transform = Transform(position=msg_ped.position , rotation=msg_ped.heading)
            ped = simulator.sim.add_agent(random.choice(simulator_types.pedestrian_types),lgsvl.AgentType.PEDESTRIAN,ped_state)
            pedestrians_created.append(msg_ped)
            print("Pedestrains was created at: "+str(ped.transform.position))
            if(msg_ped.wp_list):
                ped.follow(msg_ped.wp_list)
            else:
                ped.transform.position = lgsvl.Vector(0,0,0) 
    simulator.sim.run(1)
    time_stamp+=datetime.timedelta(0,1)
