#!/usr/bin/env python3

import datetime
from genericpath import exists
from operator import index
import random
import lgsvl
import recording_data
from lgsvl.geometry import Transform
import simulator_types
from sim_start import LounchSimulator
 

class SimPedestrian:
    def __init__(self,pedestrian,id,index):
        self.pedestrian = pedestrian 
        self.id = id
        self.index = index

def convert(date_time):
    format = '%H:%M:%S' # The format
    datetime_str = datetime.datetime.strptime(date_time, format) 
    return datetime_str

#time_stamp = convert(input("Enter a time stamp '(H:M:S)': "))
time_stamp = convert(input("Pleas Enter a Starting TimeStamp (format: H:M:S): "))
cart_position = recording_data.get_cartlocation_at_timestamp(time_stamp.time())
cart_rotation = recording_data.get_cart_yaw_at_timestamp(time_stamp.time())

print( "Cart created at: "+ str(cart_position))
print(str(cart_rotation))

simulator = LounchSimulator("Test",simulator_types.map_types.GanBIvrit,cart_position,cart_rotation)

existed_pedestrains = []

pedestrian_index =-1

def on_waypoint(agent, index):
      print("Agent: " + str(agent) + "Index: " +str(index))
      

while True:

    pedestrian_list = recording_data.get_pedestrains_from_timestamp(time_stamp)

    for msg_ped in pedestrian_list:

        if msg_ped.remove and msg_ped.created and not msg_ped.wp_list:
            pedestrian_list.remove(msg_ped)
            for peds in existed_pedestrains:
                if peds.id == msg_ped.id:
                    simulator.sim.remove_agent(peds.pedestrian)
                    existed_pedestrains.remove(peds)
        if not msg_ped.created:
            ped_state = lgsvl.AgentState()
            ped_state.transform = Transform(position=msg_ped.position , rotation=msg_ped.heading)
            ped = simulator.sim.add_agent(random.choice(simulator_types.pedestrian_types),lgsvl.AgentType.PEDESTRIAN,ped_state)
            msg_ped.created = True
            pedestrian_index += 1
            msg_ped.index = pedestrian_index
            existed_pedestrains.append(SimPedestrian(ped,msg_ped.id,msg_ped.index))
            if(msg_ped.wp_list):
                ped.follow(msg_ped.wp_list)

        else:
            for pedestrian in existed_pedestrains:
                i =0 
                if pedestrian.id == msg_ped.id and msg_ped.wp_list :
                    pedestrian.pedestrian.follow(msg_ped.wp_list)
                    msg_ped.wp_list.clear()
                    pedestrian.pedestrian.on_waypoint_reached(on_waypoint)

                   
            
    time_stamp += datetime.timedelta(0,1)
    simulator.sim.run(1)

