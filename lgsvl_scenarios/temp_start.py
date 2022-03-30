#!/usr/bin/env python3

import datetime
from geometry_msgs.msg import Point
from genericpath import exists
from operator import index
import random
import lgsvl
import recording_data
from lgsvl.geometry import Transform
import simulator_types
from sim_start import LounchSimulator
import geo_converter
 
# A bridge class between RosBag pedestrian Data and Lgsvl Pedestrian Data

class SimPedestrian:
    def __init__(self,pedestrian,id,index):
        self.pedestrian = pedestrian 
        self.id = id
        self.index = index

def convert(date_time):
    format = '%H:%M:%S' # The format
    datetime_str = datetime.datetime.strptime(date_time, format) 
    return datetime_str

def convert_sim_point(sim_point):
    world_point = Point()
    world_point.x = float(-sim_point.x)
    world_point.y = float(-sim_point.z)
    world_point.z = float(sim_point.y)
    return world_point

#recording_data.get_cart_path_at_timeStamp()
#time_stamp = convert(input("Enter a time stamp '(H:M:S)': "))
time_stamp = convert("15:17:23")
cart_position = recording_data.get_cartlocation_at_timestamp(time_stamp.time())
cart_rotation = recording_data.get_cart_yaw_at_timestamp(time_stamp.time())

print( "Cart created at: "+ str(cart_position))
print(str(cart_rotation))

cart_location_converted = convert_sim_point(cart_position)
cart_geo_location = recording_data.point_local2geo(cart_location_converted)
cart_dest = recording_data.get_cart_path()
print(str(cart_geo_location))
#print(str(cart_dest))

simulator = LounchSimulator("Test",simulator_types.map_types.GanBIvrit,cart_position,lgsvl.Vector(0,70,0))

existed_pedestrains = []

pedestrian_index =-1

simulator.ros_thread.send_drive_to_point(cart_location_converted.y,cart_location_converted.x,cart_dest.y,cart_dest.x)

while True:
    
    pedestrian_list = recording_data.get_pedestrains_from_timestamp(time_stamp)

    for msg_ped in pedestrian_list:

        #Check for pedestrian msgs clasifide as duplicated or flase detection and remove them 
        if msg_ped.remove and msg_ped.created and not msg_ped.wp_list:
            pedestrian_list.remove(msg_ped)
            for peds in existed_pedestrains:
                if peds.id == msg_ped.id:
                    simulator.sim.remove_agent(peds.pedestrian)
                    existed_pedestrains.remove(peds)

        # Create a new Pedestrain in the simulator 
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

        # match the pedestrian from the RosBag msg to the alredy created simulator pedestrian and asighn a waypoint to him
        else:
            for pedestrian in existed_pedestrains:
                if pedestrian.id == msg_ped.id and msg_ped.wp_list :
                    pedestrian.pedestrian.follow(msg_ped.wp_list)
                    msg_ped.wp_list.clear()

    time_stamp += datetime.timedelta(0,0.2)
    simulator.sim.run(0.2)

