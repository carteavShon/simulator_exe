#!/usr/bin/env python3
import math
from os import remove
import sys
from typing import List
import lgsvl
from geo_converter import geo_converter_instance, geoConverter
import datetime
sys.path.append('/home/shon/git_repos/carteav_tools')
from bags.bag_parse_2 import *

class Pedestrian():
    def __init__(self,x,y,z,id,heading,removeable):
        self.position = lgsvl.Vector(-x,0,-y)
        self.id = id
        y = math.degrees(heading)
        angle = (((-y)*(0.466))%360)
        self.heading = lgsvl.Vector(0,math.degrees(angle),0)
        self.wp_list = []
        self.created = False
        self.remove = removeable
        self.index = 0

# print("Pleas Enter The Recording File Path: " )
# rec_dir = input() 
rec_dir = '/home/shon/Videos/0015-cartbag-cart-1-28_3_22-15_17_9-Mission/0015-cartbag-cart-1-28_3_22-15_17_9-Mission_0.db3'

print("\n Loading Data From File ... \n ")

bagparser = BagFileParser(rec_dir)


def get_cart_path_data(df):
    data_dict = {datetime.datetime.fromtimestamp(index//1000000000): row \
                for index, row in df}
    return data_dict

def get_object_list_data(df):
    data_dict = {datetime.datetime.fromtimestamp(index//1000000000): row.objects \
                for index, row in df}

    return data_dict

def get_cart_location(df):
    data_dict = {datetime.datetime.fromtimestamp(index//1000000000): row \
                for index, row in df}
    return data_dict

def get_cart_yaw(df):
    data_dict = {datetime.datetime.fromtimestamp(index//1000000000): row.cart_yaw \
                for index, row in df}
    return data_dict

## CART LOCATION DATA FRAMES
cart_location_df=bagparser.get_messages("/cart_location")
cart_location_data = get_cart_location(cart_location_df)
cart_yaw_data = get_cart_yaw(cart_location_df)
cart_path_data = bagparser.get_messages("/cart_path")

## TRACKED OBJECT DATA FRAMES
tracking_obj_data = get_object_list_data(bagparser.get_messages("/tracking_objects"))

print("Loading Complete!\n")

def get_cartlocation_at_timestamp(time):
    try:
        for timestamp , msg in cart_location_data.items():
            if time == timestamp.time():
                return lgsvl.Vector(-msg.pose.position.x,0,-msg.pose.position.y)
    except:
        print("No Time Stamp Found ...")

def get_cart_yaw_at_timestamp(time):
    try:
        for time_stamp , yaw in cart_yaw_data.items():
            if time == time_stamp.time():
                linear_transform = 0.466
                y = math.degrees(yaw)
                angle = (((-y)*(-linear_transform)))
                return lgsvl.Vector(0,angle,0)
    except:
        print("No Time Stamp Found ...")



def get_cart_path(): 
    cart_point = cart_path_data[0][1].points[-1].point
    cart_lat_lon = point_local2geo(cart_point)
    return cart_lat_lon

def point_local2geo(point):
    convert = geoConverter(31.97171990, 34.77550870, 0)
    lat_lon_alt_list = convert.convertToGeo([
                point.x, 
                point.y, 
                point.z]) 
    point.x = float(lat_lon_alt_list[1]) # lon
    point.y = float(lat_lon_alt_list[0]) # lat
    point.z = float(lat_lon_alt_list[2])  
    
    return point

# Checks to see if the pedestrian from the messeg has allredy been added to avoid duplication 
# If it is the same than asign his new position as a walking waypoint 

def is_same_pedestrian(pedestrian1: Pedestrian,msg_ped):
    if (round(pedestrian1.position.z) == -math.ceil(msg_ped.y) or math.ceil(pedestrian1.position.x) == -round(msg_ped.x)) or pedestrian1.id == msg_ped.id:
        if pedestrian1.id == msg_ped.id:
            if pedestrian1.wp_list:
                pedestrian1.wp_list.clear()
            pedestrian1.wp_list.append(lgsvl.WalkWaypoint(lgsvl.Vector(-msg_ped.x,0,-msg_ped.y),0,speed=1))
        return True  
    return False  

pedestrians_list = []

def is_removeable(pedestrian_list:List , msg_pedestrian_list):
    for msg_ped in msg_pedestrian_list:
        for ped in pedestrian_list:
            if msg_ped.id == ped.id:
                ped.remove = False
            else:
                ped.remove = True


def get_pedestrains_from_timestamp(start_time):

    '''
    Colecting data each secound to get real time pedestrians position and behavior.

    params: start_time => the start time-stamp (datetime) you wish to get data from.

    *Function Must Be In a Loop*   
    '''

    global pedestrians_list
    msg_pedestrian_list = []

    for time_stamp, objects_list in tracking_obj_data.items():
        if start_time.time() <= time_stamp.time() and time_stamp.time() <= (start_time + datetime.timedelta(0,1)).time():
            for obj_data in objects_list:
                if obj_data.object_type == 0: # Type = 0 means its a pedestrian
                    msg_pedestrian_list.append(obj_data)
                    
    for obj_data in msg_pedestrian_list:
        contains = False

        if not pedestrians_list:
            pedestrians_list.append(Pedestrian(obj_data.x,obj_data.y,obj_data.z,obj_data.id,obj_data.heading,removeable=True))
            contains =True

        else:
            for ped in pedestrians_list:
                if is_same_pedestrian(ped,obj_data):
                    contains = True  

        if not contains:
            pedestrians_list.append(Pedestrian(obj_data.x,obj_data.y,obj_data.z,obj_data.id,obj_data.heading,removeable=True))
    
    is_removeable(pedestrians_list,msg_pedestrian_list)

    return pedestrians_list
                    