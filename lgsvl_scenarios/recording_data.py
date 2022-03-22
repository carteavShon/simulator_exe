#!/usr/bin/env python3
import math
from operator import contains
import os
import sys,ament_index_python
from typing import List
from warnings import catch_warnings
from turtle import position
import lgsvl

import datetime
sys.path.append('/home/shon/git_repos/carteav_tools')
from bags.bag2reader import *

class Pedestrian():
    def __init__(self,x,y,z,id,heading,removeable):
        self.position = lgsvl.Vector(-x,0,-y)
        self.id = id
        y = math.degrees(heading)
        angle = (((-y)*(-33.013378086))%360)
        self.heading = lgsvl.Vector(0,math.degrees(angle),0)
        self.wp_list = []
        self.created = False
        self.remove = removeable
        self.index = 0

print("Pleas Enter The Recording File Path: " )
rec_dir = input() 
print("\n Loading Data From File ... \n ")
topics_dict = bag_to_dataframes(rec_dir, include_topics=[], verbose=False)
print(" Loading Complete!")
topics_dict['carteav_interfaces/msg/MovingObjectTrackingList']

def get_object_list_data(df):
    data_dict = {datetime.datetime.fromtimestamp(row["timestamp"]//1000000000): row["msgs"].objects \
                for index, row in df.iterrows()}

    return data_dict

def get_cart_location(df):
    data_dict = {datetime.datetime.fromtimestamp(row["timestamp"]//1000000000): row["msgs"].pose.position\
                for index, row in df.iterrows()}
    return data_dict

def get_cart_yaw(df):
    data_dict = {datetime.datetime.fromtimestamp(row["timestamp"]//1000000000): row["msgs"].cart_yaw\
                for index, row in df.iterrows()}
    return data_dict

## TRACKED OBJECT DATA FRAMES

tracking_obj_df = topics_dict['carteav_interfaces/msg/MovingObjectTrackingList']
tracking_obj_data = get_object_list_data(tracking_obj_df)

## CART LOCATION DATA FRAMES

cart_location_df = topics_dict['carteav_interfaces/msg/CartLocation']
cart_location_data = get_cart_location(cart_location_df)
cart_yaw_data = get_cart_yaw(cart_location_df)

def get_time_stamp():
    for time_stamp,cart_location in cart_location_data.items():
        return time_stamp.time()


def get_cartlocation_at_timestamp(time):
    try:
        for time_stamp , position in cart_location_data.items():
            if time == time_stamp.time():
                return lgsvl.Vector(-position.x,0,-position.y)
    except:
        print("No Time Stamp Found ...")

def get_cart_yaw_at_timestamp(time):
    try:
        for time_stamp , yaw in cart_yaw_data.items():
            if time == time_stamp.time():
                linear_transform = 33.013378086
                y = math.degrees(yaw)
                angle = (((-y)*(-linear_transform))%360)
                return lgsvl.Vector(0,angle,0)
    except:
        print("No Time Stamp Found ...")


def is_same_pedestrian(pedestrian1: Pedestrian,msg_ped):
    # print("Ped X: " + str(round(pedestrian1.position.x)) + "    Msg X :"  + str(-round(msg_ped.x)))
    # print("Ped Z: " + str(round(pedestrian1.position.z)) + "    Msg Z :"  + str(-round(msg_ped.y))+"\n"+"------------")
    if round(pedestrian1.position.z) == -math.ceil(msg_ped.y) or math.ceil(pedestrian1.position.x) == -round(msg_ped.x):
        return True
    if pedestrian1.id == msg_ped.id:
        if pedestrian1.wp_list:
            pedestrian1.wp_list.clear()
        pedestrian1.wp_list.append(lgsvl.WalkWaypoint(lgsvl.Vector(-msg_ped.x,0,-msg_ped.y),0,speed=1))
        return True
    return False  

pedestrians_list = []


def get_pedestrains_from_timestamp(start_time):
    global pedestrians_list
    for time_stamp, objects_list in tracking_obj_data.items():
        if start_time.time() <= time_stamp.time() and time_stamp.time() <= (start_time + datetime.timedelta(0,1)).time():
            for obj_data in objects_list:
                contains = False
                if obj_data.object_type == 0:
                    if not pedestrians_list:
                        pedestrians_list.append(Pedestrian(obj_data.x,obj_data.y,obj_data.z,obj_data.id,obj_data.heading,removeable=False))
                        contains =True
                    else:
                        for ped in pedestrians_list:
                            if is_same_pedestrian(ped,obj_data):
                                contains = True
                            if ped.id == obj_data.id:
                                ped.remove = False
                            else:
                                ped.remove = True
                    if not contains:
                        pedestrians_list.append(Pedestrian(obj_data.x,obj_data.y,obj_data.z,obj_data.id,-obj_data.heading,removeable=False))
        
    return pedestrians_list
                    