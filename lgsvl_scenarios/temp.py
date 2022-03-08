#!/usr/bin/env python3
from typing import List
import rclpy
from rclpy.node import Node
from rosBag_scenerio import MovingObjectTrackingList

class Vector3():
    def __init__(self,x:float=0.0,y:float=0.0,z:float=0.0):
        self.x = x
        self.y = y
        self.z = z 

class Pedastrian():
    def __init__(self,x,y,z,id):
        self.position = Vector3(x,y,z)
        self.id = id


class Recording:

    def __init__(self):
        self.obj_msg: MovingObjectTrackingList = MovingObjectTrackingList()
        self.cartLocation = None
        self.padastriansList = []

    def get_obj_msg(self , padastrians):
        self.padastriansList = padastrians


