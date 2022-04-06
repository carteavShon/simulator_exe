#!/usr/bin/env python3

import lgsvl
from rclpy.node import Node
from carteav_interfaces.msg import MovingObjectTrackingList
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from lgsvl.geometry import Spawn, Transform, Vector



padastrinas_list = [] 

class TrackingObjectsNode(Node):
    def __init__(self):
        self.id_list = []
        self.padastrians = []
        self.obj_msg: MovingObjectTrackingList = MovingObjectTrackingList()
        besteffort_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        super().__init__("tracking_objects")
        self.subscriber = self.create_subscription(MovingObjectTrackingList , "tracking_objects",self.objects_callback,besteffort_qos)
        
    def objects_callback(self, obj_msg: MovingObjectTrackingList):
        contains = False
        for msg in obj_msg.objects:
            if msg.object_type == 0:
                if not self.padastrians:
                    self.handle_data(pedastrian(msg.x,msg.y,msg.z,msg.id))
                    contains = True
                else:
                    for ped in self.padastrians:
                        if self.is_same_pedestrian(ped,msg): 
                            contains = True
                if not contains:
                    self.handle_data(pedastrian(msg.x,msg.y,msg.z,msg.id))
                    self.id_list.append(msg.id)      

    def is_same_pedestrian(self,pedestrian1,msg_ped):
        if round(pedestrian1.position.z) == round(msg_ped.z) and round(pedestrian1.position.x) == round(msg_ped.x):
            if(pedestrian1.id == msg_ped.id):
                return True
        return False   

    def handle_data(self,padastrian):
        global padastrinas_list
        self.padastrians.append(padastrian)
        padastrinas_list = padastrian
        #print("Pedestrains Position: X:" + str(padastrian.position.x)+" , Y:"+str(padastrian.position.y) + " Z:"+str(padastrian.position.z))
        #self.recording.get_obj_msg(self.padastrians)
    
    

class pedastrian():
    def __init__(self,x,y,z,id):
        self.position = lgsvl.Vector(-x,0,-y)
        self.id = id

    