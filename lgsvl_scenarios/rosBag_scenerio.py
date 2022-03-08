#!/usr/bin/env python3

import rclpy
import lgsvl
from rclpy.node import Node
from carteav_interfaces.msg import MovingObjectTrackingList
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from lgsvl.geometry import Spawn, Transform, Vector
from temp import Recording

# objects
      
# To Do for Wendsday :
    # figure out isolating tracking object ID 
    # create a sharable variable for simulator scenario
    # add a subscriebr for cart loaction Node  

padastrinas_list = [] 

class TrackingObjectsNode(Node):
    def __init__(self):
        self.id_list = []
        self.padastrians = []
        self.obj_msg: MovingObjectTrackingList = MovingObjectTrackingList()
        self.recording = Recording()
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
                        if ped.id == msg.id and ped.position.x != msg.x : 
                            contains = True
                if not contains:
                    self.handle_data(pedastrian(msg.x,msg.y,msg.z,msg.id))
                    self.id_list.append(msg.id)      

    def handle_data(self,padastrian):
        global padastrinas_list
        padastrinas_list = padastrian
        self.padastrians.append(padastrian)
        #print("Pedestrains Position: X:" + str(padastrian.position.x)+" , Y:"+str(padastrian.position.y) + " Z:"+str(padastrian.position.z))
        #self.recording.get_obj_msg(self.padastrians)
       
    

class pedastrian():
    def __init__(self,x,y,z,id):
        self.position = lgsvl.Vector(-x,0,-y)
        self.id = id

    