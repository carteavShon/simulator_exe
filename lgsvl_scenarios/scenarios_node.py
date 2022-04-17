#!/usr/bin/env python3

from rclpy.node import Node
from rclpy.task import Future
from rclpy.qos import QoSProfile, DurabilityPolicy,HistoryPolicy, ReliabilityPolicy
from geometry_msgs.msg import Point
from rcl_interfaces.msg import Log
from carteav_interfaces.srv import CalculatePath, DrivingCommand as DrivingCommandSrv
from carteav_interfaces.msg import VehicleStateData, CartStatus, PathStatus, DrivingCommand as DrivingCommandMsg

import threading
import math

import ament_index_python, sys
sys.path.append(ament_index_python.packages.get_package_share_directory('common_pkg'))
import msg_names, carteav_log
from service_client_util import ServiceClient

from enum import Enum
class EnumDriveToDestResult(Enum):
    NONE = "None"
    INIT = "Init"
    ONPATH = "On_Path"
    FINISHED = "Finished"
    FAILED = "Failed"

class ScenariosNode(Node):

    def __init__(self, scenario_name: str):

        super().__init__('scenarios_node', allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        # params
        self.consts = {}
        self.init_params()

        self.scenario_name = scenario_name

        # services clients - send requests to Cart Manager
        self.drive_to_point_client = ServiceClient(self, self.create_client(CalculatePath, msg_names.SimDriveToPointSrv), self.drive_to_point_service_response)
        self.driving_command_client = ServiceClient(self, self.create_client(DrivingCommandSrv, msg_names.DrivingCommandSrv), None)     

        # for AUTO_DRIVE command
        self.vehicle_state_data_command_pub = self.create_publisher(VehicleStateData, msg_names.VehicleStateDataCommandTopic, 10)

        # cart status (check eta and path finished status)
        self.cart_status_sub = self.create_subscription(CartStatus, msg_names.CartStatusTopic, self.cart_status_callback, 1)
        self.cart_status = None

        # log
        self.log_pub = self.create_publisher(Log, msg_names.CarteavLogTopic, 10) 

        # fire events
        self.drive_to_dest_result = EnumDriveToDestResult.NONE 

        self.path_timeout_timer = None     
        self.timeout_sec = 0  

        self.old_path_id = ''

    def init_params(self):
        pass

    def cart_status_callback(self, cart_status_data: CartStatus): 

        #print("cart_status received")

        self.cart_status = cart_status_data

        if self.drive_to_dest_result == EnumDriveToDestResult.NONE:
            self.old_path_id = cart_status_data.mission_status.path_id

        if (self.drive_to_dest_result == EnumDriveToDestResult.INIT 
            and cart_status_data.mission_status.path_id != self.old_path_id 
            #and cart_status_data.mission_status.current_eta_sec > 0
            ):
                self.drive_to_dest_result = EnumDriveToDestResult.ONPATH

                print('ETA = ' + str(cart_status_data.mission_status.current_eta_sec))

                self.set_timeout_timer(int(4*cart_status_data.mission_status.current_eta_sec) if cart_status_data.mission_status.current_eta_sec > 0 else 1)              
        
        if self.drive_to_dest_result == EnumDriveToDestResult.ONPATH:
            if cart_status_data.mission_status.path_status in [PathStatus.ERROR]:
                self.drive_to_dest_result = EnumDriveToDestResult.FAILED
            if cart_status_data.mission_status.path_status == PathStatus.FINISHED:
                self.drive_to_dest_result = EnumDriveToDestResult.FINISHED

    def set_timeout_timer(self, sec: float):
        self.timeout_sec = sec
        if self.path_timeout_timer:
            self.path_timeout_timer.cancel()  
        self.path_timeout_timer = threading.Timer(self.timeout_sec, self.path_timeout_timer_tick)
        self.path_timeout_timer.start()        

    def path_timeout_timer_tick(self):
        if self.drive_to_dest_result != EnumDriveToDestResult.FINISHED:                   
            print("Timeout: " + str(self.timeout_sec) + " sec")
            self.drive_to_dest_result = EnumDriveToDestResult.FAILED        
        self.SendStop()

    def ended(self):
        return self.drive_to_dest_result == EnumDriveToDestResult.FAILED or self.drive_to_dest_result == EnumDriveToDestResult.FINISHED

    def drive_to_point_service_response(self, response: CalculatePath.Response):
        if not response or not response.is_path_found:
            self.drive_to_dest_result = EnumDriveToDestResult.FAILED
            
    def SendDriveToPoint(self, lat1, lon1, lat2, lon2):
        from permutation_params import PermutationParams
        carteav_log.publish_log(self, carteav_log.LogLevel.INFO, carteav_log.LogSubject.Simulator, 'Start Scenario', self.scenario_name + ', ' + PermutationParams.scenario_filename)

        request = CalculatePath.Request()
        request.source_point = Point(x=lon1, y=lat1)
        # Todo: get from location the yaw:
        print("\n"+str(self.cart_status.cart_location.cart_yaw)+"\n")
        print("\n"+str((math.degrees(self.cart_status.cart_location.cart_yaw)-90))+"\n")

        request.source_azimuth =(math.degrees(self.cart_status.cart_location.cart_yaw)-90)
        request.destination_point = Point(x=lon2, y=lat2)
        request.calculate_critiria = CalculatePath.Request.SHORTEST_PATH_CRITERIA

        if not self.drive_to_point_client.post_request(request):
            print("send CalculatePath failed")  
            self.drive_to_dest_result = EnumDriveToDestResult.FAILED
            return False

        self.drive_to_dest_result = EnumDriveToDestResult.INIT   
        self.set_timeout_timer(10.0)

        return True

    def SendStop(self):
        driving_command_request = DrivingCommandSrv.Request()
        driving_command_request.path_id = self.cart_status.mission_status.path_id if self.cart_status else ""
        driving_command_request.command.command = DrivingCommandMsg.STOP

        if not self.driving_command_client.post_request(driving_command_request):
            print("send DrivingCommand failed")  
           

    def SendVehicleModeCommandAutodrive(self):
        command_msg = VehicleStateData()
        command_msg.vehicle_mode = VehicleStateData.VEHICLE_MODE_COMPLETE_AUTO_DRIVE
        self.vehicle_state_data_command_pub.publish(command_msg)
        print('vehicle_mode command ' + str(command_msg.vehicle_mode) + " sent")

    def SpinOnce(self):
        pass
        # self.drive_to_point_client.check_out_response()
        # if (self.check_ended()):
        #     sys.exit(0)
    
    def check_ended(self):
        if (self.ended()):
            log_level = carteav_log.LogLevel.ERROR if self.drive_to_dest_result == EnumDriveToDestResult.FAILED else carteav_log.LogLevel.INFO
            print(self.drive_to_dest_result.name)
            carteav_log.publish_log(self, log_level, carteav_log.LogSubject.Simulator, self.drive_to_dest_result.name, self.scenario_name)
            self.drive_to_dest_result = EnumDriveToDestResult.NONE
            return True
        return False


