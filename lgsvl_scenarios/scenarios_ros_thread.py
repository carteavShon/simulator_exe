import rclpy
import _thread, time, threading
from scenarios_node import ScenariosNode

import ament_index_python, sys
sys.path.append(ament_index_python.packages.get_package_share_directory('common_pkg'))
import carteav_log, service_client_util

class ScenariosRosThread():
    def __init__(self):
        self.scenarios_node = None
        self.ros_thread_id = 0   
        self.mutex = threading.Lock()     

    def rclpy_spin_thread(self):
        service_client_util.ros_thread_ref = self
        while (True):
            self.mutex.acquire()
            self.ros_spin_once()
            self.mutex.release()
            time.sleep(0.01)

    # listen to ros
    def ros_spin_once(self):
        try:
            # rclpy.spin(self.scenarios_node)
            if rclpy.ok():
                rclpy.spin_once(self.scenarios_node)
            # else:
            #     print('ros not ok')
            # rclpy.shutdown()
        except Exception as e:
            errorstr = 'rclpy_spin_thread failed: ' + str(e)
            print(errorstr)

    def run(self, scenario_name: str):
        rclpy.init()

        self.scenarios_node = ScenariosNode(scenario_name)

        self.ros_thread_id = _thread.start_new_thread(self.rclpy_spin_thread, ())

        print("ROS Thread started")

        self.scenarios_node.SendStop() 

    def stop(self):
        self.scenarios_node.SendStop()
        rclpy.shutdown()

    def send_drive_to_point(self, lat1, lon1, lat2, lon2):
        _thread.start_new_thread(
            self.send_drive_to_point, (lat1, lon1, lat2, lon2))

    def send_drive_to_point(self, lat1, lon1, lat2, lon2):
        if (self.scenarios_node == None):
            return False

        self.scenarios_node.SendVehicleModeCommandAutodrive()
        return self.scenarios_node.SendDriveToPoint(lat1, lon1, lat2, lon2)

      