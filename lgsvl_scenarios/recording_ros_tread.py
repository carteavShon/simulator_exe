import rclpy
import _thread, time, threading
import ament_index_python, sys
from rosBag_scenerio import TrackingObjectsNode
sys.path.append(ament_index_python.packages.get_package_share_directory('common_pkg'))
import carteav_log, service_client_util

class RecordedScenerioThread():
    def __init__(self):
        self.scenarios_node = None
        self.ros_thread_id = 0   
        self.mutex = threading.Lock()     

    def rclpy_spin_thread(self):
        service_client_util.ros_thread_ref = self
        while (True):
            self.mutex.acquire()
            self.ros_spin()
            self.mutex.release()
            time.sleep(0.01)

    # listen to ros
    def ros_spin(self):
        try:
            # rclpy.spin(self.scenarios_node)
            if rclpy.ok():
                rclpy.spin(self.scenarios_node)
            # else:
            #     print('ros not ok')
            # rclpy.shutdown()
        except Exception as e:
            errorstr = 'rclpy_spin_thread failed: ' + str(e)
            print(errorstr)

    def run(self):

        rclpy.init()

        self.scenarios_node = TrackingObjectsNode()

        self.ros_thread_id = _thread.start_new_thread(self.rclpy_spin_thread, ())

        print("ROS Thread started")

    def stop(self):
        rclpy.shutdown()

