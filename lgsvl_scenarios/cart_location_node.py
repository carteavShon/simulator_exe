#!/usr/bin/env python3
import lgsvl
import rclpy
from rclpy.node import Node
from carteav_interfaces.msg import CartLocation
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy


class CartLocationNode(Node):
    def __init__(self):
        besteffort_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT,
                                    history=HistoryPolicy.KEEP_LAST, durability=DurabilityPolicy.VOLATILE)
        super().__init__("cart_location")
        self.subscriber = self.create_subscription(CartLocation,"cart_location",self.location_callback,besteffort_qos)

    def location_callback(self, msg: CartLocation):
        print(str(msg._pose._position._x))

def main(args=None):
    rclpy.init(args=args)
    node = CartLocationNode()
    rclpy.spin(node)
    rclpy.shutdown() 

if __name__ == "__main__":
    main()