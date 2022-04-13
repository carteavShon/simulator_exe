#!/usr/bin/env python3
import simulator_types
import lgsvl
import sim_start
from geometry_msgs.msg import Point

def convert_sim_point(sim_point):
    world_point = Point()
    world_point.x = float(-sim_point.x)
    world_point.y = float(-sim_point.z)
    world_point.z = float(sim_point.y)
    return world_point

simulator = sim_start.LounchSimulator("Multi Mission",simulator_types.map_types.GanBIvrit)
cart_local_position = simulator_types.point_local2geo(simulator.ego.transform.position)
#destination_unity = simulator_types.point_geo2local(lgsvl.Vector(559,0,4780))
print(str(cart_local_position))
def cart_drive():
    simulator.ros_thread.send_drive_to_point(
        cart_local_position.y,
        cart_local_position.x,
        simulator_types.parking2["lat"],
        simulator_types.parking2["lon"])

cart_drive()

while True:
    if round(simulator.ego.transform.position.x) == 669 or round(simulator.ego.transform.position.z) == 4780 :
        cart_local_position = simulator_types.point_local2geo(simulator.ego.transform.position)
        simulator.ros_thread.send_drive_to_point(
                cart_local_position.y,
                cart_local_position.x,
                simulator_types.westGate["lat"],
                simulator_types.westGate["lon"])
    simulator.sim.run(1)
#simulator.ros_thread.send_drive_to_point()