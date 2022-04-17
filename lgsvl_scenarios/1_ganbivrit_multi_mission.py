#!/usr/bin/env python3
from scenarios_node import EnumDriveToDestResult, ScenariosNode
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

simulator = sim_start.LounchSimulator("Multi Mission")
#destination_unity = simulator_types.point_geo2local(lgsvl.Vector(559,0,4780))

def cart_drive_to(destination):
    cart_local_position = simulator_types.point_local2geo(simulator.ego.transform.position)
    simulator.ros_thread.send_drive_to_point(
        cart_local_position.y,
        cart_local_position.x,
        destination["lat"],
        destination["lon"])

print("Cart Rotation: " + str(simulator.ego.transform.rotation.y))
cart_drive_to(simulator_types.poi_list[0])
i=1

simulator.sim.add_random_agents(lgsvl.AgentType.PEDESTRIAN)

npc_state = lgsvl.AgentState()
npc_state.transform = lgsvl.Transform(position= lgsvl.Vector(735,0,4850),rotation=lgsvl.Vector(0,180,0))
npc = simulator.sim.add_agent("Pc4Cart", lgsvl.AgentType.NPC,npc_state)


while True:
    if simulator.ros_thread.scenarios_node.drive_to_dest_result == EnumDriveToDestResult.FINISHED :
        simulator.sim.run(2)
        simulator.ros_thread.scenarios_node.drive_to_dest_result = EnumDriveToDestResult.INIT
        simulator.sim.run(1)
        cart_drive_to(simulator_types.poi_list[i])
        print("Cart Rotation: " + str(simulator.ego.transform.rotation.y))
        i+=1
    simulator.sim.run(1)
#simulator.ros_thread.send_drive_to_point()