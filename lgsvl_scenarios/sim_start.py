#!/usr/bin/env python3

import math,time
from environs import Env
import lgsvl
from lgsvl.geometry import Spawn, Transform, Vector
from scenarios_ros_thread import ScenariosRosThread
import simulator_types
from permutation_params import PermutationParams
from datetime import datetime

class LounchSimulator():

    def __init__(self,scenario_name,scene = simulator_types.map_types.GanBIvrit ,cart_position =lgsvl.Vector(731,0,4846) ,cart_rotation=lgsvl.Vector(0,180,0)):

        self.scenario_name = scenario_name
        self.scene = scene
        self.car_position = cart_position
        self.car_position = cart_position
        self.cart_rotation = cart_rotation

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")

        print('Scenrio #' + '. ' +self.scenario_name)

        vehicle=simulator_types.ego_types.DefaultType
        print('Vehicle: ' + vehicle.name + ' - ' + vehicle.value)


        self.env = Env()
        self.sim = lgsvl.Simulator(self.env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), self.env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))

        if self.sim.current_scene == self.scene.value:
            self.sim.reset()
        else:
            self.sim.load(self.scene.value)

        ego_state = lgsvl.AgentState()
        ego_state.transform = Transform(position = self.car_position, 
                                            rotation = self.cart_rotation)


        self.ego = self.sim.add_agent(vehicle.value,lgsvl.AgentType.EGO,ego_state)
        

        self.ros_thread = ScenariosRosThread()
        self.ros_thread.run(self.scenario_name)
        self.sim.run(1)
        self.ego.connect_bridge(self.env.str("LGSVL_AUTOPILOT_0_HOST",lgsvl.wise.SimulatorSettings.bridge_host),self.env.int("LGSVL_AUTOPILOT_0_PORT",lgsvl.wise.SimulatorSettings.bridge_port))

        print("Waiting For Connection ...")

        while not self.ego.bridge_connected:
            time.sleep(1)

        print("Bridge connected:", self.ego.bridge_connected)


        



    
