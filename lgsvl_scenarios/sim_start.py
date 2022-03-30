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
    def __init__(self,scenario_name,scene,cart_position,cart_rotation):
        self.scenario_name = scenario_name
        self.scene = scene
        self.car_position =cart_position
        self.car_position =cart_position
        self.cart_rotation= cart_rotation

        now = datetime.now()
        current_time = now.strftime("%H:%M:%S")

        print('Scenrio #' + '. ' +self.scenario_name)

        vehicle=simulator_types.ego_types.DefaultType
        print('Vehicle: ' + vehicle.name + ' - ' + vehicle.value)

        self.ros_thread = ScenariosRosThread()
        self.ros_thread.run(self.scenario_name)

        env = Env()
        self.sim = lgsvl.Simulator(env.str("LGSVL__SIMULATOR_HOST", lgsvl.wise.SimulatorSettings.simulator_host), env.int("LGSVL__SIMULATOR_PORT", lgsvl.wise.SimulatorSettings.simulator_port))

        if self.sim.current_scene == self.scene.value:
            self.sim.reset()
        else:
            self.sim.load(self.scene.value)

        ego_state = lgsvl.AgentState()
        ego_state.transform = Transform(position = self.car_position, 
                                            rotation = self.cart_rotation)

        forward = lgsvl.utils.transform_to_forward(ego_state)
        right = lgsvl.utils.transform_to_right(ego_state)

        ego = self.sim.add_agent(vehicle.value,lgsvl.AgentType.EGO,ego_state)

        ego.connect_bridge(env.str("LGSVL_AUTOPILOT_0_HOST",lgsvl.wise.SimulatorSettings.bridge_host),env.int("LGSVL_AUTOPILOT_0_PORT",lgsvl.wise.SimulatorSettings.bridge_port))

        print("Waiting For Connection ...")

        while not ego.bridge_connected:
            time.sleep(1)

        print("Bridge connected:", ego.bridge_connected)



    
