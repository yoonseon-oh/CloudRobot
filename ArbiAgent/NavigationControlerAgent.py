import os
import sys
import time
sys.path.append("D:\git_ws\CloudRobot") ### TEMP ###
import threading
from threading import Condition
from arbi_agent.arbi_agent.agent import arbi_agent

from arbi_agent.arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.arbi_agent.configuration import BrokerType
from arbi_agent.arbi_agent.ltm.data_source import DataSource
from arbi_agent.arbi_agent.agent import arbi_agent_excutor
from arbi_agent.arbi_agent.model import generalized_list_factory as GLFactory

from MapManagement.MapMOS import MapMOS
from NavigationControl.NavigationControl import NavigationControl
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo

class NavigationControlerDataSource(DataSource):
    def __init__(self, broker_url):
        
        self.broker = broker_url
        self.connect(self.broker, "", BrokerType.ZERO_MQ)

        self.map_file = ""
        self.MAP = MapMOS(self.map_file)

        self.sub_temp_ID = self.subscribe("(rule (fact TEMP)) --> (notify (TEMP))")
        self.AMR_LIFT_IDs = ["AMRLIFT0", "AMRLIFT1"]
        self.AMR_TOW_IDs = ["AMRTOW0", "AMRTOW1"]
        
        self.AMR_IDs = ["AMRLIFT0", "AMRLIFT1", "AMRTOW0", "AMRTOW1"]
        
        self.NC = NavigationControl(self.AMR_IDs)
        
        self.RobotPathLeft = {}
        self.RobotPose = []
        self.Collidable = []

class NavigationControlerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.lock = Condition()
        self.TA_name = "agent://" # name of MAPF
        self.LIFT_TM_name = "agent://" # name of LIFT-TM
        self.TOW_TM_name = "agent://" # name of TOW-TM
        
    def on_start(self):
        self.ltm = NavigationControlerDataSource()
        self.ltm.connect("tcp://127.0.0.1:61616", "", BrokerType.ZERO_MQ)
    
    def on_data(self, sender, data):
        temp_gl = GLFactory.new_gl_from_gl_string(data)
        if temp_gl.get_name() == "MultiRobotPath":
            multi_robot_path = {}
            temp_gl_robot_num = temp_gl.get_expression_size()
            for i in range(temp_gl_robot_num):
                temp_robot_path = temp_gl.get_expression(i).as_generalized_list()
                temp_robot_id = temp_robot_path.get_expression(0)

                temp_gl_path = temp_robot_path.get_expression(1).as_generalized_list()
                temp_gl_path_size = temp_gl_path.get_expression_size()
                for j in range(temp_gl_path_size):
                    temp_path = []
                    temp_path.append(temp_gl_path.get_expression(j))

                multi_robot_path[temp_robot_id] = temp_path
            
            self.ltm.NC.get_multipath_plan(multi_robot_path)

    def on_notify(self, sender, notification):
        temp_gl = GLFactory.new_gl_from_gl_string(notification)

        if temp_gl.get_name() == "RobotPose":
            temp_robotID = self.ltm.NC.AMR_IDs[temp_gl.get_expression(0).as_value()]
            temp_vertex_info = temp_gl.get_expression(1).as_generalized_list()
            temp_vertex_1 = temp_vertex_info.get_expression(0).as_value()
            temp_vertex_2 = temp_vertex_info.get_expression(1).as_value()

            temp_robot_pose = {}
            temp_robot_pose[temp_robotID] = [temp_vertex_1, temp_vertex_2]

            self.ltm.NC.update_robot_TM(temp_robot_pose)

        # elif temp_gl.get_name() == "RobotPathLeft":
        #     temp_path = []
        #     temp_robotID = self.ltm.NC.AMR_IDs[temp_gl.get_expression(0).as_value()]
        #     temp_gl_path = temp_gl.get_expression(1).as_generalized_list()

        #     temp_gl_path_size = temp_gl_path.get_expression_size()

        #     for i in range(temp_gl_path_size):
        #         temp_path.append(temp_gl_path.get_expression(i))

        #     self.ltm.RobotPathLeft[temp_robotID] = temp_path

        elif temp_gl.get_name() == "Collidable":
            pass

    def LIFT_notify(self, consumer):
        pass

    def TOW_notify(self, consumer):
        pass



        
    
        
        







        