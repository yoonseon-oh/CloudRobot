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
        
    def on_notify(self, content):
        gl_notify = GLFactory.new_gl_from_gl_string(content)
        
        if gl_notify.get_name() == "TEMP":
            temp__ = gl_notify.get_expression(0)

class NavigationControlerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.lock = Condition()
        
    def on_start(self):
        self.ltm = NavigationControlerDataSource()
        self.ltm.connect("tcp://127.0.0.1:61616", "", BrokerType.ZERO_MQ)
        
    def DoorStatus_query(self, consumer):
        self.query(consumer, "")

    def RobotPathLeft_query(self, consumer):
        self.query(consumer, "")

    def on_notify(self, sender, notification):
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        
        if temp_gl.get_name() == "MultiRobotPath":

            temp_gl = temp_gl.get_expression(0).as_generalized_list()

            temp_robotID = temp_gl.get_expression(0)

            temp_path_start = temp_gl.get_expression(1)
            temp_path_end = temp_gl.get_expression(2)

            temp_path = []
            temp_gl_path = temp_gl.get_expression(3).as_generalized_list()

            temp_gl_path_size = temp_gl_path.get_expression_size()

            for i in range(temp_gl_path_size):
                temp_path.append(temp_gl_path.get_expression(i))
        
            self.ltm.NC.get_multipath_plan(temp_path)

        elif temp_gl.get_name() == "RobotPose":
            temp_robotID = temp_gl.get_expression(0)
            temp_vertex_info = temp_gl.get_expression(1).as_generalized_list()
            temp_vertex_1 = temp_vertex_info.get_expression(0)
            temp_vertex_2 = temp_vertex_info.get_expression(1)

            temp_robot_pose = {}
            temp_robot_pose[temp_robotID] = [temp_vertex_1, temp_vertex_2]

            self.ltm.NC.update_robot_TM(temp_robot_pose)

        elif temp_gl.get_name() == "RobotPathLeft":
            pass

    def on_request(self, sender, request):
        temp_gl = GLFactory.new_gl_from_gl_string(request)
        if temp_gl.get_name() == "goal":
            ### Algorithm ###

            self.send(sender, "") ### TEMP(response) ###



        
    
        
        







        