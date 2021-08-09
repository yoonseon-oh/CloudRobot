import os
import sys
import threading
from threading import Condition

from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.configuration import BrokerType
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

from MapManagement.MapMOS import MapMOS
from MapManagement.MapCloudlet import MapCloudlet
from NavigationControl.NavigationControl import *
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
        self.ltm = NavigationControlDataSource()
        self.ltm.connect("tcp://127.0.0.1:61616", "", BrokerType.ZERO_MQ)
        
    def DoorStatus_query(self, consumer):
        self.query(consumer, "")

    def RobotPathLeft_query(self, consumer):
        self.query(consumer, "")
        
    
        
        








class NavigaionControlerAgent(ArbiAgent):
    def __init__(self, map_file):
        super().__init__()
        self.lock = Condition()
        self.map_file = map_file
        self.MAP = MapMOS(self.map_file)
        
    def NavigationControl_init(self, AMR_IDs):
        self.NC = NavigationControl(AMR_IDs)
    
    def on_start(self):
        ltm = NavigationControlDataSource()
        ltm.connect("tcp://127.0.0.1:61616", "", BrokerType.ZERO_MQ)
        
    
    def RobotNavCont_notify(self):
        pass
    
    def on_notify(self, sender:str, notification: str):
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        
        if temp_gl.get_name() == "RobotPose":
            pass
        
        elif temp_gl.get_name() == "RobotPathLeft":
            pass
        
    def call_LIFT_request(self):
        pass
    
    def call_TOW_request(self):
        pass
    
    def call_removeCargo_request(self):
        pass
        
class NavigationControlDataSource(DataSource):
    def __init__(self, broker_url):
        self.broker = broker_url
        self.connect(broker_url, "", BrokerType.ZERO_MQ)

        self.map_file = "/data/map_cloud.txt"
        self.MAP = MapMOS(self.map_file)

        