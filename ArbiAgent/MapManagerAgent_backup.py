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
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo

class MapManagerAgent(ArbiAgent):
    def __init__(self, map_file):
        super().__init__()
        self.lock = Condition()
        self.map_file = map_file
        self.MAP = MapMOS(self.map_file)
        # self.MM = MapCloudlet()

    def MapCloudlet_init(self):
        self.MM = MapCloudlet()
             
    def on_start(self):
        ltm = MapManagerDataSource()
        ltm.connect("tcp://127.0.0.1:61616", "", BrokerType.ZERO_MQ)
    
    def on_notify(self, sender: str, notification: str):
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        
        if temp_gl.get_name() == "RobotPathPlan":
            pass
        

    def CargoPose_notify(self):
        pass
    
    def RackPose_notify(self):
        pass
    
    def RobotPose_notify(self):
        pass
    
    def Collidable_notify(self):
        pass
    
    def DoorStatus_notify(self):
        pass
    
    def RobotPathLeft(self):
        pass
    
    
class MapManagerDataSource(DataSource):
    def __init__(self, broker_url):
        
        self.broker = broker_url
        
        self.connect(broker_url, "", BrokerType.ZERO_MQ)
        
        self.map_file = "/data/map_cloud.txt"
        self.MAP = MapMOS(self.map_file)
        
        self.AMR_LIFT_IDs = ["AMRLIFT0", "AMRLIFT1"]
        self.AMR_TOW_IDs = ["AMRTOW0", "AMRTOW1"]
        
        self.RobotPlan = {}
        
        self.CargoPose = []
        self.RackPose = []
        self.RobotPose = []
        self.Collidable = []
        self.DoorStatus = []
        self.RobotPathLeft = []
        
        
        
        self.MMAgent = MapManagerAgent()
        self.MMAgent_name = "agent://www.arbi.com/MMAgent"
        arbi_agent_excutor.execute()
        
        self.MMAgent.MapCloudlet_init()
        
        self.RobotInfoNotify = []
        self.RobotPathPlanNotify = []
        self.DoorStatusNotify = []
        
        self.sub_RobotInfo_ID = self.subscribe("(rule (fact (RobotInfo $robot_id $x $y $loading $speed $battery)) --> (notify (RobotInfo $robot_id $x $y $loading $speed $battery)))")
        # self.sub_RobotPathPlan_ID = self.subscribe() # LTM ? NavigationControllerAgent ?
        self.sub_DoorStatus_ID = self.subscribe("(rule (fact (DoorStatus $status)) --> (notify (DoorStatus $status)))")
        
    def on_notify(self, content):
        gl_notify = GLFactory.new_gl_from_gl_string(content)
        
        if gl_notify.get_name() == "RobotInfo":
            # self.RobotInfoNotify = gl_notify
            # RobotInfo 에 대한 method 작동
            temp_robotID = gl_notify.get_expression(0)
            temp_x = gl_notify.get_expression(1)
            temp_y = gl_notify.get_expression(2)
            temp_loading = gl_notify.get_expression(3)
            temp_speed = gl_notify.get_expression(4)
            temp_battery = gl_notify.get_expression(5)
            
        
        elif gl_notify.get_name() == "RobotPathPlan":
            # self.RobotPathPlanNotify = gl_notify
            # RobotPathPlan 에 대한 method 작동
            temp_robotID = gl_notify.get_expression(0)
            temp_path = gl_notify.get_expression(1).as_value()
            
        elif gl_notify.get_name() == "DoorStatus":
            # self.DoorStatusNotify = gl_notify
            # DoorStatus 에 대한 method 작동
            self.DoorStatus = gl_notify.get_expression(1)
 
if __name__ == "__main__":
    agent = MapManagerAgent()
    arbi_agent_excutor.execute(broker_url="tcp://127.0.0.1:61616", agent_name="agent://www.arbi.com/MapManagerAgent", agent=agent, broker_type=2)
    # same role with agent.initialize
    