import os
import sys
import threading
from threading import Condition
from arbi_agent.agent import arbi_agent

from arbi_agent.agent.arbi_agent import ArbiAgent
from arbi_agent.configuration import BrokerType
from arbi_agent.ltm.data_source import DataSource
from arbi_agent.agent import arbi_agent_excutor
from arbi_agent.model import generalized_list_factory as GLFactory

from MapManagement.MapMOS import MapMOS
from MapManagement.MapCloudlet import MapCloudlet
from DataType.RobotInfo import RobotInfo
from DataType.CallInfo import CallInfo

class MapManagerDataSource(DataSource):
    def __init__(self, broker_url):
        
        self.broker = broker_url
        self.connect(self.broker, "", BrokerType.ZERO_MQ)
        
        self.map_file = ""
        self.MAP = MapMOS(self.map_file)
        
        self.sub_RobotInfo_ID = self.subscribe("(rule (fact (RobotInfo $robot_id $x $y $loading $speed $battery)) --> (notify (RobotInfo $robot_id $x $y $loading $speed $battery)))")
        self.sub_DoorStatus_ID = self.subscribe("(rule (fact (DoorStatus $status)) --> (notify (DoorStatus $status)))")

        self.AMR_LIFT_IDs = ["AMRLIFT0", "AMRLIFT1"]
        self.AMR_TOW_IDs = ["AMRTOW0", "AMRTOW1"]
        
        self.AMR_LIFT_init = {"AMRLIFT0": 0, "AMRLIFT1": 0}
        self.AMR_TOW_init = {"AMRTOW0": 0, "AMRTOW1": 0}
        
        self.Rack_LIFT_init = {'RACKLIFT0':0, 'RACKLIFT1':0, 'RACKLIFT2':0, 'RACKLIFT3':0}
        self.Rack_TOW_init = {'RACKTOW0':0, 'RACKTOW1':0}
        
        self.Door_init = {'Door0':0}
        
        self.RobotPlan = {}
        
        self.CargoPose = []
        self.RackPose = []
        self.RobotPose = []
        self.Collidable = []
        
        self.RobotPathLeft = []
        
        self.MM = MapCloudlet(self.map_file, self.AMR_TOW_init, self.AMR_TOW_init, self.Rack_TOW_init, self.Rack_LIFT_init, self.Door_init)
        
    def on_notify(self, content):
        gl_notify = GLFactory.new_gl_from_gl_string(content)
        
        if gl_notify.get_name() == "RobotInfo":
            # self.RobotInfoNotify = gl_notify
            # RobotInfo 에 대한 method 작동
            # temp_robotID = gl_notify.get_expression(0)
            # temp_x = gl_notify.get_expression(1)
            # temp_y = gl_notify.get_expression(2)
            # temp_loading = gl_notify.get_expression(3)
            # temp_speed = gl_notify.get_expression(4)
            # temp_battery = gl_notify.get_expression(5)
            temp_RobotInfo = RobotInfo()
            temp_RobotInfo.id = gl_notify.get_expression(0)
            temp_RobotInfo.pos = [gl_notify.get_expression(1).value().float_value(), gl_notify.get_expression(1).value().float_value()]
            temp_RobotInfo.load = gl_notify.get_expression(3) ### Parsing ###
            temp_RobotInfo.gl = gl_notify
            #### Timestamp ####
            self.MM.update_MOS_robot_info(temp_RobotInfo)
            ### Timestamp ###
            self.MM.detect_collision()
            
        # elif gl_notify.get_name() == "RobotPathPlan":
        #     # self.RobotPathPlanNotify = gl_notify
        #     # RobotPathPlan 에 대한 method 작동
        #     temp_robotID = gl_notify.get_expression(0)
        #     temp_path = gl_notify.get_expression(1).as_value()
            
        elif gl_notify.get_name() == "DoorStatus":
            # self.DoorStatusNotify = gl_notify
            # DoorStatus 에 대한 method 작동
            temp_DoorStatus = gl_notify.get_expression(1)
            self.MM.update_MOS_door_info(temp_DoorStatus)
        
        ### TEMP ###
        elif gl_notify.get_name() == "Call_LIFT":
            ### Parsing ###
            ### Parameter ###
            self.MM.call_LIFT()
        
        ### TEMP ###
        elif gl_notify.get_name() == "Call_TOW":
            ### Parsing ###
            ### Parameter ###
            self.MM.call_TOW()
        
        ### TEMP ###
        elif gl_notify.get_name() == "Call_Cargo":
            ### Parsing ###
            ### Parameter ###
            self.MM.call_removeCargo()
            
class MapManagerAgent(ArbiAgent):
    def __init__(self):
        super().__init__()
        self.lock = Condition()
        
    def on_start(self):
        self.ltm = MapManagerDataSource()
        self.ltm.connect("tcp://127.0.0.1:61616", "", BrokerType.ZERO_MQ)
    
    def on_notify(self, sender: str, notification: str):
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        
        if temp_gl.get_name() == "RobotPathPlan":
            pass
    
    def CargoPose_notify(self, consumer):
        self.notify(consumer, "")
    
    def RackPose_notify(self, consumer):
        self.notify(consumer, "")
    
    def RobotPose_notify(self, consumer):
        self.notify(consumer, "")
    
    def Collidable_notify(self, consumer): #### To On_query ###
        self.notify(consumer, "")
    
    def DoorStatus_notify(self, consumer):
        self.notify(consumer, "")
    
    def RobotPathLeft(self, consumer): #### To On_query ###
        self.notify(consumer, "")
        
if __name__ == "__main__":
    agent = MapManagerAgent()
    arbi_agent_excutor.execute(broker_url="tcp://127.0.0.1:61616", agent_name="agent://www.arbi.com/MapManagerAgent", agent=agent, broker_type=2) # same role with agent.initialize