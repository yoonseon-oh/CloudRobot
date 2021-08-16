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
            temp_RobotInfo = RobotInfo()
            temp_RobotInfo.id = gl_notify.get_expression(0)
            temp_RobotInfo.pos = [gl_notify.get_expression(1).as_value(), gl_notify.get_expression(1).as_value()]
            temp_RobotInfo.load = gl_notify.get_expression(3).as_value() # 1 for load , o for unload
            temp_RobotInfo.gl = gl_notify
            ### Timestamp ###
            self.MM.update_MOS_robot_info(temp_RobotInfo)
            ### Timestamp ###
            self.MM.detect_collision()
            
        elif gl_notify.get_name() == "DoorStatus":
            # self.DoorStatusNotify = gl_notify
            temp_DoorStatus = {}
            temp_DoorStatus['status'] = gl_notify.get_expression(1).as_value() ### Status Type Check ###
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

        self.CM_name = "agent://www.arbi.com/ContextManagerAgent" # name of Local-ContextManager Agent
        self.TA_name = "agent://www.arbi.com/TaskAllocationAgent" # name of Local-TaskAllocation Agent
        self.NC_name = "agent://www.arbi.com/NavigationControllerAgent" # name of Overall-NavigationConrtoller Agent
        
        # self.CargoPose_notify_switch = True
        # self.RackPose_notify_switch = True
        # self.RobotPose_notify_switch = True
        # self.Collidable_notify_switch = True
        # self.DoorStatus_notify_switch = True
        # self.RobotPathLeft_switch = True

    def on_start(self):
        self.ltm = MapManagerDataSource()
        self.ltm.connect("tcp://127.0.0.1:61616", "", BrokerType.ZERO_MQ)

        time.sleep(10)

        # CargoPose_notify_thread = threading.Thread(target=self.CargoPose_notify, args=(self.CM_name))
        # RackPose_notify_thread = threading.Thread(target=self.RackPose_notify, args=())
        # RobotPose_notify_thread = threading.Thread(target=self.RobotPose_notify, args=())
        # Collidable_notify_thread = threading.Thread(target=self.Collidable_notify, args=())
        # DoorStatus_notify_thread = threading.Thread(target=self.DoorStatus_notify, args=())
        # RobotPathLeft_notify_thread = threading.Thread(target=self.RobotPathLeft, args=())

        # CargoPose_notify_thread.start()
        # RackPose_notify_thread.start()
        # RobotPose_notify_thread.start()
        # Collidable_notify_thread.start()
        # DoorStatus_notify_thread.start()
        # RobotPathLeft_notify_thread.start()
    
    def on_notify(self, sender, notification):
        temp_gl = GLFactory.new_gl_from_gl_string(notification)
        
        if temp_gl.get_name() == "RobotPathPlan": # Notify from Navigation Controller Agent
            temp_path = []
            temp_gl_amr_id = temp_gl.get_expression(0)

            temp_gl_path = temp_gl.get_expression(1).as_generalized_list()

            temp_gl_path_size = temp_gl_path.get_expression_size()

            for i in range(temp_gl_path_size):
                temp_path.append(temp_gl_path.get_expression(i))
        
            self.ltm.MM.insert_NAV_PLAN(temp_gl_amr_id, temp_path)
    
    def CargoPose_notify(self, consumer):
        temp_Cargo_info = self.ltm.MM.CARGO

        for id in temp_Cargo_info.keys():
            ### Cargo_id != -1 ###
            temp_Cargo_id = id
            temp_Cargo_vertex = temp_Cargo_info[id]['vertex']
            temp_Cargo_robot_id = temp_Cargo_info[id]['load_id'][0]
            temp_Cargo_rack_id = temp_Cargo_info[id]['load_id'][1]
            temp_Cargo_status = []

            ### temp_gl generation w/ above info
            self.notify(consumer, "")

        threading.Timer(1, self.CargoPose_notify).start()
    
    def RackPose_notify(self, consumer):
        temp_RackPose_LIFT_info = self.ltm.MM.RACK_LIFT
        for id in temp_RackPose_LIFT_info.keys():
            ### Rack_id != -1 ###
            temp_RackPose_LIFT_id = id
            temp_RackPose_LIFT_vertex = temp_RackPose_LIFT_info[id]['vertex']
            temp_RackPose_LIFT_robot_id = temp_RackPose_LIFT_info[id]['load_id'][0]
            temp_RackPose_LIFT_cargo_id = temp_RackPose_LIFT_info[id]['load_id'][1]
            temp_RackPose_LIFT_status = []

            ### temp_gl generation w/ above info ###
            self.notify(consumer, "")

        temp_RackPose_TOW_info = self.ltm.MM.RACK_TOW
        for id in temp_RackPose_TOW_info.keys():
            ### Rack_id != -1 ###
            temp_RackPose_TOW_id = id
            temp_RackPose_TOW_vertex = temp_RackPose_TOW_info[id]['vertex']
            temp_RackPose_TOW_robot_id = temp_RackPose_TOW_info[id]['load_id'][0]
            temp_RackPose_TOW_cargo_id = temp_RackPose_TOW_info[id]['load_id'][1]
            temp_RackPose_TOW_status = []

            ### temp_gl generation w/ above info ###
            self.notify(consumer, "")
            
        threading.Timer(1, self.RackPose_notify).start()
    
    def RobotPose_notify(self, consumer):
        temp_AMR_LIFT_info = self.ltm.MM.AMR_LIFT
        temp_AMR_TOW_info = self.ltm.MM.AMR_TOW

        for id in temp_AMR_LIFT_info.keys():
            temp_AMR_LIFT_id = id
            temp_AMR_LIFT_vertex = temp_AMR_LIFT_info[id]['vertex']

            self.notify(consumer, "")

        for id in temp_AMR_TOW_info.keys():
            temp_AMR_TOW_id = id
            temp_AMR_TOW_vertex = temp_AMR_TOW_info[id]['vertex']

            self.notify(consumer, "")

        # temp_gl_string = "(RobotPose " + temp_id + " (vertex_id " + temp_vertex[0][0] + " " + temp_vertex[0][1] +"))"

        threading.Timer(1, self.RobotPose_notify).start()

    
    def DoorStatus_notify(self, consumer):

        temp_Door_info = self.ltm.MM.Door
        temp_Door_status = temp_Door_info[0]['status']
        ### Timestamp ###

        value = GLFactory.int_value(temp_Door_status) ### Temporary Data Type of DoorStatus ###
        expression = GLFactory.value_expression(value)
        temp_gl = GLFactory.new_generalized_list("DoorStatus", expression)

        self.notify(consumer, temp_gl)
        threading.Timer(1, self.DoorStatus_notify).start()
    

    def on_query(self, sender, query):
        gl_query = GLFactory.new_gl_from_gl_string(query)

        if gl_query.get_name() == "Collidable":
            temp = self.ltm.MM.detect_collision()
            self.send(sender, "")

        elif gl_query.get_name() == "RobotPathLeft":
            ### ??? ###
            self.send(sender, "")
            
        
        
if __name__ == "__main__":
    agent = MapManagerAgent()
    arbi_agent_excutor.execute(broker_url="tcp://127.0.0.1:61616", agent_name="agent://www.arbi.com/MapManagerAgent", agent=agent, broker_type=2) # same role with agent.initialize