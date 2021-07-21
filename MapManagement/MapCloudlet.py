import sys, os, io, time
from MapManagement.VertexCloudlet import *
import threading
from MapManagement.MapMOS import MapMOS

class MapCloudlet:
    def __init__(self,mapfile, AMR_LIFT_IDs, AMR_TOW_IDs):
        # Mapfile: MOS map data file
        # AMR_LITF_IDS, AMR_TOW_IDs: list of robot IDs

        # Load static_map
        self.static_map = MapMOS(mapfile)
        self.STATION_LITF_TO_TOW = [100, 101] # station [LIFT, TOW] TODO: check the value

        # initialize - number
        self.AMR_LIFT_IDs = AMR_LIFT_IDs
        self.AMR_TOW_IDs = AMR_TOW_IDs
        self.CARGO_IDs = [] # Add ID, if new cargo appears
        self.RACK_TOW_IDs = []  # Add ID, if new rack_tow appears
        self.RACK_LIFT_IDs = []  # Add ID, if new rack_lift appears

        # initialize - semantic map data
        # save all object's position : dictionary {ID: {'timestamp':[], 'pos':[], 'vertex':[(vertex,vertex),(vertex,vertex)], 'carry':[]}
        # carry: 같이 있는 물체들을 list로 ex) AMR_LIFT['carry']=[[rack, cargo], [], ...], RACK['carry']=[[robot, cargo], CARGO['carry']=[[robot, rack'], ..
        self.AMR_LIFT= {} # save AMR-LIFT pose
        self.AMR_TOW = {} # save AMR-TOW pose
        self.RACK_TOW = {} # save RACK-TOW pose
        self.RACK_LIFT = {} # save RACK-LIFT pose
        self.CARGO  = {} # save cargo pose
        self.DOOR = {}

        # save plan
        self.Plan_AMR_LIFT = {}  # save AMR-LIFT's plan
        self.Plan_AMR_TOW = {}  # save AMR-TOW's plan

    def update_MOS_robot_info(self, info): # call this function when robot_information is updated by MOS
        # TODO: how can I derive info.carry
        # Add parsing codes, if we need
        if info.id in self.AMR_LIFT_IDs:
            info.vertex = self.convert_pose_to_vertex(info.pos)
            if self.robot_update_rule(self.AMR_LIFT, info):  # if the update rule is satisfied
                self.AMR_LIFT[info.id]['timestamp'].append(info.timestamp)
                self.AMR_LIFT[info.id]['pos'].append(info.pos)
                self.AMR_LIFT[info.id]['vertex'].append(info.vertex)
                self.AMR_LIFT[info.id]['carry'].append(info.carry)

                if info.carry[0] !=-1: # a robot is carrying a rack
                    id = info.carry[0]
                    self.RACK_LIFT[id]['timestamp'].append(info.timestamp)
                    self.RACK_LIFT[id]['pos'].append(info.pos)
                    self.RACK_LIFT[id]['vertex'].append(info.vertex)
                    self.RACK_LIFT[id]['carry'].append([info.id, info.carry[1]])

                if info.carry[1] != -1:  # update the state of cargos that a robot is carrying
                    id = info.carry[1]
                    self.CARGO[id]['timestamp'].append(info.timestamp)
                    self.CARGO[id]['pos'].append(info.pos)
                    self.CARGO[id]['vertex'].append(info.vertex)
                    self.CARGO[id]['carry'].append([info.id, info.carry[0]])

        elif info.id in self.AMR_TOW_IDs:
            info.vertex = self.convert_pose_to_vertex(info.pos)
            if self.robot_update_rule(self.AMR_TOW, info): # if the vertex state is changed or status is changed, save the data
                self.AMR_TOW[info.id]['timestamp'].append(info.timestamp)
                self.AMR_TOW[info.id]['pos'].append(info.pos)
                self.AMR_TOW[info.id]['vertex'].append(info.vertex)
                self.AMR_TOW[info.id]['carry'].append(info.carry)

                # update the state of objects that a robot is carrying
                if info.carry[0] != -1:  # a robot is carrying a rack
                    id = info.carry[0]
                    self.RACK_TOW[id]['timestamp'].append(info.timestamp)
                    self.RACK_TOW[id]['pos'].append(info.pos)
                    self.RACK_TOW[id]['vertex'].append(info.vertex)
                    self.RACK_TOW[id]['carry'].append([info.id, info.carry[1]])

                if info.carry[1] != -1:  # update the state of cargos that a robot is carrying
                    id = info.carry[1]
                    self.CARGO[id]['timestamp'].append(info.timestamp)
                    self.CARGO[id]['pos'].append(info.pos)
                    self.CARGO[id]['vertex'].append(info.vertex)
                    self.CARGO[id]['carry'].append([info.id, info.carry[0]])

    def robot_update_rule(self, robot, info): # define the update rule of robot status: return true to update
        return info.vertex != robot[info.ID]['vertex'][-1] or info.carry != robot[info.ID]['carry'][-1] or info.status!= robot[info.ID]['status'][-1]

    def update_MOS_door_info(self, info):  # update door_state is updated by MOS
        if info.status != self.DOOR['status']:
            self.DOOR['timestamp'].append(info.timestamp)
            self.DOOR['status'].append(info.status)

    def update_NAV_info(self, info_plan): #call when the navigation module generate a new path for a robot
        if info_plan.id in self.AMR_LIFT_IDs:
            self.Plan_AMR_LIFT[info_plan.id] = info_plan.plan

        elif info_plan.id in self.AMR_TOW_IDs:
            self.Plan_AMR_TOW[info_plan.id] = info_plan.plan

    def add_cargo_call(self, info_call): # call if a human calls AMR-LIFT
        # ADD cargo and rack id
        new_cargo_id = len(self.CARGO_IDs) # TODO: change later
        new_rack_id = len(self.RACK_LIFT_IDs)  # TODO: change later
        self.CARGO_IDs.append(new_cargo_id)
        self.RACK_LIFT_IDs.append(new_rack_id)

        # ADD CARGO
        self.CARGO[id]['timestamp']=[info_call.timestamp]
        self.CARGO[id]['pos']=[info_call.pos]
        self.CARGO[id]['vertex']=[info_call.vertex]
        self.CARGO[id]['carry']=[[-1, new_rack_id]]

        # ADD RACK
        self.RACK_LIFT[id]['timestamp'] = [info_call.timestamp]
        self.RACK_LIFT[id]['pos'] = [info_call.pos]
        self.RACK_LIFT[id]['vertex'] = [info_call.vertex]
        self.RACK_LIFT[id]['carry'] = [[-1, new_cargo_id]]

    def move_cargo_call(self, info_call): # call if a human calls AMR-TOW (cargo was moved from LIFT to TOW)
        # search a rack and cargo located at STATION_LIFT_TO_TOW[0]
        cargo_id = -1
        rack_lift_id = -1
        rack_tow_id = -1
        for id, status in self.CARGO:
            if status['vertex'][0] == self.STATION_LITF_TO_TOW[0] and status['vertex'][1] == self.STATION_LITF_TO_TOW[0]:
                cargo_id = id
                rack_lift_id = self.CARGO[id]['carry'][1]
        for id, status in self.RACK_TOW:
            if status['vertex'][0] == self.STATION_LITF_TO_TOW[1] and status['vertex'][1] == self.STATION_LITF_TO_TOW[1]:
                rack_tow_id = id

        # update the cargo
        self.CARGO[cargo_id]['timestamp'].append(info_call.timestamp)
        self.CARGO[cargo_id]['pos'].append(self.static_map.VertexPos[self.STATION_LITF_TO_TOW[1]])
        self.CARGO[cargo_id]['vertex'].append([self.STATION_LITF_TO_TOW[1],self.STATION_LITF_TO_TOW[1]])
        self.CARGO[cargo_id]['carry'].append([-1, rack_tow_id])

        # update the rack_lift
        self.RACK_LIFT[rack_lift_id]['timestamp'].append(info_call.timestamp)
        self.RACK_LIFT[rack_lift_id]['pos'].append(self.static_map.VertexPos[self.STATION_LITF_TO_TOW[0]])
        self.RACK_LIFT[rack_lift_id]['vertex'].append([self.STATION_LITF_TO_TOW[0],self.STATION_LITF_TO_TOW[0]])
        self.RACK_LIFT[rack_lift_id]['carry'].append([-1, -1])

        # update the rack_tow
        self.RACK_TOW[rack_tow_id]['timestamp'].append(info_call.timestamp)
        self.RACK_TOW[rack_tow_id]['pos'].append(self.static_map.VertexPos[self.STATION_LITF_TO_TOW[1]])
        self.RACK_TOW[rack_tow_id]['vertex'].append([self.STATION_LITF_TO_TOW[1], self.STATION_LITF_TO_TOW[1]])
        self.RACK_TOW[rack_tow_id]['carry'].append([-1, cargo_id])

    def convert_pose_to_vertex(self, pose):
        # return two closest vertex from pose, the distance should be less than threshold
        th = 0.1
        min_dist = [inf, inf], min_id =[-1, -1]
        for id, ver_pose in self.static_map.VertexPos.items():
            # compute distance
            dist_sq = (ver_pose[0]-pose[0])**2 + (ver_pose[2]-pose[1])**2
            if dist_sq < th**2:
                return [id, id]
            if dist_sq < min_dist[0]:
                min_dist[0] = dist_sq
                min_id[0] = id
            elif dist_sq < min_dist[1]:
                min_dist[1] = dist_sq
                min_id[1] = id

        return min_id

if __name__=="__main__":
    map = MapCloudlet("../data/map.txt")
    #regular_task = threading.Timer(1, map.map_update).start()
    thread = threading.Thread(target = map.map_update)
    thread.start()


    print("done")
    input()
    thread.stop()


