import sys, os, io, time
from MapManagement.VertexCloudlet import *
import numpy as np
import threading
from MapManagement.MapMOS import MapMOS

class MapCloudlet:
    def __init__(self, mapfile, AMR_LIFT_IDs, AMR_TOW_IDs, robot_init,t_init = 0):
        # Mapfile: MOS map data file
        # AMR_LITF_IDS, AMR_TOW_IDs: list of robot IDs

        # Load static_map
        self.static_map = MapMOS(mapfile)
        self.STATION_LITF_TO_TOW = [21, 22] # station [LIFT, TOW] TODO: check the value

        # initialize - number
        self.AMR_LIFT_IDs = AMR_LIFT_IDs
        self.AMR_TOW_IDs = AMR_TOW_IDs
        self.CARGO_IDs = [] # Add ID, if new cargo appears
        self.RACK_TOW_IDs = []  # Add ID, if new rack_tow appears
        self.RACK_LIFT_IDs = []  # Add ID, if new rack_lift appears
        self.t_init = 0

        # initialize - semantic map data
        # save all object's position : dictionary {ID: {'timestamp':[], 'pos':[], 'vertex':[(vertex,vertex),(vertex,vertex)], 'load_id':[], 'load':0}
        # load_id: 같이 있는 물체들을 list로 ex) AMR_LIFT['load_id']=[[rack, cargo], [], ...], RACK['load_id']=[[robot, cargo], CARGO['load_id']=[[robot, rack'], ..
        # load: 로드가 되면 1
        self.AMR_LIFT= {} # save AMR-LIFT pose
        self.AMR_TOW = {} # save AMR-TOW pose
        self.RACK_TOW = {} # save RACK-TOW pose
        self.RACK_LIFT = {} # save RACK-LIFT pose
        self.CARGO  = {} # save cargo pose
        self.DOOR = {}

        # save plan
        self.Plan_AMR_LIFT = {}  # save AMR-LIFT's plan
        self.Plan_AMR_TOW = {}  # save AMR-TOW's plan

        # initialize agian
        for id, val in robot_init.items():
            if id in self.AMR_LIFT_IDs:
                self.AMR_LIFT[id]= {'timestamp': [t_init], 'pos': [val], 'vertex' : [self.convert_pose_to_vertex(val)],
                                                                                'load':[0], 'load_id':[[-1,-1]]}
            elif id in self.AMR_TOW_IDs:
                self.AMR_TOW[id]= {'timestamp': [t_init], 'pos': [val], 'vertex' : [self.convert_pose_to_vertex(val)],
                                                                                'load':[0], 'load_id':[[-1,-1]]}


    def update_MOS_robot_info(self, info): # call this function when robot_information is updated by MOS

        if info.id in self.AMR_LIFT_IDs:
            info.vertex = self.convert_pose_to_vertex(info.pos)
            if self.robot_update_rule(self.AMR_LIFT, info):  # if the update rule is satisfied
                self.AMR_LIFT[info.id]['timestamp'].append(info.timestamp)
                self.AMR_LIFT[info.id]['pos'].append(info.pos)
                self.AMR_LIFT[info.id]['vertex'].append(info.vertex)
                self.AMR_LIFT[info.id]['load'].append(info.load)

                # update loaded/unloaded objects
                if info.load ==1:
                    if self.AMR_LIFT[info.id]['load'][-2] ==1: # a robot is carrying a rack
                        # copy previous state
                        load_id =  self.AMR_LIFT[info.id]['load_id'][-1]
                    else: # a robot loads a rack, now.
                        load_id = [self.search_obj_at_vertex(self.RACK_LIFT, info.vertex), self.search_obj_at_vertex(self.CARGO, info.vertex)]

                    # update the lift
                    self.AMR_LIFT[info.id]['load_id'].append(load_id)

                    # update the rack_lift
                    if load_id[0] != -1:  # a robot is carrying a rack
                        id = load_id[0]
                        self.RACK_LIFT[id]['timestamp'].append(info.timestamp)
                        self.RACK_LIFT[id]['pos'].append(info.pos)
                        self.RACK_LIFT[id]['vertex'].append(info.vertex)
                        self.RACK_LIFT[id]['load_id'].append([info.id, load_id[1]])

                    # update the state of cargos that a robot is carrying
                    if load_id[1] != -1:
                        id = load_id[1]
                        self.CARGO[id]['timestamp'].append(info.timestamp)
                        self.CARGO[id]['pos'].append(info.pos)
                        self.CARGO[id]['vertex'].append(info.vertex)
                        self.CARGO[id]['load_id'].append([info.id, load_id[0]])

                elif info.load ==0 and self.AMR_LIFT[info.id]['load'][-2] == 1: # unload the rack now
                    self.AMR_LIFT[info.id]['load_id'].append([-1,-1])

                    # rack id and cargo id before unloading
                    rack_id = self.AMR_LIFT[info.id]['load_id'][-2][0]
                    cargo_id = self.AMR_LIFT[info.id]['load_id'][-2][1]
                    # update the rack_lift
                    self.RACK_LIFT[rack_id]['timestamp'].append(info.timestamp)
                    self.RACK_LIFT[rack_id]['pos'].append(info.pos)
                    self.RACK_LIFT[rack_id]['vertex'].append(info.vertex)
                    self.RACK_LIFT[rack_id]['load_id'].append([-1, cargo_id])  # change this state

                    # update the state of cargos that a robot is carrying
                    self.CARGO[cargo_id]['timestamp'].append(info.timestamp)
                    self.CARGO[cargo_id]['pos'].append(info.pos)
                    self.CARGO[cargo_id]['vertex'].append(info.vertex)
                    self.CARGO[cargo_id]['load_id'].append([-1, rack_id])

                else:
                    self.AMR_LIFT[info.id]['load_id'].append([-1,-1])

        # update tow
        elif info.id in self.AMR_TOW_IDs:
            info.vertex = self.convert_pose_to_vertex(info.pos)
            if self.robot_update_rule(self.AMR_TOW, info):  # if the update rule is satisfied
                self.AMR_TOW[info.id]['timestamp'].append(info.timestamp)
                self.AMR_TOW[info.id]['pos'].append(info.pos)
                self.AMR_TOW[info.id]['vertex'].append(info.vertex)
                self.AMR_TOW[info.id]['load'].append(info.load)

                # update loaded/unloaded objects
                if info.load == 1:
                    if self.AMR_TOW[info.id]['load'][-2] == 1:  # a robot is carrying a rack
                        # copy previous state
                        load_id = self.AMR_TOW[info.id]['load_id'][-1]
                    else:  # a robot loads a rack, now.
                        load_id = [self.search_obj_at_vertex(self.RACK_TOW, info.vertex),
                                   self.search_obj_at_vertex(self.CARGO, info.vertex)]

                    # update the TOW
                    self.AMR_TOW[info.id]['load_id'].append(load_id)

                    # update the rack_TOW
                    if load_id[0] != -1:  # a robot is carrying a rack
                        id = load_id[0]
                        self.RACK_TOW[id]['timestamp'].append(info.timestamp)
                        self.RACK_TOW[id]['pos'].append(info.pos)
                        self.RACK_TOW[id]['vertex'].append(info.vertex)
                        self.RACK_TOW[id]['load_id'].append([info.id, load_id[1]])

                    # update the state of cargos that a robot is carrying
                    if load_id[1] != -1:
                        id = load_id[1]
                        self.CARGO[id]['timestamp'].append(info.timestamp)
                        self.CARGO[id]['pos'].append(info.pos)
                        self.CARGO[id]['vertex'].append(info.vertex)
                        self.CARGO[id]['load_id'].append([info.id, load_id[0]])

                elif info.load == 0 and self.AMR_TOW[info.id]['load'][-2] == 1:  # unload the rack now
                    self.AMR_TOW[info.id]['load_id'].append([-1, -1])

                    # rack id and cargo id before unloading
                    rack_id = self.AMR_TOW[info.id]['load_id'][-2][0]
                    cargo_id = self.AMR_TOW[info.id]['load_id'][-2][1]
                    # update the rack_tow
                    self.RACK_TOW[rack_id]['timestamp'].append(info.timestamp)
                    self.RACK_TOW[rack_id]['pos'].append(info.pos)
                    self.RACK_TOW[rack_id]['vertex'].append(info.vertex)
                    self.RACK_TOW[rack_id]['load_id'].append([-1, cargo_id])  # change this state

                    # update the state of cargos that a robot is carrying
                    self.CARGO[cargo_id]['timestamp'].append(info.timestamp)
                    self.CARGO[cargo_id]['pos'].append(info.pos)
                    self.CARGO[cargo_id]['vertex'].append(info.vertex)
                    self.CARGO[cargo_id]['load_id'].append([-1, rack_id])

                else:
                    self.AMR_TOW[info.id]['load_id'].append([-1, -1])

    def robot_update_rule(self, robot, info): # define the update rule of robot status: return true to update
        return info.vertex != robot[info.id]['vertex'][-1] or info.load != robot[info.id]['load'] # or info.status!= robot[info.id]['status'][-1]

    def update_MOS_door_info(self, info):  # update door_state is updated by MOS #TODO: Check
        if info.status != self.DOOR['status']:
            self.DOOR['timestamp'].append(info.timestamp)
            self.DOOR['status'].append(info.status)

    def update_NAV_info(self, info_plan): #call when the navigation module generate a new path for a robot #TODO: Check
        if info_plan.id in self.AMR_LIFT_IDs:
            self.Plan_AMR_LIFT[info_plan.id] = info_plan.plan

        elif info_plan.id in self.AMR_TOW_IDs:
            self.Plan_AMR_TOW[info_plan.id] = info_plan.plan

    def call_LIFT(self, info): # call if a human calls AMR-LIFT
        # ADD cargo and rack id
        new_cargo_id = 'CARGO' + str(len(self.CARGO_IDs)) # TODO: change later
        new_rack_id = 'RACK' + str(len(self.RACK_LIFT_IDs))  # TODO: change later
        self.CARGO_IDs.append(new_cargo_id)
        self.RACK_LIFT_IDs.append(new_rack_id)

        info.vertex = self.convert_pose_to_vertex(info.pos)
        # ADD CARGO
        self.CARGO[new_cargo_id] = {'timestamp': [info.timestamp], 'pos': [info.pos], 'vertex':[info.vertex], 'load_id': [[-1, new_rack_id]]}

        # ADD RACK
        self.RACK_LIFT[new_rack_id] = {'timestamp': [info.timestamp], 'pos': [info.pos], 'vertex':[info.vertex], 'load_id': [[-1, new_cargo_id]]}

    def call_TOW(self, info_call): # call if a human calls AMR-TOW (cargo was moved from LIFT to TOW)
        # search a rack and cargo located at STATION_LIFT_TO_TOW[0]
        cargo_id = self.search_obj_at_vertex(self.CARGO,[self.STATION_LITF_TO_TOW[0],self.STATION_LITF_TO_TOW[0]])
        rack_lift_id = self.search_obj_at_vertex(self.RACK_LIFT, [self.STATION_LITF_TO_TOW[0], self.STATION_LITF_TO_TOW[0]])
        rack_tow_id = self.search_obj_at_vertex(self.RACK_TOW,
                                                  [self.STATION_LITF_TO_TOW[1], self.STATION_LITF_TO_TOW[1]])

        # update the cargo
        self.CARGO[cargo_id]['timestamp'].append(info_call.timestamp)
        postmp = self.static_map.VertexPos[self.STATION_LITF_TO_TOW[1]]
        self.CARGO[cargo_id]['pos'].append([postmp[0],postmp[2]])
        self.CARGO[cargo_id]['vertex'].append([self.STATION_LITF_TO_TOW[1],self.STATION_LITF_TO_TOW[1]])
        self.CARGO[cargo_id]['load_id'].append([-1, rack_tow_id])

        # update the rack_lift
        self.RACK_LIFT[rack_lift_id]['timestamp'].append(info_call.timestamp)
        postmp = self.static_map.VertexPos[self.STATION_LITF_TO_TOW[0]]
        self.RACK_LIFT[rack_lift_id]['pos'].append([postmp[0],postmp[2]])
        self.RACK_LIFT[rack_lift_id]['vertex'].append([self.STATION_LITF_TO_TOW[0],self.STATION_LITF_TO_TOW[0]])
        self.RACK_LIFT[rack_lift_id]['load_id'].append([self.RACK_LIFT[rack_lift_id]['load_id'][-1][0], -1])

        # update the rack_tow
        self.RACK_TOW[rack_tow_id]['timestamp'].append(info_call.timestamp)
        postmp=self.static_map.VertexPos[self.STATION_LITF_TO_TOW[1]]
        self.RACK_TOW[rack_tow_id]['pos'].append([postmp[0],postmp[2]])
        self.RACK_TOW[rack_tow_id]['vertex'].append([self.STATION_LITF_TO_TOW[1], self.STATION_LITF_TO_TOW[1]])
        self.RACK_TOW[rack_tow_id]['load_id'].append([self.RACK_TOW[rack_tow_id]['load_id'][-1][0], cargo_id])

    def add_RACK_TOW(self, timestamp, vertex):
        # ADD a new rack
        new_rack_id = 'RACK' + str(len(self.RACK_TOW_IDs))  # TODO: change later
        self.RACK_TOW_IDs.append(new_rack_id)

        # ADD RACK
        self.RACK_TOW[new_rack_id] = {'timestamp': [timestamp], 'pos': [[self.static_map.VertexPos[vertex][0],self.static_map.VertexPos[vertex][2]]], 'vertex': [[vertex, vertex]],
                                       'load_id': [[-1, -1]]}

    def convert_pose_to_vertex(self, pose):
        # return two closest vertex from pose, the distance should be less than threshold
        th = 0.1
        min_dist = [np.inf, np.inf]
        min_id =[-1, -1]
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

    def search_obj_at_vertex(self, objlist, v): # objlist: AMR_LIFT, AMR_TOW, ...
        objs = []
        for id, info in objlist.items():
            if info['vertex'][-1] == v:
                objs.append(id)
        if len(objs) == 1:
            return objs[0]
        elif len(objs) ==0:
            return -1
        else:
            print("Error: there are multiple objects at the vertex", objs)
            return -1


if __name__=="__main__":
    map = MapCloudlet("../data/map.txt")
    #regular_task = threading.Timer(1, map.map_update).start()
    thread = threading.Thread(target = map.map_update)
    thread.start()


    print("done")
    input()
    thread.stop()


