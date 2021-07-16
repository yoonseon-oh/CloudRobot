import sys, os, io, time
from MapManagement.VertexCloudlet import *
import threading
from MapManagement.MapMOS import MapMOS

class MapCloudlet:
    def __init__(self,mapfile):

        # Load static_map
        self.static_map = MapMOS(mapfile)

        # save all object's position : dictionary {ID: {timestamp:[], pos:[], vertex:[(vertex,vertex),(vertex,vertex)], with:[]}
        # with: 같이 있는 물체들을 list로 ex) AMR_LIFT['with']=[[rack, obj], [], ...]
        self.AMR_LIFT= {} # save AMR-LIFT pose
        self.AMR_TOW = {} # save AMR-TOW pose
        self.RACK_TOW = {} # save RACK-TOW pose
        self.RACK_LIFT = {} # save RACK-LIFT pose
        self.CARGO  = {} # save cargo pose

        # save plan
        self.Plan_AMR_LIFT = {}  # save AMR-LIFT's plan
        self.Plan_AMR_TOW = {}  # save AMR-TOW's plan

    def update_MOS_info(self): # update the pose of objects when MOS data is updated
        # MOS data related with robot
        if RobotPose

    def update_TM_info(self):

    def update_NAV_info(self):

    def convert_pose_to_vertex(self, pose):
        # return two closest vertex from pose, the distance should be less than threshold
        th = 0.1
        min_dist = [inf, inf], min_id =[-1, -1]
        for id, ver_pose in self.static_map.VertexPos.items():
            # compute distance
            dist_sq = (verpose[0]-pose[0])**2 + (verpose[2]-pose[1])**2
            if dist_sq < th**2:
                return [id, id]
            if dist_sq < mindist[0]:
                min_dist[0] = dist_sq
                min_id[0] = id
            elif dist_sq < mindist[1]:
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


