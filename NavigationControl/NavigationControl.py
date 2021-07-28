class NavigationControl:
    def __init__(self, AMR_IDs):
        # initialize
        self.AMR_IDs = AMR_IDs

        # path given by multi-robot path finder
        self.NavPath = {}
        self.timing = {} # execution timing
        for id in self.AMR_IDs:
            self.NavPath[id] = []
            self.timing[id] = [] # if NavPath[rid1][idx1] == NavPath[rid2][idx2] and idx2<idx1 : timing[id]=[[rid2, time_idx2, NavPath[rid1][idx1]], ...]

        # command for RobotTM
        self.robotTM = {} # the current command for RobotTM
        self.robotTM_set = {} # full sequence of commands for RobotTM
        self.robotTM_scond = {} # start condition of robotTM
        for id in self.AMR_IDs:
            self.robotTM[id] = [] # ['type', []] type: 'move' [path] or 'cancel'
            self.robotTM_set[id] = []
            self.robotTM_scond[id] = []

        self.PlanExecutedIdx = {}
        for id in self.AMR_IDs:
            self.PlanExecutedIdx[id] = -1 # Save the last index of NavPath which the robot follows

    def insert_navpath(self, multipaths): # multipath: MultiPath type
        start_condition = {}
        for key in multipaths.keys():
            start_condition[key] = []
        # Analyze timing
        for rid, path in multipaths.items():

            for idx in range(0, len(path)): # compare
                scond = []
                rid_list = self.AMR_IDs.copy()
                rid_list.remove(rid)
                for rid2 in rid_list:
                    for idx2 in range(min(idx,len(multipaths[rid2])-1), -1, -1):
                        if path[idx] == multipaths[rid2][idx2]:
                            scond.append([rid2, idx2])

                start_condition[rid].append(scond)

        # Split navigation paths to robotTM
        robotTM_mapping_set = {}
        scondTM_set = {}
        for rid, path in multipaths.items():
            robotTM_seq = []
            robotTM_mapping = [] # save index of robotTM_set corresponding to each element in path
            scond = []
            t1 = 0
            t2 = 0

            if path != []:
                robotTM = [path[0]]
                scond = [start_condition[rid][0]]
                for ii in range(1, len(path)):
                    if start_condition[rid][ii] == []:
                        if path[ii] != robotTM[-1]:
                            robotTM.append(path[ii])
                            t2 = t2 + 1

                    else:
                        robotTM_seq.append(robotTM)
                        t1 = t1+1
                        t2 = 0
                        robotTM  = [path[ii]]
                        scond.append(start_condition[rid][ii])

                    robotTM_mapping.append([t1,t2])

                robotTM_seq.append(robotTM)

            self.robotTM_set[rid] = robotTM_seq
            robotTM_mapping_set[rid] = robotTM_mapping
            scondTM_set[rid] = scond

        # find start condnition for TM
        scond_TM_translated = {}
        for rid in multipaths.keys():
            scond = []
            for tt in range(0, len(scondTM_set[rid])):
                if scondTM_set[rid][tt] !=[]:
                    cond_cur = []
                    for cond in scondTM_set[rid][tt]:
                        cond_cur.append([cond[0], robotTM_mapping_set[cond[0]][cond[1]]])

                    scond.append(cond_cur)
                else:
                    scond.append([])

            scond_TM_translated[rid] = scond

        self.robotTM_scond = scond_TM_translated


    def update_robot_TM(self, robot_pose): # call when the robot position changes - # TODO
        # robot_pose ={robot_id: [vertex, vertex], ...}

        # check whether the current TM command is executed
        for rid, vid in robot_pose.items():
            if self.robotTM[rid] != []:
                if vid in [[self.robotTM[rid][0]]*2]:
                    self.PlanExecutedIdx[rid] = self.PlanExecutedIdx[rid] + 1

        # Update the robot TM
        for rid, vid in robot_pose.items():
            if self.NavPath[rid] !=[]:
                if self.robotTM[rid] !=[]:
                    if vid in [[self.robotTM[rid][0]]*2]:
                        self.robotTM[rid].pop(0)
                else:
                    start_idx = self.PlanExecutedIdx[rid]+1
                    if self.timing[rid][start_idx] == []: # no condition
                        self.robotTM[rid] = self.extract_TM(self.NavPath[rid][start_idx:], self.timing[rid][start_idx:])
                        self.send_RobotTM(rid, self.robotTM[rid]) # send the command

                    else: # check condition
                        flag_start = True
                        for cond in self.timing[start_idx]:
                            if self.PlanExecutedIdx[cond[0]] <= cond[1]:
                                flag_start = False
                        if flag_start:
                            self.robotTM[rid] = self.extract_TM(self.NavPath[rid][start_idx:], self.timing[rid][start_idx:])
                            self.send_RobotTM(rid, self.robotTM[rid]) # send the command

            # Reset PlanExecutedIdx
            for rid, vid in robot_pose.items():
                if self.PlanExecutedIdx[rid] == len(self.NavPath[rid])-1:
                    self.PlanExecutedIdx[rid] = -1

        print("PlanExecuted ", self.PlanExecutedIdx)
        print("Robot TM:    ",self.robotTM)


    def extract_TM(self, navpath, timing): # Extract TM from navpath
        robotTM = [navpath[0]]
        for ii in range(1, len(navpath)):
            if timing[ii] == []:
                if navpath[ii] != robotTM[-1]:
                    robotTM.append(navpath[ii])
            else:
                break

        return robotTM


        # check if the start condition of a new TM command is satisfied.

        print("update robot TM")

    def send_RobotTM(self, robotid, robotTM): # send command to RobotTM
        print("send a command to RobotTM: ", rid, robotTM)

if __name__ == "__main__":
    AMR_IDs = ['AMRLIFT0', 'AMRLIFT1', 'AMRTOW0', 'AMRTOW1']
    navcont = NavigationControl(AMR_IDs)
    multipaths = {'AMRLIFT0': [1,2,4,6,5], 'AMRLIFT1': [3,3,2,4,6,7,7,8], 'AMRTOW0':[], 'AMRTOW1':[]}
    navcont.insert_navpath(multipaths)

    # test - update extract_TM
    #rid = AMR_IDs[1]
    #for sidx in range(0,6):
    #    navcont.extract_TM(navcont.NavPath[rid][sidx:], navcont.timing[rid][sidx:])

    # test -update_TM
    paths_execute =  {'AMRLIFT0': [0,1,2,4,6,5,5,5], 'AMRLIFT1': [3,3,2,4,6,7,7,8], 'AMRTOW0':[1]*10, 'AMRTOW1':[2]*10}
    for t in range(0,7):
        robot_pose = {}
        for key, val in paths_execute.items():
            robot_pose[key] = [val[t]]*2
        print(paths_execute['AMRLIFT0'][t], paths_execute['AMRLIFT1'][t])
        navcont.update_robot_TM(robot_pose)

    print("done")

