import sys, os, io
# MapMOS class has a static map given by MOS
class MapMOS:
    def __init__(self,mapfile):
        # initialize variables

        self.VertexNames = []
        self.VertexType = {} # vertex_name: type (0:move, 1: station, 2: charging)
        self.VertexPos = {} # vertex_name: pos [x,y,z]
        self.Edge = {} # vertex_name: [vertex_name, vertex_name, vertex_name]

        self.EdgeMove = {} # A robot can move by a moving function
        self.EdgeLoad = {}  # A robot can move by a loading function
        self.EdgeUnload = {}  # A robot can move by a unloading function
        self.EdgeCharge = {}  # A robot can move by a charging function

        self.readmapfile(mapfile)

    def readmapfile(self, mapfile):
        f = open(mapfile, 'r')
        lines = f.readlines()
        lnum = 0 # line number
        vname = ""
        type = -1
        pos = []
        edges = []

        while lnum < len(lines):
            if lines[lnum] == 'Vertex\n':
                if not vname == "":
                    self.VertexNames.append(vname)
                    self.VertexType[vname] = type
                    self.VertexPos[vname] = pos
                    self.Edge[vname] = edges

                vname = ""
                type = -1
                pos = []
                edges = []
                lnum = lnum + 1

            if 'name' in lines[lnum]:
                vname = lines[lnum].split(" ")[1].split('\n')[0]
                lnum = lnum +1

            if 'type' in lines[lnum]:
                type = lines[lnum].split(" ")[1].split('\n')[0]
                lnum = lnum + 1

            if 'pos' in lines[lnum]:
                pos = [float(a) for a in lines[lnum].split(" ")[1:]]
                lnum = lnum+1

            if 'edge' in lines[lnum]:
                edges.append(lines[lnum].split(" ")[1].split('\n')[0])
                lnum = lnum + 1

        f.close()



if __name__=="__main__":
    map = MapMOS("../data/map_cloud.txt")
    print("done")

