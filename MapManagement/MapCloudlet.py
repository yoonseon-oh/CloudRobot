import sys, os, io

class MapCloudlet:
    def __init__(self,mapfile):
        vertices, edges = self.readmapfile(mapfile)

    def readmapfile(self,mapfile):
        f = open(mapfile, 'r')
        lines = f.readlines()
        for line in lines:
            if line =='Vertex\n':
                v = VertexCloudlet()

        vertices = []

        if 'Vertex' in line:
            v = VertexCloudlet()
        elif 'pos' in line:
            term = line.split("")
            v.x = term[1]
            v.y = term[3]



        print(line.split(" "))
        f.close()



if __name__=="__main__":
    map = MapCloudlet("../data/map.txt")
    print("done")

