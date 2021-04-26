class VertexCloudlet:
    T = [0, 0.1, 0.2] # time index of map data
    numT = len(T)

    def __init__(self, x, y, object, err_code, robot_stop, robot_collision, err_freq):
        self.x = x # static: x position of the vertex
        self.y = y # static: y position of the vertex
        self.object = object #
        self.err_code = err_code
        self.robot_stop = robot_stop
        self.robot_collision = robot_collision
        self.err_freq = err_freq


