import matplotlib, matplotlib.pyplot as plt

import pdb

map_resolution = 0.005
#map_origin = [0., -0.3]
map_origin = [0., 0.]
map_w, map_h = 370, 500

def world_to_map(prl): return [1, -1]*(prl-map_origin)/map_resolution+[0, map_h]

class Window:

    def __init__(self):
        pass

    def display_map(self, m):
        plt.imshow(m.img)
        ticks = matplotlib.ticker.FuncFormatter(lambda _x, pos: '{0:g}'.format(_x*m.resolution))
        plt.gca().xaxis.set_major_formatter(ticks)
        plt.gca().set_xlabel('dimensions in meters')
        plt.gca().yaxis.set_major_formatter(ticks)
        plt.gca().set_ylabel('dimensions in meters')

    def display_path(self, points):
        map_points = world_to_map(points)
        plt.plot(map_points[:,0], map_points[:,1], marker='.', markersize=10, color='r')

        
    def show(self):
        plt.show()
