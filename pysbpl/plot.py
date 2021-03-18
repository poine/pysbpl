#!/usr/bin/env python
import numpy as np
import matplotlib
import matplotlib.pyplot as plt

class Window:

     def __init__(self):
         self.map = None

     def display_map(self, map_file):
         self.map = map_file
         image = np.zeros((map_file.img.shape[0], map_file.img.shape[1],3))
         image[:,:,0] = np.invert(map_file.img)
         image[:,:,1] = np.invert(map_file.img)
         image[:,:,2] = np.invert(map_file.img)
         image[image==254]=0
         plt.imshow(image)
         ticks = matplotlib.ticker.FuncFormatter(lambda _x, pos: '{0:g}'.format(_x*map_file.resolution))
         plt.gca().xaxis.set_major_formatter(ticks)
         plt.gca().set_xlabel('dimensions in meters')
         plt.gca().yaxis.set_major_formatter(ticks)
         plt.gca().set_ylabel('dimensions in meters')
         plt.gca().invert_yaxis()

     def display_path(self, points):
         map_points = self.world_to_map(points)
         plt.plot(map_points[:,0], map_points[:,1], marker='.', markersize=1, color='#36CDC4')

     def world_to_map(self, points):
         origin = self.map.origin[:2]
         resolution = self.map.resolution
         map_height = self.map.img.shape[0]
         return [1, 1]*(points-origin)/resolution
        
     def show(self):
         plt.show()
