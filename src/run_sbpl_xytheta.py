#!/usr/bin/env python

import os, logging, yaml, matplotlib, matplotlib.image
LOG = logging.getLogger('sbpl_xytheta')
import pdb

import pysbpl, sbpl_gui as gui, guidance

class Config:
    def __init__(self, **kwargs):
        self.env = pysbpl.EnvironmentNAVXYTHETALAT()
        self.start_id, self.goal_id = self.env.InitializeEnv(**kwargs)
        self.map = kwargs['map']
        print self.start_id, self.goal_id


class Planner:
    def __init__(self, config):
        self.planner = pysbpl.ARAPlanner(config.env)
        self.config = config
        
    def initialize(self, start_id, goal_id):
        self.planner.initialize(start_id, goal_id)
        
    def run(self):
        points, headings = self.planner.run()
        points += self.config.map.origin[:2]
        return points, headings


def test_on_ethz_track():
    print('test_on_ethz_track')
    if 0:
        params = {
            'map': guidance.Map('/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/track_ethz_4_costmap.yaml'),
            'perimeter':[[-0.01, -0.04], [0.09, -0.04], [0.09, 0.04], [-0.01, 0.04]],
            'start':[0.165, 1.81, 1.57],
            'goal':[0.165, 1.80, 1.57], # one loop
            #'goal':[0.185, 1.55, 1.57],  # just on small line
            'goal_tol':[0.05, 0.05, 0.1],
            'vel':0.1, 'time_45_deg':10,
            'mprim_path':'/home/poine/work/oscar.git/oscar/oscar_navigation/params/sbpl/oscar_3.mprim',
        }
    if 0:
        params = {
            'map': guidance.Map('/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/track_ethz_dual_costmap_1.yaml'),
            'perimeter':[[-0.01, -0.04], [0.09, -0.04], [0.09, 0.04], [-0.01, 0.04]],
            'start':[2.4, 3.19, 0.],
            'goal':[2.4, 3.18, 0.], # one loop
            'goal_tol':[0.05, 0.05, 0.1],
            'vel':0.1, 'time_45_deg':10,
            'mprim_path':'/home/poine/work/oscar.git/oscar/oscar_navigation/params/sbpl/oscar_3.mprim',
        }
    if 0:
        params = {
            #'map': guidance.Map('/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/track_oval_02_costmap.yaml'),
            'map': guidance.Map('/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/track_slalom_01_costmap.yaml'),
            'perimeter':[[-0.01, -0.04], [0.09, -0.04], [0.09, 0.04], [-0.01, 0.04]],
            'start':[2.41, 0.25, 0.],
            'goal': [2.4, 0.25, 0.], # one loop
            'goal_tol':[0.05, 0.05, 0.1],
            'vel':0.1, 'time_45_deg':10,
            'mprim_path':'/home/poine/work/oscar.git/oscar/oscar_navigation/params/sbpl/oscar_3.mprim',
        }
        
    params = {
        'map': guidance.Map('/home/poine/work/rosmip.git/rosmip/rosmip_worlds/maps/enac_bench/track_test2_costmap.yaml'),
        'perimeter':[[-0.01, -0.04], [0.09, -0.04], [0.09, 0.04], [-0.01, 0.04]],
        'start':[1.21, 0.16, 0.0],
        'goal': [1.20, 0.16, 0.], # one loop
        'goal_tol':[0.05, 0.05, 0.1],
        'vel':0.1, 'time_45_deg':10,
        'mprim_path':'/home/poine/work/oscar.git/oscar/oscar_navigation/params/sbpl/oscar_3.mprim',
    }

    c = Config(**params)
    p = Planner(c)
    p.initialize(c.start_id, c.goal_id)
    points, headings = p.run()
    _path = guidance.path.Path(points=points, headings=headings)
    _path.save('/tmp/foo.npz')
    
    g = gui.Window()
    g.display_map(params['map'])
    g.display_path(points)
    g.show()
    

def test_on_julie():
    m = guidance.Map('/home/poine/work/simone/trunk/sbpl/test_sbpl/tmp/costmap.yaml')

    params = {'map':m,
              'perimeter':[[-0.01, -0.04], [0.09, -0.04], [0.09, 0.04], [-0.01, 0.04]],
              'start':[0.185, 1.5, 1.57], 'goal':[0.185, 1.49, 1.57], 'goal_tol':[0.01, 0.01, 0.1],
              'vel':0.1, 'time_45_deg':10,
              'mprim_path':'/home/poine/work/simone/trunk/julie/julie/julie_navigation/cfg/sbpl/julie_2.mprim'}

    c = Config(**params)
    
    p = Planner(c)
    p.run() # segfault... fuck!!!!
    
    g = gui.Window()
    g.display_map(m)
    g.show()

    
        
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    #test_on_julie()
    test_on_ethz_track()
