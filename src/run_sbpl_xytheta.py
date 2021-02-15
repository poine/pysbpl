#!/usr/bin/env python
# An Example using pysbplyaml_path

import os
import map_util
from planner import Planner
import pysbpl, plot as gui

if __name__ == '__main__':
    src_dir = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
    params = {
        'map': map_util.Map(yaml_path='/home/schmittle/mushr/catkin_ws/src/mushr_sim/maps/sandbox.yaml'),
        'perimeter':[[-0.05, -0.05], [0.05, -0.05], [0.05, 0.05], [-0.05, 0.05]],
        'start':[5.21, 5.16, 0.0],
        'goal': [5.20, 50.16, 0.], # one loop
        'goal_tol':[0.5, 0.5, 0.1],
        'vel':0.5, 'time_45_deg':10,
        'mprim_path': (src_dir + '/mprim/pushr.mprim').encode('utf-8'),
        'obs_thresh': 200,
        'inscribed_thresh': 200,
        'possibly_circumscribed_thresh': 200,
    }

    # Basic API
    planner = Planner(**params)
    points, headings = planner.plan(params['start'], params['goal'])

    if points is not None:
        g = gui.Window()
        g.display_map(params['map'])
        g.display_path(points)
        g.show()
