#!/usr/bin/env python
# An Example using pysbpl

import os
import pysbpl
from pysbpl import map_util, plot, planner

if __name__ == '__main__':
    src_dir = os.path.dirname(pysbpl.__file__)
    params = {
        'map': map_util.Map(yaml_path=src_dir + '/maps/sandbox.yaml'),
        'perimeter':[[-0.05, -0.05], [0.05, -0.05], [0.05, 0.05], [-0.05, 0.05]],
        'start':[5.21, 5.16, 0.0], # not required
        'goal': [5.20, 70.16, 0.], # not required 
        'goal_tol':[0.5, 0.5, 0.1], 
        'vel':0.5, 'time_45_deg':10,
        'mprim_path': (src_dir + '/mprim/pushr.mprim').encode('utf-8'),
        'obs_thresh': 200,
        'inscribed_thresh': 200,
        'possibly_circumscribed_thresh': 200,
    }

    # Basic API
    sbpl_planner = planner.Planner(**params)
    points, headings = sbpl_planner.plan(params['start'], params['goal'])

    if points is not None:
        g = plot.Window()
        g.display_map(params['map'])
        g.display_path(points)
        g.show()
