#!/usr/bin/env python
# Planner python wrapper

import os
import map_util
import pysbpl, plot as gui

class Config:
    def __init__(self, **kwargs):
        self.env = pysbpl.EnvironmentNAVXYTHETALAT()
        self.start_id, self.goal_id = self.env.InitializeEnv(**kwargs)
        self.map = kwargs['map']

class Planner:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.config = Config(**kwargs)
        self.planner = pysbpl.ARAPlanner(self.config.env)

    def plan(self, start, goal):
        print('Planning!')
        print('Start: {}'.format(str(self.kwargs['start'])))
        print('Goal: {}'.format(str(self.kwargs['goal'])))
        self.kwargs['start'] = start
        self.kwargs['goal'] = goal 
        self.config = Config(**self.kwargs)
        self.planner.initialize(self.config.start_id, self.config.goal_id)
        return self.run()
        
    def run(self):
        points, headings = self.planner.run()
        if points is None:
            return None, None
        points += self.config.map.origin[:2]
        return points, headings
