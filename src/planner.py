#!/usr/bin/env python
# Planner python wrapper
import time
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
        print('Planner Initialized')

    def plan(self, start, goal):
        self.kwargs['start'] = start
        self.kwargs['goal'] = goal 
        self.config = Config(first_time=False, **self.kwargs)
        print('Planning!')
        print('Start: {}'.format(str(self.kwargs['start'])))
        print('Goal: {} \n'.format(str(self.kwargs['goal'])))
        self.planner.initialize(self.config.start_id, self.config.goal_id)
        return self.run()
        
    def run(self):
        start_time = time.time()
        points, headings = self.planner.run()
        print('Planning Time: {}'.format(round(time.time() - start_time, 4)))
        if points is None:
            return None, None
        points += self.config.map.origin[:2]
        return points, headings
