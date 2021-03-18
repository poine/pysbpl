import time
import numpy as np
from pysbpl import sbpl

class Config:
    def __init__(self, **kwargs):
        self.env = sbpl.EnvironmentNAVXYTHETALAT()
        self.map = kwargs['map']
        self.env.update_map(self.map) # loads map once b4 planning

    def initEnv(self, **kwargs):
        self.start_id, self.goal_id = self.env.InitializeEnv(**kwargs)

class Planner:
    def __init__(self, **kwargs):
        self.kwargs = kwargs
        self.config = Config(**kwargs)
        self.planner = sbpl.ARAPlanner(self.config.env)

        print('Planner Initialized')

    def plan(self, start, goal):

        print('Planning!')
        print('Start: {}'.format(str(start)))
        print('Goal: {} \n'.format(str(goal)))

        # Transform from origin
        origin = np.array(self.config.map.origin)
        self.kwargs['start'] = np.array(start) - origin 
        self.kwargs['goal'] = np.array(goal) - origin

        self.config.initEnv(**self.kwargs)
        self.planner.initialize(self.config.start_id, self.config.goal_id)
        return self.run()
        
    def run(self):
        start_time = time.time()
        points, headings, primitives = self.planner.run()
        print('Planning Time: {}'.format(round(time.time() - start_time, 4)))
        if points is None:
            return None, None
        points += self.config.map.origin[:2]
        return points, headings, primitives
