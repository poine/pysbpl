import time
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
        self.kwargs['start'] = start
        self.kwargs['goal'] = goal 
        self.config.initEnv(**self.kwargs)

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
