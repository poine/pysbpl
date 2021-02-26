#!/usr/bin/env python

'''
 Generates a motion primitive file for SBPL
 This script replaces the matlab utily provided with SBPL.
 It still puzzles me why people keep using this prehistoric monstrosity...
 
 Schmittle: Modified heavily for readability, modularity, and connection with limit surface
'''

import os, math, numpy as np, matplotlib.pyplot as plt

class MPrim:
    '''
    Base motion primitive class
    '''
    def __init__(self, base_prim_id, th_curr, th_res, grid_res, **kwargs):
        '''
        Constructor for super class
        **Parameters**
          - `base_prim_id` (int): arbitrary ID 
          - `th_curr` (int): index of given theta 
          - `th_res` (double): theta resolution 
          - `grid_res` (double): resolution of lattice
        **Returns**
          - **None** 
        '''
        self.base_prim_id, self.th_curr = base_prim_id, th_curr
        self.th = th_curr * th_res # current theta radians
        self.cos_th, self.sin_th = math.cos(self.th), math.sin(self.th) 
        self.start_pt_c, self.start_pt = np.array([0, 0, th_curr]), np.array([0, 0, self.th])
        self.cost = kwargs.get('cost', 1) # mprim cost
        self.interm_nb = kwargs.get('interm_nb', 10) # number of interim points
        self.interm_dist = kwargs.get('interm_dist', grid_res) # distance between interim points
        self.th_nb = 2*math.pi/th_res # number of discrete angles
        self.grid_resolution = grid_res
        self.th_res = th_res
        
    def to_string(self):
        '''
        Converts mprim to string for .mprim file
        **Parameters**
          - **None** 
        **Returns**
          - `txt` (str): mprim file string for given mprim
        '''
        txt  = 'primID: {}\n'.format(self.base_prim_id)
        txt += 'startangle_c: {}\n'.format(self.th_curr)
        txt += 'endpose_c: {} {} {}\n'.format(*self.end_pt_c)
        txt += 'additionalactioncostmult: {}\n'.format(self.cost)
        txt += 'intermediateposes: {}\n'.format(len(self.interm_pts))
        txt += ''.join(['{:.4f} {:.4f} {:.4f}\n'.format(*interm_pt) for interm_pt in self.interm_pts]) 
        return txt
    
    def round(self, real_pt):
        '''
        Converts real number to grid point
        and returns real number as a multiple of resolution
        **Parameters**
          - `real_pt` (list): x, y, theta point
        **Returns**
          - `grid_pt` (list): x, y, theta point on grid
          - real_pt_g` (list): x, y, theta point multiple of grid resolution
        '''
        grid_pt = [int(np.round(real_pt[0]/self.grid_resolution)), int(np.round(real_pt[1]/self.grid_resolution)), int(round(real_pt[2]/self.th_res))%self.th_nb]
        real_pt_g = [grid_pt[0]*self.grid_resolution, grid_pt[1]*self.grid_resolution, grid_pt[2]]
        return np.array(grid_pt), np.array(real_pt_g)

class MPrim_arc(MPrim):
    '''
    Arc motion primitive
    '''
    def __init__(self, base_prim_id, th_curr, th_res, grid_res, **kwargs):
        '''
        Constructor for arc class
        **Parameters**
          - `base_prim_id` (int): arbitrary ID 
          - `th_curr` (int): index of given theta 
          - `th_res` (double): theta resolution 
          - `grid_res` (double): resolution of lattice
        **Returns**
          - **None** 
        '''
        # super class constructor
        MPrim.__init__(self, base_prim_id, th_curr, th_res, grid_res, **kwargs)

        dth_curr, R = kwargs['dth_curr'], kwargs['R']  # heading variation and arc radius
        dth = dth_curr * self.th_res # heading variation radians 
        X1 = self.pt_on_circle(R if dth>0 else -R, dth)
        R1 = np.array([[self.cos_th, -self.sin_th],[self.sin_th, self.cos_th]]) # rotation matrix
        X2 = np.dot(R1, X1) # end point rotated to current angle (th)
        self.end_pt =  np.array([X2[0], X2[1], self.th+dth])
        self.end_pt_c, self.end_pt_grid = self.round(self.end_pt) # discretize

        # interim points
        interm_ths1 = np.linspace(0, dth, self.interm_nb)
        interm_pts1 = [self.pt_on_circle(R if dth>0 else -R, th) for th in interm_ths1]
        self.interm_pts = np.zeros((self.interm_nb, 3))
        for i in range(self.interm_nb):
            self.interm_pts[i,:2] = np.dot(R1, interm_pts1[i])
            self.interm_pts[i,2] = self.th + interm_ths1[i]
        # discretize last point to match end_pt_c, not sure if we need repeated points
        self.interm_pts[-1] =  self.end_pt_grid

    def pt_on_circle(self, R, dtheta): 
        '''
        For a given dtheta give me the x, y of the point on the circle
        See: https://math.stackexchange.com/questions/260096/find-the-coordinates-of-a-point-on-a-circle
        & here: https://math.stackexchange.com/questions/1384994/rotate-a-point-on-a-circle-with-known-radius-and-position
        for more details
        **Parameters**
          - `R` (double): arc radius
          - `dtheta` (double): delta theta 
        **Returns**
          - `[x, y]` (array): x,y point on circle
        '''
        return [R*math.sin(dtheta), R*(1-math.cos(dtheta))]

class MPrim_line(MPrim):
    '''
    Straight line motion primitive
    '''
    def __init__(self, base_prim_id, th_curr, th_res, grid_res, **kwargs):
        '''
        Constructor for arc class
        **Parameters**
          - `base_prim_id` (int): arbitrary ID 
          - `th_curr` (int): index of given theta 
          - `th_res` (double): theta resolution 
          - `grid_res` (double): resolution of lattice
        **Returns**
          - **None** 
        '''
        # super class constructor
        MPrim.__init__(self, base_prim_id, th_curr, th_res, grid_res, **kwargs)
        desired_len = kwargs['len_c'] * self.grid_resolution # meters
        actual_len = desired_len
        th_err, max_th_err = float("inf"), 0.5*self.th_res
        i = 0
        while abs(th_err) > max_th_err and i<5:
            self.end_pt = self.start_pt + [self.cos_th*actual_len, self.sin_th*actual_len, 0]
            self.end_pt_c, self.end_pt_grid = self.round(self.end_pt)
            th_err = self.th - math.atan2(self.end_pt_grid[1], self.end_pt_grid[0])
            if np.sign(desired_len) < 0: th_err += math.pi  # account for driving backwards
            if th_err > math.pi: th_err = 2*math.pi-th_err  # make sure errors is in the right direction
            l_err = np.linalg.norm(self.end_pt_grid[:2]) - desired_len
            actual_len  += np.sign(desired_len)*0.5*self.grid_resolution
            i+=1
            
        dX =  self.end_pt_grid - self.start_pt
        n_interm = np.linalg.norm(dX) / self.interm_dist + 2# self.interm_nb
        self.interm_nb = n_interm
        self.interm_pts = np.array([self.start_pt + i*dX for i in np.linspace(0, 1, int(n_interm))])
        
class MPrimFactory:
    '''
    Takes base prims array and applies to each angle
    '''

    def __init__(self, base_prims, grid_resolution=0.025, th_nb=16):
        '''
        Constructor 
        **Parameters**
          - `base_prims` (list): list of dicts describing base primitives
          - `grid_resolution` (double): resolution of lattice
          - `theta_nb` (int): heading discretization
        **Returns**
          - **None** 
        '''
        self.grid_resolution = grid_resolution
        self.base_prims = base_prims
        self.nb_mprim_per_angle = len(self.base_prims)
        self.th_nb = th_nb
        self.th_res = 2*math.pi/th_nb  # heading resolution 

    def build(self):
        '''
        Build motion primitives from base and store in self.mprims
        **Parameters**
          - **None** 
        **Returns**
          - **None** 
        '''
        self.mprims = []
        for angle_c in range(0, self.th_nb):
            for bpid, bp in enumerate(self.base_prims):
                self.mprims.append(bp['kind'](bpid, angle_c, self.th_res, self.grid_resolution, **bp['params']))

    def write(self, out_path):
        '''
        Write out .mprim file to given out_path
        **Parameters**
          - `out_path` (str): path + filename for .mprim file
        **Returns**
          - **None** 
        '''
        with open(out_path, 'w') as f:
            f.write('resolution_m: {:f}\n'.format(self.grid_resolution))
            f.write('numberofangles: {:d}\n'.format(self.th_nb))
            f.write('totalnumberofprimitives: {:d}\n'.format(self.th_nb*self.nb_mprim_per_angle))
            for mprim in self.mprims:
                f.write(mprim.to_string())

    def plot(self, ngrid=50, show_angle=None, show_prim_id=None, plot_points=None, multi_plot=False):
        '''
        Plot motion primitives on grid
        **Parameters**
          - `ngrid` (int): Size of grid for visualization x & y 
          - `show_angle` (list): which angles to visualize primitves. None=all 
          - `show_prim_id` (list): which primitve to show for a given angle. None=all 
          - `plot_points` (dict): which points to plot for a given primitive
        **Returns**
          - **None** 
        '''
        # Styling
        plt.rc('grid', linestyle="-", color='black')
        minc, maxc = -ngrid*self.grid_resolution, ngrid*self.grid_resolution
        ax = plt.gca()
        ax.set_xlim([minc,maxc]); ax.set_ylim([minc,maxc])
        minor_ticks = np.arange(minc, maxc, self.grid_resolution)
        major_ticks = np.arange(minc, maxc, 4*self.grid_resolution)
        ax.set_xticks(major_ticks); ax.set_yticks(major_ticks)
        ax.set_xticks(minor_ticks, minor=True)
        ax.set_yticks(minor_ticks, minor=True)
        plt.axes().set_aspect('equal')
        ax.grid(which='minor', alpha=0.2)                                                
        ax.grid(which='major', alpha=0.5)

        if plot_points is None:
            plot_points = {'start_end': True, 'start_end_disc': True, 'start_end_c':True, 'interim': True, 'interim_thetas': False} 
        
        # Plotting
        for p in self.mprims:
            if show_angle is None or p.th_curr in show_angle: # for each angle
                if show_prim_id is None or p.base_prim_id in show_prim_id: # for each base primitive
                    #TODO align marker colors to interim colors, otherwise confusing
                    # start & end points
                    if plot_points['start_end']:
                        plt.scatter([p.start_pt[0], p.end_pt[0]], [p.start_pt[1], p.end_pt[1]], marker=(5, 2))  # real
                    if plot_points['start_end_disc']:
                        plt.scatter([p.start_pt[0], p.end_pt_grid[0]], [p.start_pt[1], p.end_pt_grid[1]]) # discretized 

                    # discretized start/end check _c matches
                    if plot_points['start_end_c']:
                        plt.scatter([p.start_pt_c[0]*self.grid_resolution, p.end_pt_c[0]*self.grid_resolution],
                                    [p.start_pt_c[1]*self.grid_resolution, p.end_pt_c[1]*self.grid_resolution], marker=(5, 2)) 

                    # intermediate points
                    if plot_points['interim']:
                        plt.plot(p.interm_pts[:,0], p.interm_pts[:,1], '.-')

                    # plot interm thetas. causes a lot of graph noise
                    if plot_points['interim_thetas']:
                        a_len = 0.1
                        for ip in p.interm_pts:
                            plt.arrow(ip[0], ip[1], a_len * math.cos(ip[2]), a_len * math.sin(ip[2]),
                                      head_width=0.1*a_len, head_length=0.1*a_len, fc='k', ec='k')
        if not multi_plot:
            plt.show()

    def check_all_dirs(self, show_prim_id, th_nb):
        '''
        Plot subset of motion primitives on grid.
        Different plot for each angle
        **Parameters**
          - `show_prim_id` (list): which primitve to show for a given angle. None=all 
          - `th_nb` (list): which angles to show
        **Returns**
          - **None** 
        '''
        for a in th_nb:
            self.plot(show_angle=[a], show_prim_id=show_prim_id, multi_plot=True)
            plt.show()

def gen_oscar_prims():
    base_prims = [
        {'kind':MPrim_line, 'params':{'len_c': 1, 'cost':2}},             # forward straight short
        {'kind':MPrim_line, 'params':{'len_c': 8, 'cost':1}},             # forward straight long
        #{'kind':MPrim_line, 'params':{'len_c':-1, 'cost':5}},            # backward straight short
        {'kind':MPrim_arc,  'params':{'R':0.24,  'dth_curr': 1, 'cost':7}},  # forward sharp turning left
        {'kind':MPrim_arc,  'params':{'R':0.24,  'dth_curr':-1, 'cost':7}},  # forward sharp turning right
        {'kind':MPrim_arc,  'params':{'R':0.30,  'dth_curr': 1, 'cost':6}},  # forward wide turning left
        {'kind':MPrim_arc,  'params':{'R':0.30,  'dth_curr':-1, 'cost':6}},  # forward wide turning right
        {'kind':MPrim_arc,  'params':{'R':0.50,  'dth_curr': 1, 'cost':5}},  # forward wide turning left
        {'kind':MPrim_arc,  'params':{'R':0.50,  'dth_curr':-1, 'cost':5}},  # forward wide turning right
        {'kind':MPrim_arc,  'params':{'R':0.75,  'dth_curr': 1, 'cost':4}},  # forward wide turning left
        {'kind':MPrim_arc,  'params':{'R':0.75,  'dth_curr':-1, 'cost':4}},  # forward wide turning right
        {'kind':MPrim_arc,  'params':{'R':1.00,  'dth_curr': 1, 'cost':3}},  # forward wide turning left
        {'kind':MPrim_arc,  'params':{'R':1.00,  'dth_curr':-1, 'cost':3}},  # forward wide turning right
        {'kind':MPrim_arc,  'params':{'R':1.25,  'dth_curr': 1, 'cost':2}},  # forward wide turning left
        {'kind':MPrim_arc,  'params':{'R':1.25,  'dth_curr':-1, 'cost':2}},  # forward wide turning right
        #{'kind':MPrim_arc,  'params':{'R':-0.15, 'dth_curr':-1, 'cost':6}}, # backward turning left
        #{'kind':MPrim_arc,  'params':{'R':-0.15, 'dth_curr': 1, 'cost':6}}  # backward turning right
    ]
    f = MPrimFactory(base_prims, grid_resolution=0.005)
    f.build()
    #f.plot()
    f.check_all_dirs(None,[0]) 
    f.write('/home/schmittle/Desktop/test.mprim')
    
if __name__ == '__main__':
    # oscar
    gen_oscar_prims()
