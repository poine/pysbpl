#!/usr/bin/env python

'''
 Generates a motion primitive file for sbpl
 This script replaces the matlab utily provided with sbpl.
 It still puzzles me why people keep using this prehistoric monstrosity...
 Do you really prefer 2k lines full of globals to 100 structured ones?
'''

import os, logging, math, numpy as np, scipy, matplotlib.image, matplotlib.pyplot as plt
import pdb

LOG = logging.getLogger('gen_mprim')

class MPrim:
    def __init__(self, base_prim_id, th_c, **kwargs):
        self.base_prim_id, self.th_c = base_prim_id, th_c
        self.th = th_c * MPrimFactory.th_res
        self.cth, self.sth = math.cos(self.th), math.sin(self.th)
        self.start_pt_c, self.start_pt = np.array([0, 0, th_c]), np.array([0, 0, self.th])
        self.cost = kwargs.get('cost', 1)
        self.interm_nb = kwargs.get('interm_nb', 10)
        self.interm_dist = kwargs.get('interm_dist', MPrimFactory.grid_resolution)
        
    def to_string(self):
        txt  = 'primID: {}\n'.format(self.base_prim_id)
        txt += 'startangle_c: {}\n'.format(self.th_c)
        txt += 'endpose_c: {} {} {}\n'.format(*self.end_pt_c)
        txt += 'additionalactioncostmult: {}\n'.format(self.cost)
        txt += 'intermediateposes: {}\n'.format(len(self.interm_pts))
        txt += ''.join(['{:.4f} {:.4f} {:.4f}\n'.format(*interm_pt) for interm_pt in self.interm_pts]) 
        return txt
    
def round_xy(X):
    X_c = [int(np.round(X[0]/MPrimFactory.grid_resolution)), int(np.round(X[1]/MPrimFactory.grid_resolution)), int(round(X[2]/MPrimFactory.th_res))%MPrimFactory.th_nb]
    X_r = [X_c[0]*MPrimFactory.grid_resolution, X_c[1]*MPrimFactory.grid_resolution, X[2]]
    return np.array(X_c), np.array(X_r)

def pt_on_0R_circle(R, th): return [R*math.sin(th), R*(1-math.cos(th))]

class MPrim_arc(MPrim):
    def __init__(self, base_prim_id, th_c, **kwargs):
        MPrim.__init__(self, base_prim_id, th_c, **kwargs)
        dth_c, R = kwargs['dth_c'], kwargs['R']  # heading variation and radius
        dth = dth_c * MPrimFactory.th_res        
        X1 = pt_on_0R_circle(R if dth>0 else -R, dth)
        R1 = np.array([[self.cth, -self.sth],[self.sth, self.cth]])
        X2 = np.dot(R1, X1)
        self.end_pt =  np.array([X2[0], X2[1], self.th+dth])
        self.end_pt_c, self.end_pt_grid = round_xy(self.end_pt)
        interm_ths1 = np.linspace(0, dth, self.interm_nb)
        interm_pts1 = [pt_on_0R_circle(R if dth>0 else -R, th) for th in interm_ths1]
        self.interm_pts = np.zeros((self.interm_nb, 3))
        for i in range(self.interm_nb):
            self.interm_pts[i,:2] = np.dot(R1, interm_pts1[i])
            self.interm_pts[i,2] = self.th + interm_ths1[i]
        self.interm_pts[-1] =  self.end_pt_grid # is that needed? CHECKME
        #print self.start_pt_c[2], self.end_pt_c[2], self.interm_pts[:,2]
        #print('a {:.3f} p {:02d}'.format(self.th, base_prim_id))

class MPrim_line(MPrim):
    def __init__(self, base_prim_id, th_c, **kwargs):
        MPrim.__init__(self, base_prim_id, th_c, **kwargs)
        desired_len = kwargs['len_c'] * MPrimFactory.grid_resolution
        actual_len = desired_len
        th_err, max_th_err = float("inf"), 0.5*MPrimFactory.th_res
        i = 0
        while abs(th_err) > max_th_err and i<5:
            self.end_pt = self.start_pt + [self.cth*actual_len, self.sth*actual_len, 0]
            self.end_pt_c, self.end_pt_grid = round_xy(self.end_pt)
            th_err = self.th - math.atan2(self.end_pt_grid[1], self.end_pt_grid[0])
            if np.sign(desired_len) < 0: th_err += math.pi  # account for driving backwards
            if th_err > math.pi: th_err = 2*math.pi-th_err  # make sure errors is in the right direction
            l_err = np.linalg.norm(self.end_pt_grid[:2]) - desired_len
            #print('l_err {:.3f} th_err {:.3f}'.format(l_err, th_err))
            actual_len  += np.sign(desired_len)*0.5*MPrimFactory.grid_resolution
            i+=1
            
        dX =  self.end_pt_grid - self.start_pt
        n_interm = np.linalg.norm(dX) / self.interm_dist + 2# self.interm_nb
        self.interm_nb = n_interm
        self.interm_pts = np.array([self.start_pt + i*dX for i in np.linspace(0, 1, n_interm)])
        #print('a {:.3f} p {:02d} l_err {:.3f} th_err {:.3f}'.format(self.th, base_prim_id, l_err, th_err))

        
        
class MPrimFactory:
    grid_resolution = 0.025
    th_nb  = 16               # heading discretization
    th_res = 2*math.pi/th_nb  # heading resolution

    def __init__(self, base_prims, grid_resolution=0.025):
        MPrimFactory.grid_resolution = grid_resolution
        self.base_prims = base_prims
        self.nb_mprim_per_angle = len(self.base_prims)

    def build(self, start_angle=0, end_angle=th_nb):
        self.mprims = []
        for angle_c in range(start_angle, end_angle):
            for bpid, bp in enumerate(self.base_prims):
                self.mprims.append(bp['kind'](bpid, angle_c, **bp['params']))

    def write(self, out_path):
        LOG.info(' writing motion primitives to {}'.format(out_path))
        with open(out_path, 'w') as f:
            f.write('resolution_m: {:f}\n'.format(MPrimFactory.grid_resolution))
            f.write('numberofangles: {:d}\n'.format(MPrimFactory.th_nb))
            f.write('totalnumberofprimitives: {:d}\n'.format(MPrimFactory.th_nb*self.nb_mprim_per_angle))
            for mprim in self.mprims:
                f.write(mprim.to_string())

    def plot(self, ngrid=50, show_angle=None, show_prim_id=None):
         plt.rc('grid', linestyle="-", color='black')
         minc, maxc = -ngrid*MPrimFactory.grid_resolution, ngrid*MPrimFactory.grid_resolution
         ax = plt.gca()
         ax.set_xlim([minc,maxc]); ax.set_ylim([minc,maxc])
         minor_ticks = np.arange(minc, maxc, MPrimFactory.grid_resolution)
         major_ticks = np.arange(minc, maxc, 4*MPrimFactory.grid_resolution)
         ax.set_xticks(major_ticks); ax.set_yticks(major_ticks)
         ax.set_xticks(minor_ticks, minor=True)
         ax.set_yticks(minor_ticks, minor=True)
         plt.axes().set_aspect('equal')
         ax.grid(which='minor', alpha=0.2)                                                
         ax.grid(which='major', alpha=0.5)
         for p in self.mprims:
             if show_angle is None or p.th_c in show_angle:
                 if show_prim_id is None or p.base_prim_id in show_prim_id:
                     plt.scatter([p.start_pt[0], p.end_pt[0]], [p.start_pt[1], p.end_pt[1]], marker=(5, 2)) 
                     #plt.scatter([p.start_pt[0], p.end_pt_grid[0]], [p.start_pt[1], p.end_pt_grid[1]]) 
                     plt.plot(p.interm_pts[:,0], p.interm_pts[:,1], '.-')
                     # check _c
                     plt.scatter([p.start_pt_c[0]*self.grid_resolution, p.end_pt_c[0]*self.grid_resolution],
                                 [p.start_pt_c[1]*self.grid_resolution, p.end_pt_c[1]*self.grid_resolution], marker=(5, 2)) 
                     # check interm thetas
                     a_len = 0.1
                     for ip in p.interm_pts:
                         plt.arrow(ip[0], ip[1], a_len * math.cos(ip[2]), a_len * math.sin(ip[2]),
                                   head_width=0.1*a_len, head_length=0.1*a_len, fc='k', ec='k')

                    

def check_all_dirs(f, show_prim_id):
    for a in range(MPrimFactory.th_nb):
        f.plot(show_angle=[a], show_prim_id=show_prim_id)
        plt.show()


def gen_oscar_prims():
    base_prims = [
        {'kind':MPrim_line, 'params':{'len_c': 1, 'cost':2}},            # forward straight short
        {'kind':MPrim_line, 'params':{'len_c': 8, 'cost':1}},            # forward straight long
        #{'kind':MPrim_line, 'params':{'len_c':-1, 'cost':5}},            # backward straight short
        {'kind':MPrim_arc,  'params':{'R':0.15,  'dth_c': 1, 'cost':4}}, # forward sharp turning left
        {'kind':MPrim_arc,  'params':{'R':0.15,  'dth_c':-1, 'cost':4}}, # forward sharp turning right
        {'kind':MPrim_arc,  'params':{'R':0.3,  'dth_c': 1, 'cost':3}},  # forward wide turning left
        {'kind':MPrim_arc,  'params':{'R':0.3,  'dth_c':-1, 'cost':3}},  # forward wide turning right
        #{'kind':MPrim_arc,  'params':{'R':-0.15, 'dth_c':-1, 'cost':6}}, # backward turning left
        #{'kind':MPrim_arc,  'params':{'R':-0.15, 'dth_c': 1, 'cost':6}}  # backward turning right
    ]
    f = MPrimFactory(base_prims, grid_resolution=0.005)
    f.build()
    f.write('/home/poine/work/oscar.git/oscar/oscar_navigation/params/sbpl/oscar.mprim')
    
def gen_julie_prims():
    base_prims = [
        {'kind':MPrim_line, 'params':{'len_c': 1, 'cost':2}},          # forward straight short
        {'kind':MPrim_line, 'params':{'len_c': 8, 'cost':1}},          # forward straight long
        {'kind':MPrim_line, 'params':{'len_c':-1, 'cost':5}},          # backward straight short
        {'kind':MPrim_arc,  'params':{'R':3.,  'dth_c': 1, 'cost':3}}, # forward turning left
        {'kind':MPrim_arc,  'params':{'R':3.,  'dth_c':-1, 'cost':3}}, # forward turning right
        {'kind':MPrim_arc,  'params':{'R':-3., 'dth_c':-1, 'cost':6}}, # backward turning left
        {'kind':MPrim_arc,  'params':{'R':-3., 'dth_c': 1, 'cost':6}}  # backward turning right
    ] 
    f = MPrimFactory(base_prims, grid_resolution=0.025)
    
if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    np.set_printoptions(precision=3, linewidth=300)
    # julie
    #f = MPrimFactory(grid_resolution=0.025)
    # oscar
    gen_oscar_prims()
    #f = MPrimFactory(grid_resolution=0.005)

    #f.build()#start_angle=0, end_angle=1)
    #f.write('/tmp/foo.mprim')
    #check_all_dirs(f, show_prim_id=[3])
    #f.plot(show_angle=[15], show_prim_id=[5, 6])
    #plt.show()

    
