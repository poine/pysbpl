# distutils: language = C++
#
#
# PySBPL is a python SBPL interface using cython 
#
# Author: Poine-2017
#
import numpy as np

from libc.stdlib cimport malloc, free
from libcpp cimport bool
from libcpp.string cimport string
from libcpp.vector cimport vector
from cython.operator cimport dereference as deref, preincrement as inc

cdef extern from "sbpl/utils/utils.h":
    cdef cppclass c_sbpl_2Dpt_t "sbpl_2Dpt_t":
        c_sbpl_2Dpt_t()
        double x;
        double y;

cdef extern from "sbpl/utils/utils.h":
    cdef cppclass c_sbpl_xy_theta_pt_t "sbpl_xy_theta_pt_t":
        c_sbpl_xy_theta_pt_t()
        double x;
        double y;
        double theta;
        
cdef extern from "sbpl/discrete_space_information/environment.h":
    cdef cppclass c_DiscreteSpaceInformation "DiscreteSpaceInformation":
        pass
#
# EnvironmentNAVXYTHETALAT
#
cdef extern from "sbpl/discrete_space_information/environment_navxythetalat.h":
    cdef cppclass c_EnvironmentNAVXYTHETALAT "EnvironmentNAVXYTHETALAT":
        c_EnvironmentNAVXYTHETALAT()
        bool InitializeEnv(const char* sEnvFile)
        bool InitializeEnv(int width, int height,
                           const unsigned char* mapdata,
                           double startx, double starty, double starttheta,
                           double goalx, double goaly, double goaltheta,
                           double goaltol_x, double goaltol_y, double goaltol_theta,
                           const vector[c_sbpl_2Dpt_t]& perimeterptsV, double cellsize_m,
                           double nominalvel_mpersecs, double timetoturn45degsinplace_secs,
                           unsigned char obsthresh, const char* sMotPrimFile)
        int SetStart(double x_m, double y_m, double theta_rad)
        int SetGoal(double x_m, double y_m, double theta_rad)
        void GetCoordFromState(int stateID, int& x, int& y, int& theta)
        void ConvertStateIDPathintoXYThetaPath(vector[int]* stateIDPath, vector[c_sbpl_xy_theta_pt_t]* xythetaPath)
        bool SetEnvParameter(const char* parameter, int value)
       
cdef class EnvironmentNAVXYTHETALAT:
    cdef c_EnvironmentNAVXYTHETALAT *thisptr      # hold a C++ instance which we're wrapping

    def __cinit__(self):
        self.thisptr = new c_EnvironmentNAVXYTHETALAT()
        if self.thisptr is NULL:
            raise MemoryError()

    def InitializeEnv(self, **kwargs):
        
        if 'cfg_filename' in kwargs:
            self.initialize_from_file(kwargs['cfg_filename'])
        else:
            self.initialize_from_params(**kwargs)
        
        self.thisptr.SetEnvParameter("cost_obsthresh", kwargs['obs_thresh'])
        self.thisptr.SetEnvParameter("cost_inscribed_thresh", kwargs['inscribed_thresh'])
        self.thisptr.SetEnvParameter("cost_possibly_circumscribed_thresh", kwargs['possibly_circumscribed_thresh'])
        
        # not sure how to get that without setting them again, or is it alsways 0 and 1 ?
        cdef double startx = kwargs['start'][0], starty = kwargs['start'][1], starttheta = kwargs['start'][2]
        cdef double goalx = kwargs['goal'][0], goaly = kwargs['goal'][1], goaltheta = kwargs['goal'][2]
        start_id = self.thisptr.SetStart(startx, starty, starttheta)
        goal_id = self.thisptr.SetGoal(goalx, goaly, goaltheta)
        return start_id, goal_id


    def initialize_from_file(self, filename):
        print "loading environment from {}".format(filename)
        self.thisptr.InitializeEnv(filename)
        
    def initialize_from_params(self, **kwargs):
        _map = kwargs['map']
        cdef int width = _map.width, height = _map.height

        cdef unsigned char* mapdata
        mapdata = <unsigned char *>malloc(width*height*sizeof(unsigned char))
        for i, pixel_row in enumerate(_map.img[::-1]):  # same as flipud
            for j in range(len(pixel_row)):
                mapdata[i*width + j] =  int(pixel_row[j]*255)
            
                
        cdef double startx = kwargs['start'][0], starty = kwargs['start'][1], starttheta = kwargs['start'][2]
        cdef double goalx = kwargs['goal'][0], goaly = kwargs['goal'][1], goaltheta = kwargs['goal'][2]
        cdef goaltol_x =  kwargs['goal_tol'][0], goaltol_y = kwargs['goal_tol'][1], goaltol_theta = kwargs['goal_tol'][2]
        cdef vector[c_sbpl_2Dpt_t] perimeter
        cdef c_sbpl_2Dpt_t pt;
        for p in kwargs['perimeter']:
            pt.x, pt.y = p 
            perimeter.push_back(pt)
        cdef double cellsize_m = kwargs['map'].resolution
        cdef double nominalvel_mpersecs = kwargs['vel'], timetoturn45degsinplace_secs = kwargs['time_45_deg']
        cdef unsigned char obsthresh =  kwargs['obs_thresh']
        cdef const char* c_mprim_path = kwargs['mprim_path']
        res = self.thisptr.InitializeEnv(width, height, mapdata,
                                         startx, starty, starttheta, goalx, goaly, goaltheta,
                                         goaltol_x, goaltol_y, goaltol_theta,
                                         perimeter, cellsize_m,
                                         nominalvel_mpersecs, timetoturn45degsinplace_secs,
                                         obsthresh, c_mprim_path)
        free(mapdata)
        if not res: return []
        

#
# SBPLPlanner
#
#cdef extern from "sbpl/planners/planner.h":
#    cdef cppclass c_SBPLPlanner "SBPLPlanner":
#        c_SBPLPlanner()

#
# ARAPlanner
#
#
cdef extern from "sbpl/planners/araplanner.h":
    cdef cppclass c_ARAPlanner "ARAPlanner":
        c_ARAPlanner(c_DiscreteSpaceInformation* environment, bool bforwardsearch)
        int set_start(int start_stateID)
        int set_goal(int goal_stateID)
        void set_initialsolution_eps(double initialsolution_eps)
        int set_search_mode(bool bSearchUntilFirstSolution);
        int replan(double allocated_time_secs, vector[int]* solution_stateIDs_V)

cdef class ARAPlanner:
    cdef c_ARAPlanner *thisptr
    cdef c_EnvironmentNAVXYTHETALAT *env

    def __cinit__(self, EnvironmentNAVXYTHETALAT e):
        cdef bool bsearch = True;
        self.thisptr = new c_ARAPlanner(<c_DiscreteSpaceInformation*>e.thisptr, bsearch)
        self.env = <c_EnvironmentNAVXYTHETALAT*>e.thisptr

    def initialize(self, start_id, goal_id):
        if self.thisptr.set_start(start_id) == 0:
            print 'set_start failed'
        if self.thisptr.set_goal(goal_id) == 0:
            print 'set_goal failed'
        cdef double initialEpsilon = 3.0
        self.thisptr.set_initialsolution_eps(initialEpsilon)
        cdef bool bsearchuntilfirstsolution = True;
        self.thisptr.set_search_mode(bsearchuntilfirstsolution);
        
    def run(self):
        cdef double allocated_time = 30.
        cdef vector[int] sol
        cdef int bRet = self.thisptr.replan(allocated_time, &sol)
        if bRet != 1: 
            print("No solution!")
            return None, None
        else:
            print("Solution Found!")
        # discrete solution
        cdef vector[int].iterator it = sol.begin()
        cdef int x=0, y=0, theta=0
        while it != sol.end():
            self.env.GetCoordFromState(deref(it), x, y, theta)
            #print deref(it), x, y, theta
            inc(it)

        # continuous solution
        cdef vector[c_sbpl_xy_theta_pt_t] xythetaPath;
        self.env.ConvertStateIDPathintoXYThetaPath(&sol, &xythetaPath);
        cdef vector[c_sbpl_xy_theta_pt_t].iterator it2 = xythetaPath.begin()
        xys, thetas = [], []
        while it2 != xythetaPath.end():
            #print deref(it2).x, deref(it2).y, deref(it2).theta
            xys.append([deref(it2).x, deref(it2).y])
            thetas.append(deref(it2).theta)
            inc(it2)
        print('continuous solution has {} values'.format(len(xys)))

        return np.array(xys), np.array(thetas)
