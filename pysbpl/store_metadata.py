#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 17 23:53:09 2021

@author: alrick
"""
#Script for saving files in steering angle and speed.
import math, json

class Store_metadata_mprims:
    """
    Class to store steering angle, speed, and travel time from the motion 
    primitives.
    """
    
    def __init__(self, wheel_base=0.225, save_path=None):
        """
        Initliazing class params.
        """
        self.save_path = "metadata.json" if not save_path else save_path
        self.mprims_map = {}
        self.wheel_base = wheel_base
        
    def add_mprim(self, base_prim):
        """
        Processing the base_prim from MPrimFactory to output steering angle,
        and travel time (path len / speed). Speed is fixed to be 0.5 m/s.
        Input: base_prim: Class MPrim_arc (will later extend to MPrim_line)
        
        Formula to calculate the steering angle.
        Link: https://github.com/prl-mushr/mushr/blob/master/mushr_description/kinematic_car_model.pdf
        """
        
        t = str(tuple([base_prim.th_curr] + list(base_prim.end_pt_c)))
        
        if (t not in self.mprims_map):
            speed = 0.5
            
            #Computing steering angle.
            beta = math.asin(self.wheel_base / (2 * base_prim.R))
            st_angle = math.atan( 2 * math.tan(beta) )
            
            #Compute path lenght, then convert to
            #Have to check for angles greater than pi.
            # d = math.sqrt(sum(x**2 for x in base_prim.end_pt_c[:2]))*base_prim.grid_resolution
            # print ("Before Path len: ", d, base_prim.R, 2 * base_prim.R**2 -  d**2, \
            #        ( 2 * base_prim.R**2 -  d**2) / (2 * base_prim.R**2), (base_prim.end_pt_c[-1] * base_prim.th_res))
            # path_len = math.acos( ( 2 * base_prim.R**2 -  d**2) / (2 * base_prim.R**2) ) * base_prim.R
            
            #Path len (difference in start and end angle (in radians)) * radius
            path_len = abs((base_prim.end_pt_c[-1] * base_prim.th_res - \
                            base_prim.th) * base_prim.R * base_prim.grid_resolution)
                
            #Store mprims
            self.mprims_map[t] = {"speed":speed, "steering_angle":st_angle, \
                                  "time":path_len/speed}
                
    def save(self):
        """
        Save self.mprims_map in a json file at self.save_path.
        """
        print ("\nSaving Metadata.")
        with open(self.save_path, 'w') as file:
            json.dump(self.mprims_map, file, sort_keys=False, indent=4)
            