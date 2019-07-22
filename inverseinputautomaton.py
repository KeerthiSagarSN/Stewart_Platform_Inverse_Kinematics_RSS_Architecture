# -*- coding: utf-8 -*-
"""
Created on Wed Dec 07 16:23:07 2016

@author: keerthi
"""

# Inverse kinematics
import numpy as np
for loopn in xrange(1,2):
    # Geometrical Input parameters
    
    # Geometrical properties of the base- Mouting points distance of the legs A,B and C - ALl values in mm Only magnitude are given not direction - (Angle)
    d_a = 0.4434*1000
    d_b = 0.3455*1000
    d_c = 0.7798*1000
    # Distance between normalized unit twist between joint "1" and joint "2" of leg "A"
    l_12_A = 0.1023*1000
    # Distance between normalized unit twist between joint "1" and joint "2" of leg "C"
    l_12_C = 0.1523*1000
    # "j" coorindate of ""P_4_A" --> Projection of "P_2_A" --> Axis line of revolute joint "4" in  leg "A"
    p_A = 0.1523*1000
    # "i" coorindate of ""P_5_B" --> Projection of "P_1_B" --> Axis line of revolute joint "5" in  leg "B"
    p_B = 0.1324*1000
    # "j" coorindate of ""P_4_C" --> Projection of "P_2_C" --> Axis line of revolute joint "4" in  leg "C"    
    p_C = 0.2523*1000
    # "distance of axis of joint "4" of leg "A" to plane containing axis of joint "5" of leg "B" and axis of joint "4" of leg "A & C""
    h_A = 0.04*1000
    # "distance of axis of joint "4" of leg "C" to plane containing axis of joint "5" of leg "B" and axis of joint "4" of leg "A & C""
    h_C = 0.023*1000
    # perpendicular distance between the platform and the plane pi_0
    h_z = 0.2*1000
    # X - axis distance between the platform and the plane pi_0
    h_x = 0.2828*1000
    # Coordinates of the spherical joint centre/ point - Main input variant - Change this to obtain the new inverse kinematics position
    S_x = 0.02*1000
    S_y = 0.7*1000
    S_z = 1.02*1000
    
    np.savez('inverseinputparameters'+str(loopn),xx = d_a,yy = d_b,zz = d_c,aa = l_12_A,bb = l_12_C,pa = p_A,pb = p_B,pc= p_C,ha = h_A,hc = h_C,hz = h_z,hx = h_x,sx = S_x,sy = S_y,sz = S_z)


