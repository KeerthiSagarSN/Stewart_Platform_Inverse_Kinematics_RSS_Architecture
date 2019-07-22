# -*- coding: utf-8 -*-
"""
Created on Wed Dec 07 16:23:07 2016

@author: keerthi
"""
# Library funtions
import numpy as np
import matplotlib.pyplot as plt
import os
import math
import array
#import matplotlib.pyplot as plt
 
import matplotlib.cm as cm

#import sklearn.neighbors 
import scipy as scipy
#from scipy import spatial
import scipy.spatial.distance as ndist
import sys
from gurobipy import *
from scipy.spatial import distance
from matplotlib.lines import Line2D 
# To create wait functions in time
import time  
# Inverse kinematics
from linearalgebra import V_unit
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import norm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
# Symbolic computation
# Creating Reference frame 
from sympy.physics.vector import ReferenceFrame, dot, cross

# Visualization library

from pydy.viz.shapes import Cylinder, Sphere

from pydy.viz.visualization_frame import VisualizationFrame
from pydy.viz.scene import Scene

# LAtex printing library

from sympy import init_printing
init_printing(use_latex='mathjax',pretty_print = False)
from sympy.solvers import solve
# Importing the symbol libraruy

from sympy import symbols
# Scalar variables imported from Sympy.abc library
from sympy.abc import  theta,alpha
from sympy import sin,cos,tan,pi,acos,asin

#N = ReferenceFrame('N')
#A = N.orientnew('A','Axis',(theta,N.z))

#import matplotlib.pyplot as plt
# Import the linear algebra function file linearalgebra.py

import linearalgebra

# Import the visualization library

from visualbuild import revolute,link

# Read data from input automaton
for loopn in xrange(1,2):
    geodata = np.load('inverseinputparameters'+str(loopn)+'.'+'npz')
    
    geodata.files
    # Refer input automaton for details of variable naming
    d_a_val = geodata['xx']
    d_b_val = geodata['yy']
    d_c_val = geodata['zz']
    # Distance between normalized unit twist between joint "1" and joint "2" of leg "A"
    l_12_A_val = geodata['aa']
    # Distance between normalized unit twist between joint "1" and joint "2" of leg "C"
    l_12_C_val = geodata['bb']
    # "j" coorindate of ""P_4_A" --> Projection of "P_2_A" --> Axis line of revolute joint "4" in  leg "A"
    p_A_val = geodata['pa']
    # "i" coorindate of ""P_5_B" --> Projection of "P_1_B" --> Axis line of revolute joint "5" in  leg "B"
    p_B_val = geodata['pb']
    # "j" coorindate of ""P_4_C" --> Projection of "P_2_C" --> Axis line of revolute joint "4" in  leg "C"    
    p_C_val = geodata['pc']
    # "distance of axis of joint "4" of leg "A" to plane containing axis of joint "5" of leg "B" and axis of joint "4" of leg "A & C""
    h_A_val = geodata['ha']
    # "distance of axis of joint "4" of leg "C" to plane containing axis of joint "5" of leg "B" and axis of joint "4" of leg "A & C""
    h_C_val = geodata['hc']
    # perpendicular distance between the platform and the plane pi_0
    h_z_val = geodata['hz']
    # X - axis distance between the platform and the plane pi_0
    h_x_val = geodata['hx']
    # Coordinates of the spherical joint centre/ point - Main input variant - Change this to obtain the new inverse kinematics position
    S_x_val = geodata['sx']
    S_y_val = geodata['sy']
    S_z_val = geodata['sz']
    
# Initiating the base reference frame
    P_1_A = [0,-d_a_val,0]
    P_1_B = [d_b_val,0,0]
    P_1_C = [0,d_c_val,0]
#    
#    
## Unit vector parallel to the axis A and C k_1_A = k_1_C = j_b
#    k_1_A = [0,1,0]
#    k_1_C = [0,1,0]
    
# Calculation of "e" from "S"
# Numerical example


    # Calculation of cos(alpha)
#    del_1_b = 1.00
#    t_0 = math.sqrt(math.pow(S_x,2) + math.pow(S_z,2) )
#    ca = (-del_1_b*S_x*( math.sqrt((math.pow(t_0,2) - math.pow(h_x,2) ))) + (h_x*S_z))/(math.pow(t_0,2))
#    # Calculation of sin(alpha)
#    sa = (del_1_b*S_z*( math.sqrt((math.pow(t_0,2) - math.pow(h_x,2) ))) + (h_x*S_x))/(math.pow(t_0,2))
#    # Calculation of sin(beta)
#    sb = (-del_2_b*S_y*)/(math.pow(t_0,2))
    
# Visualising the joints
#v_origin = [0,0,0]    
#p0 = [0,-100,0]
#p1 = [0,0,0]
#r_cylinder = 10
#extr = 25
#
## x,y,z coordinates for the revolute joint
##x_cyl, y_cyl, z_cyl,x_start, y_start, z_start,x_end, y_end, z_end,cyl_unit = cylinder(p0,p1,r_cylinder,extr)
#
#
## Visualizing the cylinder
#fig = plt.figure()    
#ax = Axes3D(fig)
##ax.set_alpha = 1
##ax = fig.add_subplot(111, projection='3d')
#
#
#
#
#
#
#
## Link coorindates
#p0 = np.array(p0)
#pl = p0 + (extr/2)*(cyl_unit)
#pm = [1,0,0]
#pm = np.array(pm)
#v_axis = np.cross(-1*cyl_unit,pm)
#l = extr/3
#w = extr/3
#extr = 200
#x_link,y_link,z_link = link(pl,v_axis,l,w,extr)
#    
#verts = []
#for i in xrange(0,21,4):
#    
#    verts += [zip(x_link[i:i+4],y_link[i:i+4],z_link[i:i+4])]
#
#    
##ax.add_collection3d(Poly3DCollection(verts))
##ax.add_collection3d(Poly3DCollection(x_cyl, y_cyl, z_cyl))
##ax.add_collection3d(Poly3DCollection(x_start, y_start, z_start))
##ax.add_collection3d(Poly3DCollection(x_end, y_end, z_end))
#
#
##plt.show()    
#
#'''
#ax.plot(*zip(p0, p1), color = 'red')
#ax.plot_surface(x_cyl, y_cyl, z_cyl)
#ax.plot_surface(x_cyl, y_cyl, z_cyl,  rstride=1, cstride=1, color='red', linewidth=10, alpha=1)
#ax.plot_surface(x_start, y_start, z_start)
#ax.plot_surface(x_end, y_end, z_end)    
#'''
#
## Reference frame declaration
#
#O = ReferenceFrame('O')
#
## Creating the symbols
#
## Distance of the revolute joints from the Reference frame
#d_a = symbols("d^a")
#d_b = symbols("d^b")
#d_c = symbols("d^c")
#
## Vector of the first revolute joint
## Creating the symbols for representation
#P_1_A = symbols("P_1^A")
#P_1_B = symbols("P_1^B")
#P_1_C = symbols("P_1^C")
#
## First Revolute joint of leg A
#a = []
#
#P_1_A = 0*O.x + (-d_a_val)*O.y + 0*O.z
#
## First Revolute joint of leg C
#P_1_B = d_b_val*O.x + 0*O.y + 0*O.z
#
## First Revolute joint of leg C
#P_1_C = 0*O.x + d_c_val*O.y + 0*O.z
#
#
#
## Displaying the revolute joint 
#
##P_1_A_disp = revolute()
## Substitute 
## Creating the symbols for representation
#l_12_A = symbols("l_{12}^A")
#
#
#P_2_A = l_12_A*cos(alpha)*O.x + (-d_a_val)*O.y + l_12_A*sin(alpha)*O.z
#
##l_12_A = l_12_A_val*cos(theta)*O.x 
#v_orig = 0*O.x + 0*O.y + 0*O.z
## Displacement vector of link 12 in leg  A
#v_1_A = symbols("v_{1}^A")
#v_1_A = P_1_A - v_orig
#
#
## Displacement vector of link 12 in leg  A
#v_12_A = symbols("v_{12}^A",real = True)
#v_12_A = P_2_A - P_1_A
#
#V_12_A = v_12_A.to_matrix(O)
#
## Calling linear algebra library
#
#nu_12_A = V_unit(V_12_A)
#
#
#
## Displacement vector of link 23 in leg  A
#   
##P_3_A = n_12_A*cos(theta).O.x + 
#
#
#l_23_A = symbols("l_{23}^A")
#


S = np.array([S_x_val,S_y_val,S_z_val])


# To build revolute joint
#rev1 = P_2_A.to_matrix(O)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
#X = [P4A[0],P4C[0],P3B[0],P4A[0]]
#Y = [P4A[1],P4C[1],P3B[1],P4A[1]]
#Z = [P4A[2],P4C[2],P3B[2],P4A[2]]
#ax.plot(X,Y,Z)
O_orig = [0,0,0]

ax.scatter(O_orig[0],O_orig[1],O_orig[2],color = 'c')

# COnversion of S to e

delta_B1 = 1
delta_B2 = 1


S_x_val = S_x_val + 10
ax.scatter(S_x_val,S_y_val,S_z_val)
# Temp variables

to = math.sqrt(S_x_val*S_x_val + S_z_val*S_z_val )

c_alpha  = ((-delta_B1*S_x_val*math.sqrt(pow(to,2) - pow(h_x_val,2)) + h_x_val*S_z_val)/pow(to,2))
s_alpha  = ((delta_B1*S_z_val*math.sqrt(pow(to,2) - pow(h_x_val,2)) + h_x_val*S_x_val)/pow(to,2))


l_plane_x = [0,0,500,500,0]
l_plane_y = [0,500,500,0,0]
l_plane_z = [0,0,0,0,0]
#for i in range(int(len(l_plane_x))):
ax.plot(l_plane_x,l_plane_y,l_plane_z)

l_ori_ix = [0,250]
l_ori_iy = [0,0]
l_ori_iz = [0,0]

ax.plot(l_ori_ix,l_ori_iy,l_ori_iz,color = 'r')


l_ori_jx = [0,0]
l_ori_jy = [0,250]
l_ori_jz = [0,0]

ax.plot(l_ori_jx,l_ori_jy,l_ori_jz,color = 'r')


l_ori_kx = [0,0]
l_ori_ky = [0,0]
l_ori_kz = [0,250]

ax.plot(l_ori_kx,l_ori_ky,l_ori_kz,color = 'r')

k2_A = [s_alpha,0,c_alpha]

ax.scatter(k2_A[0],k2_A[1],k2_A[2],color = 'g',s = 30)
ax.scatter(P_1_A[0],P_1_A[1],P_1_A[2])
ax.scatter(P_1_B[0],P_1_B[1],P_1_B[2])
ax.scatter(P_1_C[0],P_1_C[1],P_1_C[2])

P_1_B_S = S - P_1_B


e_h = np.cross(k_5_B,k2_A)

t_6 = (delta_B1*(pow(to,2) - d_b_val*S[0])*math.sqrt(pow(to,2) - pow(h_x_val,2)) + d_b_val*h_x_val*S_z_val)/pow(to,2)
s_beta = -delta_B2*S[1]/math.sqrt(pow(t_6,2) + pow(S[1],2))
c_beta = delta_B2*t_6/math.sqrt(pow(t_6,2) + pow(S[1],2))


#k_5_B = np.cross(P_1_B_S,k2_A)

k_5_B = [-s_beta*c_alpha,c_beta,s_beta*s_alpha]

P = S - h_x_val*k2_A - h_z_val*e_h*[S[0] + (h_z_val*c_beta*c_alpha) - (h_x_val*s_alpha), S[1] + h_z_val*s_beta, S[2] - (h_z_val*c_beta*s_alpha) - (h_x_val*c_alpha)]
#ax.plot(k2_A_i,k2_A_j,k2_A_k,color = 'b')
ax.scatter(P[0],P[1],P[2])

#
#c_alpha  = (((S_x_val*math.sqrt(pow(to,2) - pow(h_x_val,2)))) - ((delta_B1)*h_x_val*S_z_val))/pow(to,2)
#s_alpha  = (((S_z_val)*(math.sqrt(pow(to,2) - pow(h_x_val,2))) + delta_B1*h_x_val*S_x_val)/pow(to,2))

#
## Displacement Vector
#la = P4A - P1A
#lb = P3B - P1B
#lc = P4C - P1C
#
#lad = V_mod(la)
#lbd = V_mod(lb)
#lcd = V_mod(lc)
#
#ax.scatter(Sb[0],Sb[1],Sb[2],'.')
#ax.scatter(Ob[0],Ob[1],Ob[2],'.')
#ax.scatter(P1B[0],P1B[1],P1B[2],'.')
#ax.scatter(P1A[0],P1A[1],P1A[2],'.')
#ax.scatter(P1C[0],P1C[1],P1C[2],'.')
#
#ax.plot([P4A[0],P1A[0]],[P4A[1],P1A[1]],[P4A[2],P1A[2]])
#ax.plot([P4C[0],P1C[0]],[P4C[1],P1C[1]],[P4C[2],P1C[2]])
#ax.plot([P3B[0],P1B[0]],[P3B[1],P1B[1]],[P3B[2],P1B[2]])    

    





