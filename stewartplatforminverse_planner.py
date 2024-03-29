# -*- coding: utf-8 -*-
"""
Created on Mon Apr 24 11:50:18 2017

Main planner for Stewart platform (RSS - Revolute - SPherical- Spherical) architecture


@author: Keerthi
"""

############## Library #######################################
import numpy as np
import matplotlib.pyplot as plt
import os
import math
import array


import scipy.spatial.distance as ndist
import sys
#from gurobipy import *
from scipy.spatial import distance
from matplotlib.lines import Line2D 
# To create wait functions in time
import time  
# Inverse kinematics
from linearalgebra import V_unit
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import norm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from scipy.optimize import fsolve
import scipy as scipy

# Importing the symbol libraruy

from sympy.solvers import nsolve
from sympy.solvers import solve
from sympy import Symbol
from sympy import Matrix
from numpy import deg2rad
import sympy



from linearalgebra import *

# Import the visualization library

from visualbuild import revolute,link

from sympy.physics.vector import *
from sympy.physics.mechanics import ReferenceFrame, Vector, dot

from math import pi as Pi


plt.ion()
# Base points declaration as points in base coordinate system



################################# Input data ##########################################################
# 
point_gen = [[0,0,0,0,0,350],[0,0,0,0,0,400],[0,0,0,0,0,350],[0,0,0,0,0,300],[0,0,0,0,0,350],[0,0,0,0,100,350],[0,0,0,0,-100,350]
,[0,0,0,100,0,350],[0,0,0,-100,0,350],[0,0,0,0,0,350],[5,0,0,0,0,350],[-5,0,0,0,0,350],[0,0,0,0,0,350],
[0,5,0,0,100,350],[0,-5,0,0,0,350],[0,0,0,0,0,350],[0,0,5,0,0,350],[0,0,-5,0,0,350],[0,0,0,0,0,350]] # [Yaw,pitch,roll,surge,sway,heave] - Each array represents one pose





ob = np.zeros(shape = [3,1])

ob_s_a = np.zeros(shape= [3,1])
# Length from of all base points from the centre (Base coordinate system)
l_ob = 573.8500
# Starting base axis vector
ob_s_a[0,0] = ob[0,0]+l_ob
ob_s_a[1,0] = ob[1,0]
ob_s_a[2,0] = ob[2,0]

# Length of crank in 'mm'

l_cr  = 190

# Length of connecting rod in 'mm'

l_rod = 450
# Try to write a function for this
# Rotation angle for the base axis - 120 degrees each - Counterclockwise direction
rotangle_ba = -120

# Declaring the 3x3 matrix for the base axis - 120 degrees apart - Each column represents each base axis point 
ob_axis = np.zeros(shape = [3,3])
for i in xrange(3):
    # Rotation around the z- axis
    rotangle_ba = rotangle_ba + 120
    # Rotating the first axis point around the z-axis
    ob_axis_dummy = []
    ob_axis_dummy = R_Z(rotangle_ba)*ob_s_a
    for j in  xrange(len(ob_axis_dummy)):
        ob_axis[j,i] = ob_axis_dummy[j]

# Base points from the base axis - Alfa is the angle from the base centre

# All points at an angle of 15 degrees from their corresponding base axis
angle_alfa = np.degrees(np.arctan(60.000/573.85000))

# Odd side of the servos - Counterclockwise direction from the base axis
# Each column is the base vector 
ob_odd = np.zeros(shape = [3,3])

for i in xrange(3):
    ob_axis_dum = np.zeros(shape = [3,1])
    for k in xrange(len(ob_axis)):
        ob_axis_dum[k,0] = ob_axis[k,i]
    ob_odd_dummy = R_Z(angle_alfa)*ob_axis_dum
    for j in xrange(len(ob_odd_dummy)):
        ob_odd[j,i] = ob_odd_dummy[j]


# Even side of the servos - Counterclockwise direction from the base axis
# Each column is the base vector 
ob_even = np.zeros(shape = [3,3])

for i in xrange(3):
    ob_axis_dum = np.zeros(shape = [3,1])
    for k in xrange(len(ob_axis)):
        ob_axis_dum[k,0] = ob_axis[k,i]
    ob_even_dummy = R_Z(-1*angle_alfa)*ob_axis_dum
    for j in xrange(len(ob_even_dummy)):
        ob_even[j,i] = ob_even_dummy[j]


# The main crank at the horizontal positions is calculated here

# For all odd cranks
oc_first_odd = np.zeros(shape = [3,1])
oc_first_odd_axis = np.zeros(shape = [3,1])
oc_per_even = np.zeros(shape = [3,1])

# Distance of the crank axis from the origin axis
l_offset = 60
phi = math.acos(l_offset/l_ob) # 60 is the offset of the motors from the central axis
h_crank = (np.sin(phi))*l_ob
for i in xrange(3):
    oc_per = np.dot(R_Z(90),V_unit(ob_axis[:,i])) # Unit vector
    oc_per_scale = l_offset*oc_per
    oc_per_even = np.hstack((oc_per_even,np.transpose(oc_per_scale)))
    # the vector from the crank perpendicular to the actual fixed point
    v_final_axis = ob_odd[:,i] - l_offset*(oc_per)
    oc_first_odd_axis = np.hstack((oc_first_odd_axis,np.transpose(v_final_axis)))
    oc_vec = ob_odd[:,i] + l_cr*oc_per
    oc_first_odd = np.hstack((oc_first_odd,np.transpose(oc_vec)))
    
oc_first_odd = oc_first_odd[:,1:]
oc_first_odd_axis = oc_first_odd_axis[:,1:]
oc_per_even = oc_per_even[:,1:]
# For all axis of the crank


    
    
# For all even cranks
oc_first_even = np.zeros(shape = [3,1])
oc_first_even_axis = np.zeros(shape = [3,1])
oc_per_odd = np.zeros(shape = [3,1])
for i in xrange(3):
    oc_per = np.dot(R_Z(-90),V_unit(ob_axis[:,i]))
    oc_per_scale = l_offset*oc_per
    oc_per_odd = np.hstack((oc_per_odd,np.transpose(oc_per_scale)))
    # the vector from the crank perpendicular to the actual fixed point
    v_final_axis = ob_even[:,i] - l_offset*(oc_per)
    oc_first_even_axis = np.hstack((oc_first_even_axis,np.transpose(v_final_axis)))
    oc_vec = ob_even[:,i] + l_cr*oc_per
    oc_first_even = np.hstack((oc_first_even,np.transpose(oc_vec)))
    
    
oc_first_even = oc_first_even[:,1:]
oc_first_even_axis = oc_first_even_axis[:,1:]    
oc_per_odd = oc_per_odd[:,1:]


# Unifying odd and even arrays
def uniarray(even_arr,odd_arr):
    new_arr = np.zeros(shape = [1,3])
    new_arr[:,:] = -1
    for k in  xrange(3):
        even_dum = []
        even_dum = np.matrix(even_arr[:,k])
        new_arr = np.append(new_arr,np.asarray(even_dum),axis = 0)
        odd_dum = []
        odd_dum = np.matrix(odd_arr[:,k])
        #op_odd_dum = np.transpose(op_odd_dum)
        new_arr = np.append(new_arr,np.asarray(odd_dum),axis = 0)
    new_arr = new_arr[1:,:]
    return new_arr

def plotarray(old_arr):
    dummy_arr = np.matrix(old_arr[0,:])
    new_arr_plt = np.append(old_arr,np.asarray(dummy_arr),axis = 0)
    return new_arr_plt



# The main horizontal crank array positions in the below matrix
# UNifying all the even and odd arrays into one

oc_first = uniarray(np.asarray(oc_first_even),np.asarray(oc_first_odd))
oc_first_axis = uniarray(np.asarray(oc_first_even_axis),np.asarray(oc_first_odd_axis))
oc_per = uniarray(np.asarray(oc_per_even),np.asarray(oc_per_odd))
op = np.zeros(shape = [3,1])


ang_yaw = 5
ang_pitch = 0
ang_roll = 0
transx = 0
transy = 0
transz = 0
Rotmat = R_Z(ang_yaw)*R_Y(ang_pitch)*R_X(ang_roll)

Transmat = np.zeros(shape = [3,1]) 
Transmat[0,0] = transx
Transmat[1,0] = transy
Transmat[2,0] = transz

op = Rotmat*op + Transmat



# PLatform points - Exact copy of base points

op_s_a = np.zeros(shape= [3,1])
# Length from of all base points from the centre (Base coordinate system)
l_op = 544.500
# Starting base axis vector
op_s_a[0,0] = op[0,0] + l_op
op_s_a[1,0] = op[1,0]
op_s_a[2,0] = op[2,0]





# Rotation angle for the platform axis - 120 degrees each - Counterclockwise direction
rotangle_pa = -120.0

# Declaring the 3x3 matrix for the base axis - 120 degrees apart - Each column represents each base axis point 

op_axis = np.zeros(shape = [3,3])
for i in xrange(3):
    # Rotation around the z- axis
    rotangle_pa = rotangle_pa + 120.0
    # Rotating the first axis point around the z-axis
    op_axis_dummy = []
    op_axis_dummy = R_Z(rotangle_pa)*op_s_a
    for j in  xrange(len(op_axis_dummy)):
        op_axis[j,i] = op_axis_dummy[j]


# Base points from the base axis - Alfa is the angle from the base centre

# All points at an angle of 6 degrees from their corresponding base axis
#angle_alfa = np.math.degrees(np.math.atan(60.0/544.500))
angle_alfa = 5
# Odd side of the servos - Counterclockwise direction from the base axis
# Each column is the base vector 
op_odd = np.zeros(shape = [3,3])

for i in xrange(3):
    op_axis_dum = np.zeros(shape = [3,1])
    for k in xrange(len(op_axis)):
        op_axis_dum[k,0] = op_axis[k,i]
    #axis_dum = np.transpose(np.matrix(op_axis_dum[:,0]))
    #op_axis_fin = axis_dum - op
    op_odd_dummy = np.dot(R_Z(angle_alfa),op_axis_dum)
    for j in xrange(len(op_odd_dummy)):
        op_odd[j,i] = op_odd_dummy[j]


op_even = np.zeros(shape = [3,3])

for i in xrange(3):
    op_axis_dum = np.zeros(shape = [3,1])
    for k in xrange(len(op_axis)):
        op_axis_dum[k,0] = op_axis[k,i]
    #axis_dum = np.transpose(np.matrix(op_axis_dum[:,0]))
    #op_axis_fin = axis_dum - op
    op_even_dummy = np.dot(R_Z(-1*angle_alfa),op_axis_dum)
    for j in xrange(len(op_even_dummy)):
        op_even[j,i] = op_even_dummy[j]




# Rotation is along X - Roll axis, Rotation along Y - Pitch axis, Rotation along Z - yaw axis


def plot_visual(ob_even,ob_odd,op_even,op_odd,oc_first,oc_per,oc_first_axis,op,ob,op_home,ob_fixed):
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    
    ax.set_xlim(-600,600)
    ax.set_ylim(-600,600)
    ax.set_zlim(-100,600)
    
    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_aspect('equal')
    ax.patch.set_facecolor("white")
    

    ax.plot(op_home_plt[:,0],op_home_plt[:,1],op_home_plt[:,2],'.r-')
    ax.plot(ob_fixed_plt[:,0],ob_fixed_plt[:,1],ob_fixed_plt[:,2],'.k-')
    
    
    for k in xrange(len(ob_even)):
        ax.scatter(ob_even[0,k],ob_even[1,k],ob_even[2,k],color = 'c')
        ax.scatter(op_even[0,k],op_even[1,k],op_even[2,k],color = 'c')
        
        
        #ax.scatter(p_even_b[0,k],p_even_b[1,k],p_even_b[2,k],color = 'c')
        
    for k in xrange(len(ob_odd)):
        ax.scatter(ob_odd[0,k],ob_odd[1,k],ob_odd[2,k],color = 'r')
        ax.scatter(op_odd[0,k],op_odd[1,k],op_odd[2,k],color = 'r')
        #ax.scatter(p_odd_b[0,k],p_odd_b[1,k],p_odd_b[2,k],color = 'r')
    
    for k in xrange(6):
        ax.scatter(oc_first[k,0],oc_first[k,1],oc_first[k,2],color = 'cyan')
        
        ax.scatter(oc_first_axis[k,0],oc_first_axis[k,1],oc_first_axis[k,2],color = 'y')
        ax.scatter(oc_per[k,0],oc_per[k,1],oc_per[k,2],color = 'y')

    ax.scatter(op[0,0],op[1,0],op[2,0],color = 'b')
    ax.scatter(ob[0,0],ob[1,0],ob[2,0],color = 'b')
    
    return ax,fig



ob_fixed = np.zeros(shape = [1,3])
ob_fixed[:] = -1
#op_new = np.zeros(shape = [1,3])
#op_new[:] = -1
cnt = 0


ob_fixed = uniarray(ob_even,ob_odd)
op_home = uniarray(op_even,op_odd)

#op_new = np.zeros(shape = [6,3])

def Orient(get_angle,op,v_axis,pt):
    orient_axis = np.zeros(shape = [3,1])
    v_axis = np.matrix(v_axis)
    [itera1,itera2] = np.shape(v_axis)
    if itera1 > itera2: 
        for k in xrange(itera1):
            orient_axis[k,0] = v_axis[k,0]
    else:
        for k in xrange(itera2):
            orient_axis[k,0] = v_axis[0,k]
    # unit vector
    #roll_axis = V_unit(roll_axis)
    
    # Pitch axis
    # A dummy vector perpendicular to the plane of the roll_axis
    
    op_new = np.zeros(shape = [len(pt),3])
    #orient_axis[0,0] = v_axis[0,0]
    #orient_axis[1,0] = v_axis[1,0]
    #orient_axis[2,0] = v_axis[2,0]
    for l in xrange(len(op_home)):
    #v_axis[2,0]  = 400
        #op_odd_new_dummy = np.dot(R_Rod(get_roll,V_unit(roll_axis)),op_odd[:,l])
        #op_odd_new_dummy = np.dot(R_X(get_roll),op_odd[:,l])
        R_rot = R_L(get_angle,op,orient_axis)
        p_dum = np.zeros(shape = [3,1])
        p_dum = np.matrix(pt[l,:])
        p_dum = np.transpose(p_dum)
        lcol = np.matrix([1])
        p_dum = np.append(p_dum,lcol,axis = 0)
        op_new_dummy = np.dot(R_rot,p_dum)
        for j in xrange(3):
            op_new[l,j] = op_new_dummy[j,0]
    return op_new

def Translation(surge,sway,heave,op_new):
    op_new1 = np.zeros(shape = [6,3])
    for i in xrange(len(op_new)):
        dum = Transl(surge,sway,heave,op_new[i,:])
        for j in xrange(3):
            op_new1[i,j] = dum[j,0]
    return op_new1

def New_position(angle_yaw_pitch_roll_x_y_z,ob_fixed):
    
    input_vec = angle_yaw_pitch_roll_x_y_z
    input_mat = np.matrix([[input_vec[0]],[input_vec[1]],[input_vec[2]],[input_vec[3]],[input_vec[4]],[input_vec[5]]])
    
    Rotmat = R_Z(input_mat[0,0])*R_Y(input_mat[1,0])*R_X(input_mat[2,0])
    Trans_mat = np.matrix([[input_mat[3,0]],[input_mat[4,0]],[input_mat[5,0]]])
    
    op_new = np.matrix([-1,-1,-1])
    for k in xrange(len(ob_fixed)):
        ob_pt = np.matrix(ob_fixed[k])
        ob_pt = np.transpose(ob_pt)
        dum  = (Rotmat*ob_pt + Trans_mat)
        dum = np.transpose(dum)
        op_new = np.vstack((op_new,dum))
    
    op_new = op_new[1:,:]
    op_c = np.array(Trans_mat)
    return op_new,op_c
    
    
    
    
def lineplot(plt_arr):
    plt.plot(plt_arr[:,0],plt_arr[:,1],plt_arr[:,2],'.g-')



rollangle = [0,0,0,0,0]
pitchangle = [0,0,0,0,0]
yawangle = [0,0,0,0,0]


            
    


    
# Crank shaft and connecting rod: 


# Using sympy solver - Works best among scipy and other methods
#################### SOLVER CLASS #########################
class Solver(object):
    # Simultaneous equation solver
    
           
    def Simult3(self,op_new,ob_fixed,ob_fixed_axis,ob_fixed_axis_1,op_c,oc_first,oc_first_axis,l_cr,l_rod,alfa):
        try:
        # Try two initial starting guesses

            from sympy.physics.vector import *
            
            from sympy.physics.mechanics import ReferenceFrame, Vector, dot
            N = ReferenceFrame('N')
            oc1x = Symbol('oc1x',real = True)
            oc1y = Symbol('oc1y',real = True)
            oc1z = Symbol('oc1z',real = True)
            auxvar = Symbol('auxvar',real = True)
            jA = Symbol('jA')
            
            jAunit = Symbol('jAunit')
            qvec = Symbol('qvec')
           

            # Maximum spherical joint allowance
            #sphal = 190*math.cos(deg2rad(alfa))
            # Simultaneous functions
            
            f1 = (op_new[0] - oc1x)*(op_new[0] - oc1x) + (op_new[1] - oc1y)*(op_new[1] - oc1y) + (op_new[2] - oc1z)*(op_new[2] - oc1z) - 450*450
            
            f2 = (oc1x - ob_fixed[0])*(oc1x - ob_fixed[0]) + (oc1y - ob_fixed[1])*(oc1y - ob_fixed[1]) + (oc1z - ob_fixed[2])*(oc1z - ob_fixed[2]) - 190*190
            #f3 = (oc1x - ob_fixed[0])*V2_norm[0] + (oc1y - ob_fixed[1])*V2_norm[1] + (oc1z - ob_fixed[2])*V2_norm[2] - 0
            f3 = (oc1x - ob_fixed[0])*(ob_fixed_axis[0]) + (oc1y - ob_fixed[1])*(ob_fixed_axis[1]) + (oc1z - ob_fixed[2])*(ob_fixed_axis[2])
            #f4 < (oc1x - ob_fixed[0])*(oc1x-ob_fixed[0]) + (oc1y - ob_fixed[1])*(oc1y-ob_fixed[1]) + (oc1z - ob_fixed[2])*(oc1z-ob_fixed[2]) - 1.5707
            #f5 > (oc1x - ob_fixed[0])*(oc1x-ob_fixed[0]) + (oc1y - ob_fixed[1])*(oc1y-ob_fixed[1]) + (oc1z - ob_fixed[2])*(oc1z-ob_fixed[2]) - 1.5707
            #f4 = (oc1x - ob_fixed[0])*(ob_fixed_axis_1[0]) + (oc1y - ob_fixed[1])*(ob_fixed_axis_1[1]) + (oc1z - ob_fixed[2])*(ob_fixed_axis_1[2])
            solution = sympy.solvers.solve([f1,f2,f3],[oc1x,oc1y,oc1z],simplify = True)
            print ('All solutions are here')
            print solution
            #solution1 = solution[0]
            solution1 = solution[1]
            fda = np.array([0,0,0])
            fda[0] = solution1[0]
            fda[1] = solution1[1]
            fda[2] = solution1[2]
            
            print np.linalg.norm((fda - ob_fixed))
            
            angle_vec = V_ang_sign((fda - ob_fixed),(oc_first - ob_fixed),oc_first_axis)
            print angle_vec
            time.sleep(2)
            if (-(Pi/2.0) < angle_vec) and (angle_vec < (Pi/2.0)): # ANgle in radians
                    solution1 = solution[1]
            else:
                solution1 = solution[0]

            return solution1
            
            
            #
            
        except ValueError:
            print('Nothing')

    
    def mainsolver(self,ob_axis,ob_fixed,ob_fixed_axis,ob_fixed_axis_1,op_c,op_new,oc_first,oc_first_axis,l_cr,l_rod,alfa):
        
        #V1_norm = V_cross2((ob_axis[:,0] - ob_fixed[0,:] ),(op_axis[:,0] - ob_axis[:,0]))
        #V2_norm = V_cross2((ob_axis[:,1] - ob_fixed[3,:] ),(op_axis[:,1] - ob_axis[:,1]))
        #V3_norm = V_cross2((ob_axis[:,2] - ob_fixed[5,:] ),(op_axis[:,2] - ob_axis[:,2]))
        
        #V_per = np.append(np.matrix(V1_norm),np.matrix(V2_norm),axis  = 0)
        #V_per = np.append(V_per,np.matrix(V3_norm),axis  = 0)
        #V_per = np.asarray(V_per)
        
        # Declaring the crank point matrix - 
        oc = np.zeros(shape = [6,3])
        oc[:,:] = -1
        counter = 0
        for i in xrange(3):
        
            sol1 = []
            sol1 =  self.Simult3(op_new[counter,:],ob_fixed[counter,:],ob_fixed_axis[counter,:],ob_fixed_axis_1[counter,:],op_c,oc_first[counter,:],oc_first_axis[counter,:],l_cr,l_rod,alfa)
            
            if sol1 is not None :
                for j in xrange(3):
                    oc[counter,j] = sol1[j]
            
            counter += 1
            
            sol2 = []    
            sol2  =  self.Simult3(op_new[counter,:],ob_fixed[counter,:],ob_fixed_axis[counter,:],ob_fixed_axis_1[counter,:],op_c,oc_first[counter,:],oc_first_axis[counter,:],l_cr,l_rod,alfa)
            
            if sol2 is not None:
                for j in xrange(3):
                    oc[counter,j] = sol2[j]
            counter += 1
        return oc
    
    def plotarr1(self,ocn,pivot_arr,ax):
        ocn = ocn.flatten()
        pivot_arr = pivot_arr.flatten()
       
        ax.plot(xs = [ocn[0],pivot_arr[0]],ys = [ocn[1],pivot_arr[1]],zs = [ocn[2],pivot_arr[2]])
        
        #print 'i am done'
        return ax
          
    def plotlines(self,ob_fixed,op_new,oc,ax):
        for j in xrange(6):
            self.plotarr1(oc[j,:],ob_fixed[j,:],ax)
            #plt.show()
        
        for j in xrange(6):
            self.plotarr1(oc[j,:],op_new[j,:],ax)
            #plt.show()
        return ax
    
                  
                 
             
                 
             
alfa = 60
# Length of crank
l_cr = 190
# Length of connecting rod
l_rod = 450

# Initiate inverse kinematics solver             
invkin = Solver() # Solver class object



angle_crank = [-100,-100,-100,-100,-100,-100]

ob_fixed_axis = np.array([-1,-1,-1])
for p in xrange(6):
    oc_per_n_scale = oc_per[p,:]
    dum = ob_fixed[p,:] - oc_per_n_scale
    ob_fixed_axis = np.vstack((ob_fixed_axis,dum))

ob_fixed_axis_1 = ob_fixed_axis[1:,:]

ob_fixed_axis = np.array([[573.85,0,0],[573.85,0,0],[-286.925,496.96867796,0],[-286.925,496.96867796,0],[-286.925,-496.96867796,0],[-286.925,-496.96867796,0]])

op_home_plt = plotarray(op_home)
ob_fixed_plt = plotarray(ob_fixed)

### Plotting happens here #################
for i in  xrange(len(point_gen)):
    
    ax,fig = plot_visual(ob_even,ob_odd,op_even,op_odd,oc_first,oc_per,oc_first_axis,op,ob,op_home,ob_fixed)
    
    
    [op_new,op_c] = New_position(point_gen[i],op_home)
    op_new_plt = plotarray(op_new)
    op_new_plt = np.array(op_new_plt)
    plt.plot(op_new_plt[:,0],op_new_plt[:,1],op_new_plt[:,2],'.g-')
    ax.scatter(op_c[0,0],op_c[1,0],op_c[2,0],color = 'cyan')
    #lineplot(op_new_plt)
    
    op_new = np.array(op_new)
    
    # Maximum spherical cone angle in degrees
    

    invkinsol_1 = invkin.mainsolver(ob_axis,ob_fixed,ob_fixed_axis,ob_fixed_axis_1,op_c,op_new,oc_first,oc_first_axis,l_cr,l_rod,alfa)
    
    #invkinsol = invkin.checkside(invkinsol_1,oc_first,oc_first_axis,ob_fixed)
    invkinsol = invkinsol_1
    
    
    
    #invkinsol = invkinsol_1
    # Plotting the legs
    if invkinsol[0,0] != -1:
        dum1 = [-100,-100,-100,-100,-100,-100]
        for r in xrange(6):
            dum1[r] = (V_ang((invkinsol[r,:] - ob_fixed[r,:]),(oc_first[r,:] - ob_fixed[r,:])))*1.0
        
        angle_crank = np.vstack((angle_crank,dum1))
        
        invkin.plotlines(ob_fixed,op_new,invkinsol,ax)
        
        plt.pause(0.01)
        fig.savefig('stewart_'+str(i)+'_'+'.png')
        # Saving as gif 
angle_crank = angle_crank[1:,:]
    



