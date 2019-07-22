# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 17:34:17 2016

@author: keerthi
"""

# Rotation Matrix definitions
import numpy as np
import math
from sympy import sqrt



# Rotation around X axis
# Angle in degrees
def R_X(ang):
    return np.matrix([[1, 0, 0], [0, math.cos(math.radians(ang)), -math.sin(math.radians(ang))], [0, math.sin(math.radians(ang)), math.cos(math.radians(ang))] ])
        
    
# Rotation around Y axis
# Angle in degrees
def R_Y(ang):
    return np.matrix([[math.cos(math.radians(ang)), 0, math.sin(math.radians(ang))], [0, 1, 0], [-math.sin(math.radians(ang)), 0, math.cos(math.radians(ang))] ])
 
# Rotation around Y axis
# Angle in degrees
def R_Z(ang):
    return np.matrix([[math.cos(math.radians(ang)), -math.sin(math.radians(ang)), 0], [math.sin(math.radians(ang)), math.cos(math.radians(ang)), 0], [0, 0, 1]])

# General Rotation matrix

def R_r(angx, angy, angz):
    return R_X(angx)*R_Y(angy)*R_Z(angz)

# Module/ Magnitude of a vector
def V_mod(V):
    # Square root 
    return np.linalg.norm(V)

# unit vector of a vector V

def V_unit(V):
    # V_norm = V/V_mod
    V = np.array(V)
    all_zeros = not np.any(V)
    if all_zeros == True:
        return V
    else:
         return V*(np.power(V_mod(V),-1))
        

# Unit vector of a Vector S

def S_unit(S):
    # S_norm = S/S_mod
    S = np.array(S)
    all_zeros = not np.any(V)
    if all_zeros == len(S):
        return S
    else:
        S = np.matrix(S)
        
        return S*(np.power(S_mod(S),-1))    
    
 # Angle between two vectors
def V_ang(V1,V2):
    # V1.V2 = |V1||V2|cos(tet)
    # Angle in radians
       
    return np.arccos(np.clip(np.dot(V_unit(V1), V_unit(V2)), -1.0, 1.0))
    



 # Angle between two vectors - 
def V_ang_sign(V1,V2,V_ref):
    # Ref: https://www.opengl.org/discussion_boards/showthread.php/159385-Deriving-angles-from-0-to-360-from-Dot-Product
    #from scipy.linalg import norm
    # V1.V2 = |V1||V2|cos(tet)
    # Angle in radians
    #dot_V = np.dot(V1,V2)
    #aa = np.cross(V1,V2)
    #det_V = np.dot(aa,aa)
    #dot_V  = np.dot(V1,V2)
    cross_V = np.cross(V1,V2)
    angle_V = np.arccos(np.clip(np.dot(V_unit(V1), V_unit(V2)), -1.0, 1.0))
    #angle_V = np.arctan2(np.linalg.norm(cross_V),np.dot(V_unit(V1), V_unit(V2)))
    direction_V = np.dot(V_unit(V_ref),V_unit(cross_V))
    
    if direction_V < 0:
        angle_V = -angle_V
    
     
    
    return angle_V
    
    #return np.arctan2(det_V,dot_V) 
    
# Skew Matrix
'''
def V_skew(V):
    
    return
'''

def py_ang(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'    """
    cosang = np.dot(v1, v2)
    sinang = np.linalg.norm(np.cross(v1, v2))
    return np.arctan2(sinang, cosang)
    
# Sign of the vector basis
# m,n indicates the X and y plane
def V_sign(V1,m,n):
    if (len(V1) > 1) and (len(V2) > 1):
        V1_msign = np.sign(V1[m])
        V1_nsign = np.sign(V1[n])
        return V1_msign, V1_nsign
        #return V1_ysign = np.sign(V1[1])
    
        

# Angle of 2 Vector when no direction is  given between two vectors - XY Plane - Counterclockwise

def V_angxy(V1,V2):
    if (len(V1) > 1) and (len(V2) > 1):
       m = 0
       n = 1
       V1_sign = V_sign(V1,m,n)
       V2_sign = V_sign(V2,m,n)
       sign_xy = np.concatenate((V1_sign,V2_sign),axis = 0)
       V1_xy = (V1[m],V1[n])
       V2_xy = (V2[m],V1[n])
       # X coordinate  
       ang_xy = V_ang(V1_xy,V2_xy)
       return ang_xy,sign_xy
    else:
        print ('Vector size is not 2 Dimensional, Enter a 2D size vector')
        
# Angle of 2 Vector when no direction is  given between two vectors - YZ Plane - Counterclockwise
def V_angyz(V1,V2):
    if (len(V1) > 2) and (len(V2) > 2):
       m = 1
       n = 2
       V1_sign = V_sign(V1,m,n)
       V2_sign = V_sign(V2,m,n)
       sign_yz = np.concatenate((V1_sign,V2_sign),axis = 0)
       V1_yz = (V1[m],V1[n])
       V2_yz = (V2[m],V1[n])
       # X coordinate  
       ang_yz = V_ang(V1_yz,V2_yz)
       return ang_yz,sign_yz
    else:
        print ('Vector size is not 3 Dimensional, Enter a 3D size vector')       

# Angle of 2 Vector when no direction is  given between two vectors - XZ Plane - Counterclockwise
def V_angxz(V1,V2):
    if (len(V1) > 2) and (len(V2) > 2):
       m = 0
       n = 2
       V1_sign = V_sign(V1,m,n)
       V2_sign = V_sign(V2,m,n)
       sign_xz = np.concatenate((V1_sign,V2_sign),axis = 0)
       V1_xz = (V1[m],V1[n])
       V2_xz = (V2[m],V1[n])
       # X coordinate  
       ang_xz = V_ang(V1_xz,V2_xz)
       return ang_xz,sign_xz
    else:
        print ('Vector size is not 3 Dimensional, Enter a 3D size vector')             
    
     #X Coordinate
# Normal vector when one vector is given
# Vector 1 needs to be an array of 3d
def V_cross1(V1):
    V2 = np.zeros(3)
    if V1[0] == 0 and V1[1] == 0:
        V2[0] = 100
    if V1[0] == 0 and V1[2] == 0:
        V2[0] = 100
    if V1[1] == 0 and V1[2] == 0:
        V2[1] = 100
    else:
        V2[0] = V1[0] + 300
        V2[1] = V1[1] + 300
    return np.cross(V1,V2)
    
def V_cross2(V1,V2):
    
    return np.cross(V1,V2)    

        
        
      
# Homogeneous matrix
def H_mat(T):
    H_mat = np.zeros(shape = [4,4])
    H_mat[0,0] = 1
    H_mat[0,1] = 0
    H_mat[0,2] = 0
    H_mat[0,3] = T[0,0]
    H_mat[1,0] = 0
    H_mat[1,1] = 1
    H_mat[1,2] = 0
    H_mat[1,3] = T[1,0]
    H_mat[2,0] = 0
    H_mat[2,1] = 0
    H_mat[2,2] = 1
    H_mat[2,3] = T[2,0]

# Rodriguez rotation matrix - Only when axis is not parallel to basis vector axis - x,y,z -Axis is parallel and offset to basis axis
def R_Rod(teta,v_axis):
    # I - Identity matrix 3x3
    # v_axis should be a matrix of dimension 3x1
    I = np.identity(3)

    
    # The cross product matrix - Check wikipedia - Rodriguez rotation formula for reference
    K = np.zeros(shape = [3,3])
    K[0,0] = 0
    K[1,1] = 0
    K[2,2] = 0
    K[0,1] = -1*v_axis[2,0]
    K[0,2] = v_axis[1,0]
    K[1,0] = v_axis[2,0]
    K[1,2] = -1*v_axis[0,0]
    K[2,0] = -1*v_axis[1,0]
    K[2,1] = v_axis[0,0]
    R_Rod = I + math.sin(math.radians(teta))*K + (1- math.cos(math.radians(teta)))*np.dot(K,K)
    return R_Rod

# Translating a point to the origin
def T_orig(p):
    D = np.identity(4)
    D[0,3] = -1*p[0,0]
    D[1,3] = -1*p[1,0]
    D[2,3] = -1*p[2,0] 
    return D
def Transl(i,j,k,pt):
    pt = np.matrix(pt)
    pt = np.transpose(pt)
    D = np.identity(4)
    D[0,3] = i
    D[1,3] = j
    D[2,3] = k
    # pt is input as  3x1 matrix need to convert it into 4x1 matrix to perform multiplication with 4x4 and 4x1
    dd = np.zeros(shape=[1,1])
    dd[0,0] = 1
    pt = np.append(pt,dd,axis = 0)
    Td = np.dot(D,pt)
    return Td
# Rotation matrix about an arbitrary line - Multiply this matrix with the point to get Rotated point
# Teta - angle, l_p1 - First point on the line-axis to rotate, l_p2 - Second point on the line axis to rotate
def R_L(teta,l_p1,l_p2):
    v_axis = l_p2 - l_p1
    # Translate the point to the origin
    D = T_orig(l_p1)
    # Rodriguez rotation matrix
    R_rod = R_Rod(teta,V_unit(v_axis))
    # Rodriguez Rotation matrix is 3x3, but the entire Rotation matrix is 4x4
    endrow = np.zeros(shape = [1,3])
    R_rod = np.append(R_rod,endrow,axis = 0)
    endcolumn = np.zeros(shape = [4,1])
    endcolumn[3,0] = 1
    R_rod = np.append(R_rod,endcolumn,axis = 1)
    # Final rotation matrix
    R_rot = np.dot(np.linalg.inv(D),np.dot(R_rod,D))
    return R_rot
    # Step 1 : Translate point to origin:
        
        
       
    
        
        
        

# Round matrix with given direction

#def M_round(M):
#    
#    return


    
    
    

    
    

  

