# -*- coding: utf-8 -*-
"""
Created on Tue Jan 03 11:03:35 2017

@author: Keerthi
"""
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from linearalgebra import V_mod

# Geometrical parameters input
# Spherical joint coordinates in Base frame - Considering position where alpha angle is 90 to the x - axis
Sb = np.array([-100,700,-400])


Ob = np.array([0,0,0])


SO = Sb - Ob

# In base frame
P4A = SO +  [0,-100,0]

P4C = SO +  [0,100,0]

P3B = SO + [100,0,0]

P1A = Ob + [0,-500,0]
P1C = Ob + [0,500,0]
P1B = Ob + [500,0,0]

# In 
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')
X = [P4A[0],P4C[0],P3B[0],P4A[0]]
Y = [P4A[1],P4C[1],P3B[1],P4A[1]]
Z = [P4A[2],P4C[2],P3B[2],P4A[2]]
ax.plot(X,Y,Z)

# Displacement Vector
la = P4A - P1A
lb = P3B - P1B
lc = P4C - P1C

lad = V_mod(la)
lbd = V_mod(lb)
lcd = V_mod(lc)

ax.scatter(Sb[0],Sb[1],Sb[2],'.')
ax.scatter(Ob[0],Ob[1],Ob[2],'.')
ax.scatter(P1B[0],P1B[1],P1B[2],'.')
ax.scatter(P1A[0],P1A[1],P1A[2],'.')
ax.scatter(P1C[0],P1C[1],P1C[2],'.')

ax.plot([P4A[0],P1A[0]],[P4A[1],P1A[1]],[P4A[2],P1A[2]])
ax.plot([P4C[0],P1C[0]],[P4C[1],P1C[1]],[P4C[2],P1C[2]])
ax.plot([P3B[0],P1B[0]],[P3B[1],P1B[1]],[P3B[2],P1B[2]])

