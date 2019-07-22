# -*- coding: utf-8 -*-
"""
Created on Wed Nov 30 17:34:17 2016

@author: keerthi
"""

# Rotation Matrix definitions
import matplotlib
matplotlib.use('Qt4Agg')
import numpy as np
import math
from linearalgebra import V_unit
from mpl_toolkits.mplot3d import Axes3D
from scipy.linalg import norm
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import matplotlib.pyplot as plt


matplotlib.use('TkAgg') 

# Joint visualisations

#r_cylinder = 5
#extr = 50



# Creating a cylinder surface for revoulute joint
# Reference Link - Cylinder about arbitrary axis : http://mathforum.org/library/drmath/view/51734.html
#http://stackoverflow.com/questions/32317247/how-to-draw-a-cylinder-using-matplotlib-along-length-of-point-x1-y1-and-x2-y2
# Input for cylinder is the 3D vector of the axis and the radius of the cylinder
def revolute(p0,p1,r_revolute,extr):
    R = r_revolute
    p1 = np.array(p1)
    p0 = np.array(p0)
    v_axis = p1 - p0
    
    v_unit = V_unit(v_axis)
    # Create a random unit vector not in the same direction as v_unit
    not_v_unit = np.array([1,0,0])
    # Checking if random unit vector lies along the unit vector
    if (not_v_unit == v_unit).all():
        not_v_unit = np.array([0,1,0])
    # Vector perpendicular to v_unit
    n1 = np.cross(v_unit,not_v_unit)
    # Normalizing the perpendicular vector
    n1 /= norm(n1)
    # Common unit vector perpendicular to v_unit and n1
    n2 = np.cross(v_unit,n1)
    # Creating surface with this unit vector just as a circle equation helps
    # mag can take value of length of the vector but in our case a fixed extrusion for the revolute joint is followed
    mag = extr
    t = np.linspace(0, mag, 100)
    theta = np.linspace(0, 2 * np.pi, 100)
    #use meshgrid to make 2d arrays
    t, theta = np.meshgrid(t, theta)
    #generate coordinates for surface
    x_cyl, y_cyl, z_cyl = [p0[i] + R * np.sin(theta) * n1[i] + R * np.cos(theta) * n2[i] + v_unit[i] * t for i in [0, 1, 2]]
    # Start cap for the cylinder
    R_start = np.linspace(0,R,2)
    R_start, theta = np.meshgrid(R_start, theta)
    
    x_start, y_start, z_start = [p0[i] + R_start * np.sin(theta) * n1[i] + R_start * np.cos(theta) * n2[i] + v_unit[i]*0 for i in [0, 1, 2]]
                                 
    # End cap for the cylinder
    R_end = np.linspace(0,R,2)
    R_end, theta = np.meshgrid(R_end, theta)
    x_end,y_end,z_end = [p0[i] + R_end * np.sin(theta) * n1[i] + R_end * np.cos(theta) * n2[i] + v_unit[i]*mag for i in [0, 1, 2]]
    
    # Calculating the perpendicular bisector of the axis.
    
    #cyl_per = np.cross()
    return x_cyl, y_cyl, z_cyl,x_start, y_start, z_start,x_end, y_end, z_end,v_unit
    

# Create a connection link
def link(p0,v_axis,l,w,extr):
    # Create a cuboid structure
    
    v_axis = np.array(v_axis)
    v_unit = V_unit(v_axis)
    axis_sign = np.sign(v_unit[2])
    # Hypotenuse starting point of the rectangle
    
    # Starting from left corner
    x_link = [p0[0] + w/2,p0[0] - w/2,p0[0] - w/2,p0[0] + w/2,p0[0] + w/2,p0[0] + w/2,p0[0] + w/2,p0[0] + w/2,p0[0] + w/2,p0[0] - w/2,p0[0] - w/2,p0[0] + w/2,p0[0] - w/2,p0[0] - w/2,p0[0] - w/2,p0[0] - w/2,p0[0] - w/2,p0[0] + w/2,p0[0] + w/2,p0[0] - w/2,p0[0] - w/2,p0[0] - w/2,p0[0] + w/2,p0[0] + w/2,p0[0] - w/2] 
    y_link = [p0[1] + l/2,p0[1] + l/2,p0[1] - l/2,p0[1] - l/2,p0[1] + l/2,p0[1] + l/2,p0[1] - l/2,p0[1] - l/2,p0[1] + l/2,p0[1] + l/2,p0[1] + l/2,p0[1] + l/2,p0[1] + l/2,p0[1] - l/2,p0[1] - l/2,p0[1] + l/2,p0[1] - l/2,p0[1] - l/2,p0[1] - l/2,p0[1] - l/2,p0[1] - l/2,p0[1] + l/2,p0[1] + l/2,p0[1] - l/2,p0[1] - l/2]
    z_link = [p0[2],p0[2],p0[2],p0[2],p0[2],p0[2] + axis_sign*(extr),p0[2] + axis_sign*(extr),p0[2],p0[2],p0[2],p0[2] + axis_sign*(extr),p0[2] + axis_sign*(extr),p0[2],p0[2],p0[2] + axis_sign*(extr),p0[2] + axis_sign*(extr),p0[2],p0[2] + axis_sign*(extr),p0[2] + axis_sign*(extr),p0[2],p0[2] + axis_sign*(extr),p0[2] + axis_sign*(extr),p0[2] + axis_sign*(extr),p0[2] + axis_sign*(extr),p0[2] + axis_sign*(extr)]

    return x_link,y_link,z_link


    
# Plotting the rectangle

#ax = Axes3D(fig)
#l = 5
#w = 5
#v_axis = [1,1,1]
#x_linkf1,y_linkf1,z_linkf1,x_linkf2,y_linkf2,z_linkf2,x_linkf3,y_linkf3,z_linkf3,x_linkf4,y_linkf4,z_linkf4,x_linkf5,y_linkf5,z_linkf5,x_linkf6,y_linkf6,z_linkf6,x_link,y_link,z_link = link(p0,v_axis,l,w,extr)
#
#verts = []
#for i in xrange(0,21,4):
#    
#    verts += [zip(x_link[i:i+4],y_link[i:i+4],z_link[i:i+4])]
#    
#    
#
#ax.add_collection3d(Poly3DCollection(verts))
#plt.show()

    
    
    
    
    
    




    
    

  

