# -*- coding: utf-8 -*-
"""
Created on Mon Dec 26 12:33:47 2016

@author: Keerthi
"""


from sympy import init_printing
init_printing(use_latex='mathjax',pretty_print = False)
from sympy.solvers import solve


from matplotlib import pyplot as plt

# Scalar variables imported from Sympy.abc library
from sympy.abc import a,b,c,d,e,f,g,h,l, theta
from sympy import sin,cos,tan,pi,acos,asin
from sympy import symbols

# For profiling

import cProfile

# Creating Reference frame 
from sympy.physics.vector import ReferenceFrame, dot, cross


# Visualization library

from pydy.viz.shapes import Cylinder, Sphere

from pydy.viz.visualization_frame import VisualizationFrame
from pydy.viz.scene import Scene

N = ReferenceFrame('N')
A = N.orientnew('A','Axis',(theta,N.z))
#a = c*N.x + c*N.y + d*N.z
#
## Converting to matrix form from vector form
#
#b = a.to_matrix(N)
#
## Magnitude of a vector
#
#bb = a.magnitude()
#
#gt = a + 2*a



v1 = l*cos(pi/4)*N.x + l*sin(pi/4)*N.y + 0*N.z
v3 = l*cos(theta)*N.x + l*sin(theta)*N.y + 0*N.z

# To display real value magnitude


l = symbols('l',real = True)

v1 = l*cos(pi/4)*N.x + l*sin(pi/4)*N.y + 0*N.z
bvt = v1.magnitude()

# Dot product

v = dot(v1,v3)

# FInding angle between two vectors using dot product
v3 = a*N.x + b*N.y + a*N.z
v4 = b*N.x + a*N.y + b*N.z

pt = acos(dot(v3,v4)/(v3.magnitude()*(v4.magnitude())))


 # FInding angle between two vectors using cross product
v5 = a*N.x + b*N.y + a*N.z
v6 = b*A.x + a*A.y + b*A.z

g = cross(v5,v6)
# Method way of expressing cross product
h = v5.cross(v6)



# Display cross product w.r.t to one amtrix

rt = (v5 + v6).express(A)

