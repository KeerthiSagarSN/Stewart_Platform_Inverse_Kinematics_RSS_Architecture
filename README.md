# Inverse_Kinematics_StewartPlatform_RSS
 Inverse Kinematics Planner for 6 DOF Stewart Platform with Rotary actuators
 
 A 6 DOF gaming rig required a Stewart plaform architecture to achieve all ranges of motion - Roll, pitch, yaw, heave, surge and sway.
 Generally stewart platform architecture comprises of Linear actuators, in this project rotary actuators (Motors) were preferred with Revolute-Spherical-Spherical (RSS) joints. A prototype inverse kinematic planner based on Python was developed to determine the geometry, architecture and range of motions that can be achieved. A more optimized and fast planner was later developed with a C++ version. This is only for testing purpose but provides the exact result.
 
 Reference paper: Szufnarowski, Filip. "Stewart platform with fixed rotary actuators: a low cost design study." Advances in medical Robotics (2013).
 
 # Prerequisities
 
 Python 2.7, Sympy
 
 # Instructions
 
 1. Main planner file is stewartplatforminverse_8_Baseframe
 2. All geometrical parameters are hardcoded, but most important parameters are commented accordingly
 3. Execute "stewartplatforminverse_8_Baseframe.py" for obtaining visual representation of the link length and angle.
 4. Code runs slow because symbolic computations (Sympy) were used for fast prototyping. 
 
 # Input 
 
 1. Line 771 in "stewartplatforminverse_8_Baseframe.py" consists of the input array in the form of list.
 Ex. point_gen = [[0,0,0,0,0,350],[0,0,0,0,150,350]], where [yaw,pitch,roll,surge,sway,heave] in absolute position to the base frame/Origin.
 
 # Output
 
 1. Heave - 100 mm 
 
 ![alt text](http://url/to/img.png)
 
 2. Sway - 100 mm
 
 ![alt text](http://url/to/img.png)
 
 3. Surge - 100 mm
 
 ![alt text](http://url/to/img.png)
 
 4. Roll - 5 degrees
 
 ![alt text](http://url/to/img.png)
 
 5. Pitch - 5 degrees
 
 ![alt text](http://url/to/img.png)
 
 6. Yaw - 5 degrees
 
 ![alt text](http://url/to/img.png)
 
 
