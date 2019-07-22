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
 
 
