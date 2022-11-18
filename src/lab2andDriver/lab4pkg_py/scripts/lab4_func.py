#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab4_header import *
import math
# PI = np.pi

L1 = 0.152
L2 = 0.120
L3 = 0.244
L4 = 0.093
L5 = 0.213
L6 = 0.083
L7 = 0.083
L8 = 0.082
L9 = 0.0535
L10 = 0.059
L = np.array([0, L1, L2, L3 , L4, L5, L6, L7, L8, L9, L10])

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

'''
We get inspirations of Tower of Hanoi algorithm from the website below.
This is also on the lab manual.
Source: https://www.cut-the-knot.org/recurrence/hanoi.shtml

To run this fiel:
source devel/setup.bash
roslaunch ur3_driver ur3_driver.launch
rosrun lab2pkg_py lab2_exec.py
'''

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([[0., -1., 0., 0.390],
				  [0., 0., -1., 0.401],
				  [1., 0., 0.,  0.2155],
				  [0., 0., 0.,  1.]])	

	S = np.array([[0., 0., 1.,  0.150, 0.150,  0.000],
				  [0., 1., 0., -0.162, 0.000, -0.150],
				  [0., 1., 0., -0.162, 0.000,  0.094],
				  [0., 1., 0., -0.162, 0.000,  0.307],
				  [1., 0., 0.,  0.000, 0.162, -0.260],
				  [0., 1., 0., -0.162, 0.000,  0.390]])

	# ==============================================================#
	return M, S
# Get_MS()

"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M, S = Get_MS()
	
	S_bucket = np.zeros((6,4,4))
	# Theta = [theta1, theta2, theta3, theta4, theta5, theta6]

	for i in range(6):
		S_bucket[i][0][1] = -S[i][2] #w3
		S_bucket[i][1][0] = S[i][2]

		S_bucket[i][0][2] = S[i][1]  #w2
		S_bucket[i][2][0] = -S[i][1]

		S_bucket[i][1][2] = -S[i][0]  #w1
		S_bucket[i][2][1] = S[i][0]

		S_bucket[i][0][3] = S[i][3]
		S_bucket[i][1][3] = S[i][4]
		S_bucket[i][2][3] = S[i][5]

	T1 = np.dot(expm(S_bucket[6-1]*theta6),M)
	T2 = np.dot(expm(S_bucket[5-1]*theta5),T1)
	T3 = np.dot(expm(S_bucket[4-1]*theta4),T2)
	T4 = np.dot(expm(S_bucket[3-1]*theta3),T3)
	T5 = np.dot(expm(S_bucket[2-1]*theta2),T4)
	T  = np.dot(expm(S_bucket[1-1]*theta1),T5)
	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	global L
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = -np.pi/2
	theta6 = 0.0
	yaw_radians = np.deg2rad(yaw_WgripDegree)

	# =================== Your code starts here ====================#
	# 1 calculate gripper's coordinates in base frame
	y_offset = 0.15
	x_offset = -0.15
	z_offset = 0.01
	x_grip = xWgrip - x_offset
	y_grip = yWgrip - y_offset
	z_grip = zWgrip - z_offset

	# 2 calculate cen coordinates
	x_cen = x_grip-L[9]*math.cos(yaw_radians)
	y_cen = y_grip-L[9]*math.sin(yaw_radians)
	z_cen = z_grip
	print("xyz_cen = " + str([x_cen, y_cen, z_cen]))
	# 3 calculate theta_1
	theta1 = math.atan2(y_cen, x_cen) - math.asin((L[2]-L[4]+L[6])/((x_cen**2+y_cen**2)**0.5))

	# 4 calculate theta_6
	theta6 = np.pi/2 - yaw_radians + theta1

	# 5 calculate the coordinates of 3end point
	x_3end = x_cen - L[7]*math.cos(theta1) + (L[6]+0.027)*math.sin(theta1)
	y_3end = y_cen - L[7]*math.sin(theta1) - (L[6]+0.027)*math.cos(theta1)
	z_3end = z_cen + L[10] + L[8]
	print("xyz_3end = " + str([x_3end, y_3end, z_3end]))
	# print("Z_3end = " + str(z_3end))

	# 6 calculate theta2,3,4
	d = (x_3end**2+y_3end**2)**0.5
	h = z_3end-L[1]
	theta3 = np.pi - math.acos((L[3]**2+L[5]**2-h**2-d**2) / (2*L[3]*L[5]))

	alpha = math.acos((d**2 + h**2 + L[1]**2 - (z_3end**2+d**2))/(2*(d**2+h**2)**0.5*L[1]))
	beta  = math.acos((d**2 + h**2 + L[3]**2 - L[5]**2)/(2*(d**2+h**2)**0.5*L[3]))
	print("alpha = " + str(np.rad2deg(alpha)))
	print("beta = " + str(np.rad2deg(beta)))
 
	theta2 = np.pi/2 - alpha - beta
	theta4 = -theta2-theta3

	# convert angles from radians to degree
	theta1_d = np.rad2deg(theta1)
	theta2_d = np.rad2deg(theta2)
	theta3_d = np.rad2deg(theta3)
	theta4_d = np.rad2deg(theta4)
	theta5_d = np.rad2deg(theta5)
	theta6_d = np.rad2deg(theta6)

	theta_d = [theta1_d, theta2_d, theta3_d, theta4_d, theta5_d, theta6_d]
	print("Theta_degree = " + str(theta_d))
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)


# lab_invk(0.1, 0.25, 0.15, 90)
