import time
import numpy as np
import serial
from pyquaternion import Quaternion
from math import *
from my_arm_jacobian6 import jacobian6
from my_arm_FK import FK
from my_arm_FK6 import FK6
# from my_arm_jacobian import jacobian
# from my_arm_jacobian5 import jacobian5
ser=serial.Serial('COM3',115200,timeout=1)

def convert_serial_number(num):
	r_num=round(num,2)
	num_int=int(r_num)
	num_dec2=int(100*r_num%100)
	return num_int,num_dec2

def IK_jacobian(v): 
	J=jacobian6(current_relative_angle[0],current_relative_angle[1],current_relative_angle[2],\
		current_relative_angle[3],current_relative_angle[4],current_relative_angle[5])
	J=np.transpose(J)
	pinvJ=np.linalg.pinv(J)
	theta_dot=pinvJ.dot(v)
	return theta_dot
def move_arm(target):
	global current_relative_angle,buf
	end_pos=FK6(current_relative_angle[0],current_relative_angle[1],current_relative_angle[2],\
		current_relative_angle[3],current_relative_angle[4],current_relative_angle[5])
	error=end_pos-target
	error_norm=np.linalg.norm(error)
	if error_norm>0.001:
		v=-error*kp
		theta_dot=IK_jacobian(v)
		theta_dot[0:3]=np.clip(theta_dot[0:3],-1.0,1.0)
		theta_dot[3:]=np.clip(theta_dot[3:],-2.0,2.0)


		current_relative_angle+=theta_dot*dt
		abs_ang=current_relative_angle+[pi/2,170/180*pi,80/180*pi,pi/2,pi/2,pi/2]

		for i in range(6):
			num_int,num_dec2=convert_serial_number(abs_ang[i]/pi*180)
			buf[2*i+1:2*i+3]=[num_int,num_dec2]

		ser.write(buf)

	return error_norm
def Loop_sleep(sec):
	TT = time.time() # for some reson time.sleep() dose not work for small number?
	TTelapsed=0
	while TTelapsed<sec:
		TTelapsed = time.time() - TT


dt=0.05
angle_k=2
kp=3*np.array([1,1,1,angle_k,angle_k,angle_k])
buf=[255,0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 30,0]
direction=0 
starting_pose=FK6(0,0,0,0,0,0)
test_end_effector_pose=starting_pose+[-0.15,-0.05,-0.08,0,0,0]
current_relative_angle=[0,0,0,0,0,0]
time.sleep(2)
ser.write([255,90,0, 170,0, 80,0, 90,0, 90,0, 90,0, 00,0])
time.sleep(2)
# while True:

# 	# target_x=input('target x:')
# 	# target_y=input('target y:')
# 	# target_z=input('target z:')
# 	# end_effector_pose=[target_x,target_y,target_z,1,cos(direction),sin(direction)]
# 	Loop_sleep(dt)
# 	error_norm=move_arm(test_end_effector_pose)
# 	if error_norm<0.005:
# 		break


