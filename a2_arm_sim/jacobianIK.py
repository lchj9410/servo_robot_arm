import time
import numpy as np
import serial
from math import *
from my_arm_jacobian3 import jacobian3
from my_arm_FK3 import FK3

ser=serial.Serial('COM3',115200,timeout=1)

def convert_serial_number(num):
	r_num=round(num,2)
	num_int=int(r_num)
	num_dec2=int(100*r_num%100)
	return num_int,num_dec2

def IK_jacobian(v): 
	J=jacobian3(current_relative_angle[0],current_relative_angle[1],current_relative_angle[2])
	J=np.transpose(J)
	pinvJ=np.linalg.pinv(J)
	theta_dot=pinvJ.dot(v)
	return theta_dot
def move_arm(target):
	global current_relative_angle,buf
	end_pos=FK3(current_relative_angle[0],current_relative_angle[1],current_relative_angle[2])
	error=end_pos-target
	error_norm=np.linalg.norm(error)
	if error_norm>0.0001:
		v=-error*kp
		theta_dot=IK_jacobian(v)
		theta_dot=np.clip(theta_dot[0:3],-1.5,1.5)

		current_relative_angle+=theta_dot*dt
		abs_ang=current_relative_angle+[pi/2,    170/180*pi,    80/180*pi   ]

		for i in range(3):
			num_int,num_dec2=convert_serial_number(abs_ang[i]/pi*180)
			buf[2*i+1:2*i+3]=[num_int,num_dec2]

		ser.write(buf)

	return error_norm
def Loop_sleep(sec):
	TT = time.time() # for some reson time.sleep() dose not work for small number?
	TTelapsed=0
	while TTelapsed<sec:
		TTelapsed = time.time() - TT


dt=0.02
angle_k=2
kp=3*np.array([1,1,1])
buf=[255,0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 30,0]
direction=0 
starting_pose=FK3(0,0,0)
test_end_effector_pose=[-0.15,0.2,0.1]
current_relative_angle=[0,0,0]
time.sleep(2)
ser.write([255,90,0, 170,0, 80,0, 90,0, 90,0, 90,0, 30,0])
time.sleep(2)
while True:

	# target_x=input('target x:')
	# target_y=input('target y:')
	# target_z=input('target z:')
	# end_effector_pose=[target_x,target_y,target_z,1,cos(direction),sin(direction)]
	Loop_sleep(5*dt)
	error_norm=move_arm(test_end_effector_pose)
	if error_norm<0.005:
		break


