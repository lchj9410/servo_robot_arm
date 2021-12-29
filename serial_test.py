import serial
import time
import numpy as np

ser=serial.Serial('COM3',115200,timeout=1)

def convert_serial_number(num):
	r_num=round(num,2)
	num_int=int(r_num)
	num_dec2=int(100*r_num%100)
	return num_int,num_dec2

def test_port():
	global buf
	user_in=float(input("Enter value: "))
	port=int(input("Enter port: "))
	num_int,num_dec2=convert_serial_number(user_in)
	buf[2*port+1:2*port+3]=[num_int,num_dec2]
	ser.write(buf)

def test_vel(hz):
	global buf
	user_in=float(input("Enter value: "))
	port=int(input("Enter port: "))
	abs_vel=float(input("Enter abs_vel: "))
	current_angle=buf[2*port+1]+buf[2*port+2]/100
	delta_angle=user_in-current_angle+0.001
	vel=np.sign(delta_angle)*abs_vel
	dq=vel/hz
	num_of_steps= int(abs(delta_angle/dq))
	# print(f"num_of_steps= {num_of_steps}")
	for i in range(num_of_steps):
		current_angle+=dq
		num_int,num_dec2=convert_serial_number(current_angle)
		buf[2*port+1:2*port+3]=[num_int,num_dec2]
		ser.write(buf)
		TT = time.time()
		TTelapsed=0
		while TTelapsed<1/hz:
			TTelapsed = time.time() - TT



time.sleep(2)
# a=ser.write([255,0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 30,0])
buf=[255,0,0, 0,0, 0,0, 0,0, 0,0, 0,0, 0,0]
while True:
	# test_port()
	# time.sleep(1)
	test_vel(50)
	




