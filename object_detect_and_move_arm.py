import cv2
import numpy as np
from math import *
import time
import serial
from s1 import q1
from s2 import q2
from my_arm_FK3 import FK3

ser=serial.Serial('COM3',115200,timeout=1)
def Loop_sleep(sec):
	TT = time.time() # for some reson time.sleep() dose not work correctly???
	TTelapsed=0
	while TTelapsed<sec:
		TTelapsed = time.time() - TT
def convert_serial_number(num):
	r_num=round(num,2)
	num_int=int(r_num)
	num_dec2=int(100*r_num%100)
	return num_int,num_dec2
Q1,Q2=[0,0]
def analyticIK(position): 
	global Q1,Q2
	x=position[0]
	y=position[1]
	z=position[2]
	origin_offset_angle=pi/2-atan2(1,2)
	try:
		angle_offset=asin(sin(pi-origin_offset_angle)/sqrt(x**2+y**2)*sqrt(0.01**2+0.02**2))
	except:
		angle_offset=0
	true_angle=atan2(y,x)+angle_offset
	arm_extend_length=(4*x**2 + 4*y**2 + 3/1250)**(1/2)/2 + 1/100

	try:
		Q1=q1(arm_extend_length,0.19,0.2,z-0.05)
		Q2=q2(arm_extend_length,0.19,0.2,z-0.05)
		if Q1[0]<-pi/2:
			index=1
		else:
			if Q1[1]<-pi/2:
				index=0
			else:
				index=2
		if index!=2: # check reachable sort of...
			target_angle= [true_angle+12.5/180*pi,Q1[index]+177.5/180*pi,Q2[index]+75.5/180*pi, -Q1[index]-Q2[index]+85/180*pi]
			for i in range(4):
				num_int,num_dec2=convert_serial_number(target_angle[i]/pi*180)
				buf[2*i+1:2*i+3]=[num_int,num_dec2]
			ser.write(buf)
	except:
		print('out of reach')

dt=0.05
move_duration=1.0
buf=[255,100,0, 172,50, 77,50, 85,0, 90,0, 50,0]
time.sleep(2)
ser.write([255,100,0, 172,50, 77,50, 85,0, 90,0, 50,0])
time.sleep(2)
starting_pose=FK3(0,0,0).astype('float64')
current_pose=FK3(0,0,0).astype('float64')
def move_arm_increment(target_pose,vel,approaching):
	global current_pose
	gripper_length_center=0.07
	gripper_length_full=0.15
	x=target_pose[0]
	y=target_pose[1]
	z=target_pose[2]
	origin_offset_angle=pi/2-atan2(1,2)
	angle_offset=asin(sin(pi-origin_offset_angle)/sqrt(x**2+y**2)*sqrt(0.01**2+0.02**2))
	true_angle=atan2(y,x)+angle_offset
	if not approaching:
		target_pose[0]-=gripper_length_center*cos(true_angle)
		target_pose[1]-=gripper_length_center*sin(true_angle)
	else:
		target_pose[0]-=gripper_length_full*cos(true_angle)
		target_pose[1]-=gripper_length_full*sin(true_angle)
		target_pose[2]+=0.05
	Delta=abs(target_pose-current_pose)
	duration=sqrt(Delta[0]**2+Delta[1]**2+Delta[2]**2)/vel
	steps=duration/dt
	delta=(target_pose-current_pose)/(steps+1)
	current_pose+=delta
	analyticIK(current_pose)
	return sum(Delta)

def rot(axis,angle): #rotation matrix 
	if axis=='x':
		R=np.array([[1,0,0],[0, cos(angle),- sin(angle)],[0, sin(angle), cos(angle)]])
	if axis=='y':
		R=np.array([[ cos(angle),0, sin(angle)],[0,1,0],[- sin(angle),0, cos(angle)]])
	if axis=='z':
		R=np.array([[ cos(angle),- sin(angle),0],[ sin(angle), cos(angle),0],[0,0,1]])
	return R
camera_rotation=rot(   'z',  30*pi/180   ).dot(  rot(  'x',(36-90)*pi/180   )  )  # 40~50?
def object_vector(coordinates):
	cx=coordinates[0]
	cy=coordinates[1]
	B=(cx-width/2)/(width/2)*0.5458+0.00001
	A=-(cy-height/2)/(height/2)*0.4357
	vec=np.array([1,1/tan(B),tan(A)/tan(B)])
	return vec
def gripper_close(close_or_open):
	global buf
	buf[11]=0 if close_or_open else 50
	ser.write(buf)

def reset_all():
	global approaching_error,gripping_error,lift_up_error,put_down_error,return_to_starting_pose_error,delay_before_gripping_counter,\
	before_put_down_delay_counter,gripper_open_delay_counter,gripping_delay_counter,obj_x,retract_error,place_from_above_error
	approaching_error=99.99
	gripping_error=99.99
	lift_up_error=99.9
	put_down_error=99.99
	retract_error=99.99
	place_from_above_error=99.99
	return_to_starting_pose_error=99.99
	delay_before_gripping_counter=0
	before_put_down_delay_counter=0
	gripper_open_delay_counter=0
	gripping_delay_counter=0
	obj_x=0

obj_coordinate=np.array([])
threshold=0.5
net=cv2.dnn.readNet('C:/cam_vison/yolov4-tiny.weights','C:/cam_vison/yolov4-tiny.cfg')
# net=cv2.dnn.readNet('C:/cam_vison/yolov4-obj_best.weights','C:/cam_vison/yolov4-obj.cfg')
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
classes=[]
with open('C:/cam_vison/coco.names','r') as f:
	classes = f.read().splitlines()

cam_url = "http://asd:11355@192.168.0.12:8080/video"
cap = cv2.VideoCapture(cam_url)
T_now=time.time()
filter_e=0.3
obj_x=0
approaching_error=99.99
gripping_error=99.99
lift_up_error=99.9
put_down_error=99.99
retract_error=99.99
place_from_above_error=99.99
return_to_starting_pose_error=99.99
stage_delay=0.2
delay_before_gripping_counter=0
before_put_down_delay_counter=0
gripper_open_delay_counter=0
gripping_delay_counter=0

TARGET='cup'
while True:
	_,img=cap.read()
	# img= cv2.imread('C:/cam_vison/test_img.jpg')
	height,width,_=img.shape
	blob= cv2.dnn.blobFromImage(img,1/255,(416,416),(0,0,0),swapRB=True,crop=False)
	net.setInput(blob)
	out_put_layers_names = net.getUnconnectedOutLayersNames()
	layerOutputs = net.forward(out_put_layers_names)
	boxes=[]
	confidences=[]
	class_ids=[]
	for output in layerOutputs:
		for detection in output:
			scores = detection[5:]
			class_id=np.argmax(scores)
			confidence=scores[class_id]
			if confidence>threshold:
				x=int(detection[0]*width)
				y=int(detection[1]*height)
				w=int(detection[2]*width)
				h=int(detection[3]*height)
				x_l=int(x-w/2)
				y_l=int(y-h/2)
				boxes.append([x_l,y_l,w,h])
				confidences.append(float(confidence))
				class_ids.append(class_id)
				if str(classes[class_id])==TARGET:
					estimate_coordinate=np.array([x-w/7.5,y+h/(2.5+ (y-height/2)/(height/2)   )])
					obj_vec=object_vector(estimate_coordinate)
					obj_vec=camera_rotation.dot(obj_vec)
					obj_x=obj_vec[0]*(-0.46/obj_vec[2])   #-0.46 : table height relative to camera
					obj_y=obj_vec[1]*(-0.46/obj_vec[2])
				
	indexes=cv2.dnn.NMSBoxes(boxes,confidences,threshold,0.4)
	font = cv2.FONT_HERSHEY_PLAIN
	colors=np.random.uniform(0,255,size=[len(boxes),3] )
	if len(indexes)>0:
		for i in indexes.flatten():
			x,y,w,h =boxes[i]
			label=str(classes[class_ids[i]])
			confidence=str(round(confidences[i],2))
			color=colors[i]
			cv2.rectangle(img,(x,y),(x+w,y+h),color,2)
			cv2.putText(img,label + " " + confidence, (x,y+20),font,2,[255.0,255.0,255.0],2)
	if obj_x!=0:
		if time.time() - T_now>dt:
			if len(obj_coordinate)==0:
				obj_coordinate=np.array([obj_x,obj_y])
			else:
				obj_coordinate=np.array([obj_x,obj_y])*filter_e+obj_coordinate*(1-filter_e)
			if approaching_error>0.02:
				approaching_error=move_arm_increment(np.array([obj_coordinate[0]+0.31,obj_coordinate[1]+0.014,0.13]),0.2,approaching=True)
			else:
				if gripping_error>0.02:
					gripping_error=move_arm_increment(np.array([obj_coordinate[0]+0.31,obj_coordinate[1]+0.014,0.13]),0.2,approaching=False)
				else:
					if delay_before_gripping_counter<stage_delay/dt:
						delay_before_gripping_counter+=1
					else:
						if gripping_delay_counter<stage_delay/dt:
							gripper_close(close_or_open=True)
							gripping_delay_counter+=1
						else:
							if lift_up_error>0.02:
								lift_up_error=move_arm_increment(starting_pose+[0,0.05,0.0],0.3,approaching=False)
							else:
								if place_from_above_error>0.02:
									place_from_above_error=move_arm_increment(np.array([-0.2,0.1,0.18]),0.3,approaching=False)
								else:
									if put_down_error>0.02:
										put_down_error=move_arm_increment(np.array([-0.2,0.1,0.12]),0.3,approaching=False)
									else:
										if before_put_down_delay_counter<stage_delay/dt:
											before_put_down_delay_counter+=1
										else:
											if gripper_open_delay_counter<stage_delay/dt:
												gripper_close(close_or_open=False)
												gripper_open_delay_counter+=1
											else:
												if retract_error>0.02:
													retract_error=move_arm_increment(np.array([-0.1,0.1,0.12]),0.3,approaching=False)
												else:
													if return_to_starting_pose_error>0.02:
														return_to_starting_pose_error=move_arm_increment(starting_pose+[0,0.05,0.0],0.3,approaching=False)
													else:
														reset_all()

			T_now=time.time()
	cv2.imshow('i',img)
	key=cv2.waitKey(1)
	if key==27:
		break
cap.release()

