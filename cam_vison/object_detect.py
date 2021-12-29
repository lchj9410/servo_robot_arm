import cv2
import numpy as np
from math import *

def rot(axis,angle): #rotation matrix 
	if axis=='x':
		R=np.array([[1,0,0],[0, cos(angle),- sin(angle)],[0, sin(angle), cos(angle)]])
	if axis=='y':
		R=np.array([[ cos(angle),0, sin(angle)],[0,1,0],[- sin(angle),0, cos(angle)]])
	if axis=='z':
		R=np.array([[ cos(angle),- sin(angle),0],[ sin(angle), cos(angle),0],[0,0,1]])
	return R
camera_rotation=rot('z',40*pi/180).dot(rot('x',(32-90)*pi/180))  # 40~50?
def object_vector(coordinates):
	cx=coordinates[0]
	cy=coordinates[1]
	B=(cx-width/2)/(width/2)*0.5458+0.00001
	A=-(cy-height/2)/(height/2)*0.4357
	vec=np.array([1,1/tan(B),tan(A)/tan(B)])
	return vec

threshold=0.4
net=cv2.dnn.readNet('C:/cam_vison/yolov4-tiny.weights','C:/cam_vison/yolov4-tiny.cfg')
# net=cv2.dnn.readNet('C:/cam_vison/yolov4-obj_best.weights','C:/cam_vison/yolov4-obj.cfg')
net.setPreferableBackend(cv2.dnn.DNN_BACKEND_CUDA)
net.setPreferableTarget(cv2.dnn.DNN_TARGET_CUDA)
classes=[]
with open('C:/cam_vison/coco.names','r') as f:
	classes = f.read().splitlines()

cam_url = "http://lcj005:woaistorm5@192.168.0.12:8080/video"
cap = cv2.VideoCapture(cam_url)
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
				if str(classes[class_id])=='cup':
					estimate_coordinate=np.array([x-w/8,y+h/3])
					obj_vec=object_vector(estimate_coordinate)
					obj_vec=camera_rotation.dot(obj_vec)
					obj_x=obj_vec[0]*(-0.47/obj_vec[2])
					obj_y=obj_vec[1]*(-0.47/obj_vec[2])
					print(obj_x,obj_y)
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

	cv2.imshow('i',img)
	key=cv2.waitKey(1)
	if key==27:
		break
cap.release()

