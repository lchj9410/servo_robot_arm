import rospy
import numpy as np
from pyquaternion import Quaternion
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Point
from math import *
from my_arm_jacobian import jacobian
from my_arm_jacobian6 import jacobian6
from my_arm_FK import FK
rospy.init_node('jacobian_IK', anonymous=True)
rate = rospy.Rate(1000)

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X =  atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y =  asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z =  atan2(t3, t4)
    return X, Y, Z

shoulder_controller = rospy.Publisher('shoulder_controller/command', Float64, queue_size=1)
elbow_controller = rospy.Publisher('elbow_controller/command', Float64, queue_size=1)
turn_controller = rospy.Publisher('turn_controller/command', Float64, queue_size=1)
wristx_controller = rospy.Publisher('wristx_controller/command', Float64, queue_size=1)
wristy_controller = rospy.Publisher('wristy_controller/command', Float64, queue_size=1)
wristz_controller = rospy.Publisher('wristz_controller/command', Float64, queue_size=1)

[elbow, shoulder, turn, wristx, wristy, wristz]=np.zeros(6)
[elbowv, shoulderv, turnv, wristxv, wristyv, wristzv]=np.zeros(6)
def JointState_callback(data):
	global elbow, shoulder, turn, wristx, wristy, wristz, elbowv, shoulderv, turnv, wristxv, wristyv, wristzv
	[elbow, shoulder, turn, wristx, wristy, wristz]=data.position
	[elbowv, shoulderv, turnv, wristxv, wristyv, wristzv]=data.velocity


rospy.Subscriber('/joint_states',JointState,JointState_callback,queue_size=1)

wrist_z_pose=[0,0,0,0,0,0]
wrist_vel=[0,0,0]
wrist_pos=[0,0,0]
def link_state_callback(data):
	global wrist_z_pose,wrist_vel,wrist_pos
	roll,pitch,yaw=quaternion_to_euler(data.pose[8].orientation.x,data.pose[8].orientation.y,data.pose[8].orientation.z,data.pose[8].orientation.w)
	wrist_z_pose=[data.pose[8].position.x,data.pose[8].position.y,data.pose[8].position.z,roll,pitch,yaw]
	wrist_pos=np.array([data.pose[8].position.x,data.pose[8].position.y,data.pose[8].position.z])
	wrist_vel=[data.twist[8].linear.x, data.twist[8].linear.y, data.twist[8].linear.z]
rospy.Subscriber('/gazebo/link_states', LinkStates, link_state_callback,queue_size=1)

end_effector_pose=np.array([0.3,0.3,0.34, 0.0,0.9,-0.05]) #####################################################
def end_effector_position_callback(data):
	global end_effector_pose
	end_effector_pose[0:3]=np.array([data.x,data.y,data.z])
rospy.Subscriber('/cmd_position', Point, end_effector_position_callback,queue_size=1)

def end_effector_orientation_callback(data):
	global end_effector_pose
	end_effector_pose[3:6]=np.array([data.x,data.y,data.z])
rospy.Subscriber('/cmd_orientation', Point, end_effector_orientation_callback,queue_size=1)

def rot(axis,angle): #rotation matrix 
	if axis=='x':
		R=np.array([[1,0,0],[0, cos(angle),- sin(angle)],[0, sin(angle), cos(angle)]])
	if axis=='y':
		R=np.array([[ cos(angle),0, sin(angle)],[0,1,0],[- sin(angle),0, cos(angle)]])
	if axis=='z':
		R=np.array([[ cos(angle),- sin(angle),0],[ sin(angle), cos(angle),0],[0,0,1]])
	return R

Base=np.array([0,0,0.1])
turn_table=np.array([0,0,0.1])
larm=np.array([0,0,0.3])
uarm=np.array([0,0,0.35])
wrist=np.array([0,0,0.05])
def forward_kinematic():
	End_effector=Base+rot('z',turn).dot(turn_table)+ rot('z',turn).dot(rot('x',shoulder).dot(larm)) + rot('z',turn).dot(rot('x',shoulder).dot(rot('x',elbow).dot(uarm))) + \
			rot('z',turn).dot(rot('x',shoulder).dot(rot('x',elbow).dot(rot('x',wristx).dot(wrist))))+\
			rot('z',turn).dot(rot('x',shoulder).dot(rot('x',elbow).dot(rot('x',wristx).dot(rot('y',wristy).dot(wrist)))))+\
			rot('z',turn).dot(rot('x',shoulder).dot(rot('x',elbow).dot(rot('x',wristx).dot(rot('y',wristy).dot(rot('z',wristz).dot(wrist))))))
	z_axis=rot('z',turn).dot(rot('x',shoulder).dot(rot('x',elbow).dot(rot('x',wristx).dot(rot('y',wristy).dot(rot('z',wristz).dot(np.array([0,0,1])))))))
	y_axis=rot('z',turn).dot(rot('x',shoulder).dot(rot('x',elbow).dot(rot('x',wristx).dot(rot('y',wristy).dot(rot('z',wristz).dot(np.array([0,1,0])))))))
	x_axis=rot('z',turn).dot(rot('x',shoulder).dot(rot('x',elbow).dot(rot('x',wristx).dot(rot('y',wristy).dot(rot('z',wristz).dot(np.array([1,0,0])))))))
	End_effector=np.append(End_effector,np.array([x_axis[1],y_axis[0],z_axis[2]]))
	return End_effector

def IK_jacobian(v): 
	J=jacobian6(elbow, shoulder, turn, wristx, wristy, wristz)
	J=np.transpose(J)
	# measured_joint_vel=np.array([elbowv, shoulderv, turnv, wristxv, wristyv, wristzv])
	# vv=J.dot(measured_joint_vel)
	# measured_vel=np.array(wrist_vel)
	#print(vv, measured_vel)
	pinvJ=np.linalg.pinv(J)
	# pinvJ=np.transpose(pinvJ)
	theta_dot=pinvJ.dot(v)




	return theta_dot

# def check_reachable(side,Target_foot_position):

def move_arm(target):
	global error_int
	error=forward_kinematic()-target
	error_int+=error*dt
	v=-error*kp-error_int*ki

	
	theta_dot=IK_jacobian(v)
	# [elbow, shoulder, turn, wristx, wristy, wristz]
	elbow_controller.publish(theta_dot[0])
	shoulder_controller.publish(theta_dot[1])
	turn_controller.publish(theta_dot[2])
	wristx_controller.publish(theta_dot[3])
	wristy_controller.publish(theta_dot[4])
	wristz_controller.publish(theta_dot[5])


error_int=np.zeros(6)
dt=0.01
angle_k=2
kp=3*np.array([1,1,1,angle_k,angle_k,angle_k])
ki=0.5*np.array([1,1,1,angle_k,angle_k,angle_k])

while not rospy.is_shutdown():
	rospy.sleep(dt)
	move_arm(end_effector_pose)


