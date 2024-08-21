import sys
import tf
import copy
from canard import can
from canard.hw import cantact
import rospy
import time
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import sensor_msgs.msg
from std_msgs.msg import Empty
from std_msgs.msg import Float64
import math
angle = 0
target_angle = 0
speed = 500
accel = 1000
count = 0
data1 = sensor_msgs.msg.JointState()
dev = cantact.CantactDev("/dev/ttyACM0")
dev.set_bitrate(1000000)
dev.start()

scanForwardFlag = False
scanBackwardFlag = False
weldFlag = False
spinFlag = False

# FUNCTIONS FOR CONTROLLING ROBOT

def init_robot():
	frame=can.Frame(1)
	frame.id=0xff
	frame.dlc=4
	frame.data=[0x02,0x30,3,0]
	dev.send(frame)
	print(frame)

	frame.data=[0x02, 0x0a, 1,0]
	dev.send(frame)
	print(frame)

	frame.data=[0x02,0x41, speed%256, speed/256]
	dev.send(frame)
	print(frame)

	frame.data=[0x02, 0x42, accel%256, accel/256]
	dev.send(frame)
	print(frame)

	print("Done initializing robot")

def robot_move_to_joint_angle(joint_num,angle):
	if joint_num<=3:
		angle=int((float(angle)/360.0)*363.0*4096.0)
	else :
		angle=int((float(angle)/360.0)*303.0*4096.0)
	frame=can.Frame(1)
	frame.id=joint_num
	frame.dlc=6
	frame.data=[0x02, 0x36, (angle>>0)&0xff,(angle>>8)&0xff,(angle>>16)&0xff,(angle>>24)&0xff]
	dev.send(frame)

def set_speed(max_speed, max_acc):
	frame=can.Frame(1)
	frame.id=0xff
	frame.dlc=4
	frame.data=[0x02,0x41,int(max_speed%256),int(max_speed/256)]
	dev.send(frame)
	frame.data=[0x02, 0x42, int(max_acc%256), int(max_acc/256)]
	dev.send(frame)

# CALLBACKS FOR ROS

def scan_callback(data):
	robot_move_to_joint_angle(6, -175)
	scanBackwardFlag = True
	target_angle = -175

def weld_callback(data):
	pass

def spin_callback(data):
	spinFlag = True
	robot_move_to_joint_angle(6, 175)
	target_angle = 175

def stop_callback(data):
	spinFlag = False
	weldFlag = False
	scanBackwardFlag = False
	scanForwardFlag = False

def speed_callback(data):
	speed = data.data
	set_speed(speed, accel)
	print("Speed set to: " + str(speed))
	print("Acceleration set to: " + str(accel))

def accel_callback(data):
	accel = data.data
	set_speed(speed, accel)
	print("Speed set to: " + str(speed))
	print("Acceleration set to: " + str(accel))

def change_angle_callback(data):
	target_angle = data.data
	robot_move_to_joint_angle(6, target_angle)
	print("Change angle to: " + str(target_angle))

def joint_state_callback(data):
	angle = data.position[4]

if __name__ == "__main__":
	set_speed(speed, accel)

	if (scanBackwardFlag and angle >= -180*math.pi/180 and angle <= -174*math.pi/180):
		target_angle = 175
		robot_move_to_joint_angle(6, 175)
		scanBackwardFlag = False
		scanForwardFlag = True

	if (scanForwardFlag and angle >= 174*math.pi/180 and angle <= 180*math.pi/180):
		target_angle = 0
		robot_move_to_joint_angle(6, 0)
		scanForwardFlag = False

	if (spinFlag):
		if (angle >= -180*math.pi/180 and angle <= -174*math.pi/180):
			target_angle = 175
			robot_move_to_joint_angle(6, 175)
		elif (angle >= 174*math.pi/180 and angle <= 180*math.pi/180):
			target_angle = -175
			robot_move_to_joint_angle(6, -175)

	rospy.init_node('listener', anonymous=True)
	rospy.Subscriber("scan", Empty, scan_callback)
	rospy.Subscriber("weld", Empty, weld_callback)
	rospy.Subscriber("spin", Empty, spin_callback)
	rospy.Subscriber("stop", Empty, stop_callback)
	rospy.Subscriber("rev_speed", Float64, speed_callback)
	rospy.Subscriber("rev_accel", Float64, accel_callback)
	rospy.Subscriber("change_angle", Float64, change_angle_callback)
	rospy.Subscriber("joint_states", sensor_msgs.msg.JointState, joint_state_callback)
	rospy.spin()
