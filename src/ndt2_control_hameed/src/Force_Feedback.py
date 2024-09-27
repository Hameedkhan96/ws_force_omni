#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
import numpy as np

global Fs
Fs = 0; F0 = 0; F00 = 0; F = 0;

#---Callbacks---------
def force_callback(data):
	global Fs
	if len(data.states) != 0:
		Fs = data.states[0].total_wrench.force.z
	else:
		Fs = 0


a1 = 0.01; a2 = 0.99; a3 = 0.0;

rospy.init_node('Force_Feedback', anonymous=True)
#rospy.Subscriber("/force_sensor_topic",ContactsState,force_callback)
rospy.Subscriber("/ndt2/ft_sensor",ContactsState,force_callback)

pub = rospy.Publisher('/force_feedback', Float32, queue_size=10)

rate = rospy.Rate(200) # 200hz
while not rospy.is_shutdown():
	F = a1*Fs + a2*F0 + a3*F00
	msg = Float32(F)
	pub.publish(msg)
	F0 = F
	F00 = F0
	print(F)
	rate.sleep()
