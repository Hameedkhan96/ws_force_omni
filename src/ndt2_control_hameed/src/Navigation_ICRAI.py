#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from inputimeout import inputimeout 
import numpy as np

global x,y,z,F,Fd,Fd0,e,ie,e0,uF;
F = 0; Fd = 1; Fd0 = 0; ie = 0; e0 = 0; uF = 0;

# Clear the file at the start of the program
#open('/media/hameed/ros_matlab_force_ws/src/ndt2_control_hameed/src/Results/Force.txt', 'w').close()
open('/media/hameed/Study/PhD_Data/work_with_Santos/Drone_Journal_paper_work/Results/Force.txt', 'w').close()
def force_callback(data):
	global F
	F = data.data

def save(Fd,F,e,xd):
#	file = open('/media/hameed/ros_matlab_force_ws/src/ndt2_control_hameed/src/Results/Force.txt','a')
	file = open('/media/hameed/Study/PhD_Data/work_with_Santos/Drone_Journal_paper_work/Results/Force.txt','a')
	file.write(str(Fd)+','+str(F)+','+str(e)+','+str(xd)+'\n')
	file.close()  


Xd = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],float)

rospy.init_node('Navigation', anonymous=True)
#---Publishers--------
pub = rospy.Publisher('references', Float32MultiArray, queue_size=10)
#---Subscribers-------
rospy.Subscriber("/force_feedback",Float32,force_callback)
rate = rospy.Rate(10) # 10hz

auxFlag = 0

while not rospy.is_shutdown():
	if(auxFlag==0):
		mode = int(input("Mode:\n1. Position\n2. Force\n3. Quit\n"))
	if(mode==1):
		print("Desired position\n")
		xd = float(input("x = \n"))
		yd = float(input("y = \n"))
		zd = float(input("z = \n"))
		take_off = 1
		sw = 0
		array = [xd,yd,zd,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,take_off,sw,0]
		msg = Float32MultiArray(data=array)
		pub.publish(msg)
		print("---------")
	elif(mode==2):
		try:
			Fd = float(inputimeout(prompt="x = \n",timeout=1))
			if(Fd)<0:
				auxFlag = 0

		except Exception:
			e = Fd-F; ie = e + e0
			xd = xd + 0.002*e + 0.0008*ie
			print("u = "+str(xd)+" e = "+str(e))
			print("Fd = "+str(Fd)+", F = "+str(F))
			save(Fd,F,e,xd)
			auxFlag = 1
			e0 = e
		take_off = 1
		sw = 0	#1
		array = [xd,yd,zd,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,take_off,sw,uF]
		msg = Float32MultiArray(data=array)
		pub.publish(msg)
		print("---------")
	else:
		break
	rate.sleep()
