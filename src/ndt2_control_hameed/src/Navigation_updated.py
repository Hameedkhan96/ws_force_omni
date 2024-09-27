#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from inputimeout import inputimeout 
from geometry_msgs.msg import WrenchStamped 
import numpy as np
import os 
import time  # For timing in cubic trajectory

global x,y,z,F,Fd,Fd0,e,ie,e0,uF;
F = 0; Fd = 0.5; Fd0 = 0; ie = 0; e0 = 0;uF = 0;


def force_callback(data):
	global F
	#F = data.data
	# Maybe try with minus (-)
	F = data.wrench.force.x

def save(Fd,F,e,xd):
	  # Save force control and position data to a file
	file = open('/media/hameed/Study/PhD_Data/work_with_Santos/Journal_paper_work/Results/Force.txt','a')
	file.write(str(Fd)+','+str(F)+','+str(e)+','+str(xd)+'\n')
	#file.write(str(Fd)+','+str(F)+'\n')
	file.close()  

#os.remove('/media/hameed/Study/PhD_Data/work_with_Santos/Journal_paper_work/Results/Force.txt')
Xd = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],float)


# ROS Initialization
rospy.init_node('Navigation', anonymous=True)

# Publishers and Subscribers
#---Publishers--------
pub = rospy.Publisher('references', Float32MultiArray, queue_size=10)
#---Subscribers-------
rospy.Subscriber("/wrench_estimation",WrenchStamped,force_callback)
#rospy.Subscriber("/force_feedback",Float32,force_callback)
rate = rospy.Rate(10) # 10hz rate

auxFlag = 0


# Cubic Trajectory Variables (for Mode 4)
y0 = 0  # Initial Y position
yf = 1  # Final Y position (can be updated as needed)
tf = 10  # Trajectory duration (can be customized)
t = 0  # Start time for trajectory
update_time = 1  # Time interval for trajectory updates (can be customized)

# Main Loop
while not rospy.is_shutdown():
	if(auxFlag==0):
		mode = int(input("Mode:\n1. Position\n2. Force\n3. Slide on Y-axis (Cubic Trajectory)\n4. Quit\n"))
		#mode = int(input("Mode:\n1. Position\n2. Force\n3. Quit\n"))
    

	if(mode==1):
		        # Position mode: Set X, Y, Z position for free flight
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

		
	elif(mode==2):    # Force control mode: Apply force to push the surface
		try:
			Fd = float(inputimeout(prompt="x = \n",timeout=1))

			if(Fd)<0:
				auxFlag = 0

		except Exception:
		e = Fd-F
		ie = e + e0
			#uF = uF + 0.01*e+0.005*ie# + 0.1*ie -- 0.001
		xd = xd + 0.002*e + 0.0008*ie
		print("u = "+str(xd)+" e = "+str(e))
		print("Fd = "+str(Fd)+", F = "+str(F))
		#print("u = "+str(uF)+" e = "+str(e))
		save(Fd,F,e,xd)
		auxFlag = 1
		e0 = e
		take_off = 1
		sw = 0	#1
		array = [xd,yd,zd,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,take_off,sw,uF]
		msg = Float32MultiArray(data=array)
		pub.publish(msg)
		save(Fd,F,e,xd)
		print("---------")

    elif(mode==3): # Mode 4: Slide on Y-axis using cubic trajectory
        print("Sliding on Y-axis using cubic trajectory...")
        # Ask user for the final Y position (yf)
        yf = float(input("Enter the final Y position (yf): "))
        # Loop to execute cubic trajectory
        while t <= tf:
            # Cubic trajectory equation
            yd = (6 / pow(tf, 2)) * t * (yf - y0) - (6 / pow(tf, 3)) * t * t * (yf - y0)
            # Update array with sliding Y-axis position (yd)
            array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            # Print for debugging/logging
            print(f"Time: {t:.2f}s, Y-position: {yd:.4f}")
            
            # Update time for next iteration
            t += update_time
            
            # Delay to simulate real-time execution
            time.sleep(update_time)
            
            # Update initial Y position to the current one (to ensure continuous sliding)
            y0 = yd

            # Reset time after completing the trajectory
            t = 0
            print("Cubic trajectory sliding complete.")
	
	
	else:
		break
	 #save data here
	rate.sleep()