#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray
from inputimeout import inputimeout, TimeoutOccurred 
from geometry_msgs.msg import WrenchStamped 
import numpy as np
import os 
import time  # For timing in cubic trajectory

global x, y, z, F, Fd, Fd0, e, ie, e0, uF
F = 0; Fd = 0.5; Fd0 = 0; ie = 0; e0 = 0; uF = 0

def force_callback(data):
    global F
    F = data.wrench.force.x

def save(Fd, F, e, xd):
    # Save force control and position data to a file
    try:
        with open('/home/hameed/Force_2.txt', 'a') as file:  # Use a writable directory (e.g., home folder)
            file.write(str(Fd)+','+str(F)+','+str(e)+','+str(xd)+'\n')
    except OSError as err:
        print(f"Error saving file: {err}")

Xd = np.array([0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0],float)

# ROS Initialization
rospy.init_node('Navigation', anonymous=True)

# Publishers and Subscribers
pub = rospy.Publisher('references', Float32MultiArray, queue_size=10)
rospy.Subscriber("/wrench_estimation", WrenchStamped, force_callback)

rate = rospy.Rate(10)  # 10hz rate

y0 = 0  # Initial Y position
yf = 1  # Final Y position (can be updated as needed)
#tf = 10  # Trajectory duration (can be customized)
update_time = 1
tf = 10  # Trajectory duration

# Main Loop
while not rospy.is_shutdown():
    if auxFlag == 0:
        mode = int(input("Mode:\n1. Position\n2. Force\n3. Slide on Y-axis (Cubic Trajectory)\n4. Quit\n"))

    if mode == 1:
        # Position mode
        print("Desired position\n")
        xd = float(input("x = \n"))
        yd = float(input("y = \n"))
        zd = float(input("z = \n"))
        take_off = 1
        sw = 0
        array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, take_off, sw, 0]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)
        print("---------")

    elif mode == 2:
        # Force control mode
        try:
            Fd = float(inputimeout(prompt="x = \n", timeout=1))

            if Fd < 0:
                auxFlag = 0

        except Exception:
            
        e = Fd - F
        ie = e + e0
        xd = xd + 0.002 * e + 0.0008 * ie
        print("u = "+str(xd)+" e = "+str(e))
		print("Fd = "+str(Fd)+", F = "+str(F))
        save(Fd, F, e, xd)
        auxFlag = 1
        e0 = e
        take_off = 1
        sw = 0
        array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, take_off, sw, uF]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)
        save(Fd,F,e,xd)
        print("---------")

    elif mode == 3:
        # Slide on Y-axis using cubic trajectory
        print("Sliding on Y-axis using cubic trajectory...")
        yf = float(input("Enter the final Y position (yf): "))

        while t <= tf:
            yd = (6 / pow(tf, 2)) * t * (yf - y0) - (6 / pow(tf, 3)) * t * t * (yf - y0)
            array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            pub.publish(msg)
            print(f"Time: {t:.2f}s, Y-position: {yd:.4f}")
            t += update_time
            time.sleep(update_time)
            y0 = yd

        t = 0
        print("Cubic trajectory sliding complete.")

    else:
        break

    rate.sleep()
