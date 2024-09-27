#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float32
from inputimeout import inputimeout
from geometry_msgs.msg import WrenchStamped
import numpy as np
import time  # For timing in cubic trajectory

# Global variables
global x, y, z, F, Fd, Fd0, e, ie, e0, uF
F = 0
Fd = 0.5
Fd0 = 0
ie = 0
e0 = 0
uF = 0

def force_callback(data):
    global F
    F = data.wrench.force.x  # Extract the force data from sensor

def save(Fd, F, e, xd):
    # Save force control and position data to a file
    file = open('/media/hameed/Study/PhD_Data/work_with_Santos/Drone_Journal_paper_work/Results/Force.txt', 'a')
    file.write(str(Fd) + ',' + str(F) + ',' + str(e) + ',' + str(xd) + '\n')
    file.close()

Xd = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0], float)

# ROS Initialization
rospy.init_node('Navigation', anonymous=True)

# Publishers and Subscribers
pub = rospy.Publisher('references', Float32MultiArray, queue_size=10)
rospy.Subscriber("/wrench_estimation", WrenchStamped, force_callback)
rate = rospy.Rate(10)  # 10Hz rate

auxFlag = 0

# Cubic Trajectory Variables (for Mode 3)
y0 = 0  # Initial Y position
yf = 1  # Final Y position (can be updated as needed)
update_time = 0.01  # Smaller time interval for smoother trajectory

# Main Loop
while not rospy.is_shutdown():
    if auxFlag == 0:
        mode = int(input("Mode:\n1. Position\n2. Force\n3. Slide on Y-axis (Cubic Trajectory)\n4. Quit\n"))
    
    if mode == 1:
        # Position mode: Set X, Y, Z position for free flight
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
        # Force control mode: Apply force to push the surface
        try:
            Fd = float(inputimeout(prompt="x = \n", timeout=1))

            if Fd < 0:
                auxFlag = 0

        except Exception:
            e = Fd - F
            ie = e + e0
            xd = xd + 0.002 * e + 0.0008 * ie
            print("u = " + str(xd) + " e = " + str(e))
            print("Fd = " + str(Fd) + ", F = " + str(F))
            save(Fd, F, e, xd)
            auxFlag = 1
            e0 = e
        
        take_off = 1
        sw = 0
        array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, take_off, sw, uF]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)
        save(Fd, F, e, xd)
        print("---------")
    
    elif mode == 3:
        # Mode 3: Slide on Y-axis using cubic trajectory
        print("Sliding on Y-axis using cubic trajectory...")
        
        # Initialize xd and zd if they are not defined
        if 'xd' not in globals():
            xd = 0  # Default X position
        if 'zd' not in globals():
            zd = 0  # Default Z position
        
        # Ask user for the final Y position (yf)
        yf = float(input("Enter the final Y position (yf): "))
        
        # Dynamically adjust tf based on yf to allow enough time for larger Y targets
        tf = abs(yf - y0) * 2  # Scaling trajectory duration with yf
        
        # Loop to execute cubic trajectory
        t = 0  # Reset time for trajectory
        while t <= tf:
            # Cubic trajectory equation
            yd = (6 / pow(tf, 2)) * t * (yf - y0) - (6 / pow(tf, 3)) * t * t * (yf - y0)
            
            # Update array with sliding Y-axis position (yd)
            array = [xd, yd, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
            msg = Float32MultiArray(data=array)
            
            # Publish continuously at every step
            pub.publish(msg)
            
            # Print for debugging/logging
            print(f"Time: {t:.2f}s, Y-position: {yd:.4f}")
            
            # Update time for next iteration
            t += update_time
            
            # Delay to simulate real-time execution
            time.sleep(update_time)
            
            # Update initial Y position to the current one (to ensure continuous sliding)
            y0 = yd

        # Stop the drone at the final position by publishing the final Y value
        array = [xd, yf, zd, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0]
        msg = Float32MultiArray(data=array)
        pub.publish(msg)

        print(f"Final Y position reached: {yf:.4f}")
    
    elif mode == 4:
        # Quit the program (formerly Mode 3)
        break

    rate.sleep()

