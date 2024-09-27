#!/usr/bin/python3
"""
Content: Wrench estimator
Author: Julien Mellet
Date: 2024-08-11
Description: A Python script to estimate external wrench applied on the robot through ROS1.
"""

import rospy
from geometry_msgs.msg import WrenchStamped, TwistStamped
import numpy as np

class WrenchEstimation:
    def __init__(self):

        self.odom_sub = rospy.Subscriber("/ndt2/local_vel", TwistStamped, self.odom_callback)
        self.torque_sub = rospy.Subscriber("/ndt2/cmd/wrench", WrenchStamped, self.wrench_callback)

        self.wrench_estimation_pub = rospy.Publisher('/wrench_estimation', WrenchStamped, queue_size=1)

        self.timer_period = 0.033  # 30 Hz
        # Création du timer
        self.timer = rospy.Timer(rospy.Duration(self.timer_period), self.calculate_wrench_estimation)

        self.vehicle_mass = 6.3833
        self.vehicle_inertia = np.array([[0.1943, 0, 0],
                                         [0, 0.1943, 0],
                                         [0, 0, 0.235]])
        self.ext_lin_acc = np.zeros(3)
        self.ext_ang_acc = np.zeros(3)
        self.int_vec_lin = np.zeros(3)
        self.int_vec_ang = np.zeros(3)
        self.cmd_torque = np.zeros(3)
        self.cmd_force = np.zeros(3)
        #self.cmd_wrench = np.zeros(6)
        self.offset = np.zeros(6)
        self.offset[2] = -30.870
        self.offset[1] = 0.462

        self.gravity = np.array([0, 0, 9.81])

        self.lin_vel_B = np.zeros(3)
        self.ang_vel_B = np.zeros(3)

        self.k_I_lin = np.array([0.9, 0.9, 0.9])
        self.k_I_ang = np.array([0.9, 0.9, 0.9])

        self.wrench_stamped = WrenchStamped()
    
    def odom_callback(self, msg):
        try:
            self.lin_vel_B = np.array([msg.twist.linear.x,
                                       msg.twist.linear.y,
                                       msg.twist.linear.z])
            self.ang_vel_B = np.array([msg.twist.angular.x,
                                       msg.twist.angular.y,
                                       msg.twist.angular.z])
        except ValueError:
            pass


    def wrench_callback(self, msg):
        try:
            #print(msg)
            self.cmd_force = np.array([msg.wrench.force.x,
                                       msg.wrench.force.y,
                                       msg.wrench.force.z])
            
            self.cmd_torque = np.array([msg.wrench.torque.x,
                                        msg.wrench.torque.y,
                                        msg.wrench.torque.z])
        except ValueError:
            pass

    def calculate_wrench_estimation(self, event):
        # Wrench command
        cmd_lin_acc = 1.0 / self.vehicle_mass * self.cmd_force
        cmd_ang_acc = np.linalg.inv(self.vehicle_inertia).dot(self.cmd_torque)

        # TODO: rotate gravity vector with drone attitude
        self.grav_B = self.gravity

        # Compute nonlinear terms
        n_ang = np.linalg.inv(self.vehicle_inertia).dot(-np.cross(self.vehicle_inertia.dot(self.ang_vel_B), self.ang_vel_B))
        n_lin = np.cross(self.ang_vel_B, self.lin_vel_B) + self.grav_B

        # Compute the integral term using forward Euler
        self.int_vec_lin += (cmd_lin_acc + self.ext_lin_acc - n_lin) * self.timer_period
        self.int_vec_ang += (cmd_ang_acc + self.ext_ang_acc - n_ang) * self.timer_period

        # Update estimation
        self.ext_lin_acc = (self.lin_vel_B - self.int_vec_lin) * self.k_I_lin
        self.ext_ang_acc = (self.ang_vel_B - self.int_vec_ang) * self.k_I_ang

        # Compute force and torque
        force = self.vehicle_mass * self.ext_lin_acc
        torque = np.matmul(self.vehicle_inertia, self.ext_ang_acc)

        self.wrench_stamped.header.stamp = rospy.Time.now()
        self.wrench_stamped.wrench.force.x =  force[0]  + self.offset[0]
        self.wrench_stamped.wrench.force.x =  -  self.wrench_stamped.wrench.force.x 
        self.wrench_stamped.wrench.force.y =  force[1]  + self.offset[1]
        self.wrench_stamped.wrench.force.z =  force[2]  + self.offset[2] 
        self.wrench_stamped.wrench.torque.x = torque[0] + self.offset[3]
        self.wrench_stamped.wrench.torque.y = torque[1] + self.offset[4]
        self.wrench_stamped.wrench.torque.z = torque[2] + self.offset[5]

        self.wrench_estimation_pub.publish(self.wrench_stamped)

if __name__ == '__main__':
    try:
        rospy.init_node('wrench_estimation_node', anonymous=True)

        wrench_est = WrenchEstimation()
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
