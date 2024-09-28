#!/usr/bin/env python

import rospy
import numpy as np
from std_msgs.msg import Float64, Float32MultiArray
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import WrenchStamped
import math

# Short cosine and sine functions
def c(x):
    return np.cos(x)

def s(x):
    return np.sin(x)

# Saturation function
def sat(x, val):
    return max(min(x, val), -val)

# Dead band for control
def db(x, val):
    return x if abs(x) > val else 0.0

# Quaternion to Rotation Matrix
def QuatToMat(quat):
    w, x, y, z = quat
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - w*z), 2*(x*z + w*y)],
        [2*(x*y + w*z), 1 - 2*(x*x + z*z), 2*(y*z - w*x)],
        [2*(x*z - w*y), 2*(y*z + w*x), 1 - 2*(x*x + y*y)]
    ])

# Rotation Matrix to XYZ angles
def R2XYZ(R):
    theta = np.arcsin(R[0, 2])
    if abs(np.cos(theta)) > 1e-10:
        phi = np.arctan2(-R[1, 2] / np.cos(theta), R[2, 2] / np.cos(theta))
        psi = np.arctan2(-R[0, 1] / np.cos(theta), R[0, 0] / np.cos(theta))
    else:
        if abs(theta - np.pi / 2) < 1e-5:
            phi = np.arctan2(R[1, 0], R[2, 0])
            theta = np.pi / 2
            psi = 0.0
        else:
            phi = np.arctan2(-R[1, 0], R[2, 0])
            theta = -np.pi / 2
            psi = 0.0
    return np.array([phi, theta, psi])

# Save function
first_write = True
def save(xd, yd, zd, x, y, z, phi, the, psi, fx, wrench_est_force_x):
    global first_write
    with open("Results.txt", "a" if not first_write else "w") as f:
        f.write(f"{xd},{yd},{zd},{x},{y},{z},{phi},{the},{psi},{fx},{wrench_est_force_x}\n")
    first_write = False

# Callbacks for ROS
def joint_states_callback(msg):
    global joints
    joints = msg

def references_callback(msg):
    global references
    references = msg

def odometry_callback(msg):
    global odom
    odom = msg

def wrench_estimation_callback(msg):
    global wrench_est
    wrench_est = msg

# Main function to initialize ROS and start the control loop
def main():
    global motor_vel, errors, references, odom, wrench_est, Fxd

    # Initialize references and odom with default values
    references = Float32MultiArray()
    references.data = [0] * 21  # Assuming the reference data has at least 21 elements

    odom = Odometry()
    odom.pose.pose.position.x = 0
    odom.pose.pose.position.y = 0
    odom.pose.pose.position.z = 0
    odom.twist.twist.linear.x = 0
    odom.twist.twist.linear.y = 0
    odom.twist.twist.linear.z = 0
    odom.pose.pose.orientation.w = 1
    odom.pose.pose.orientation.x = 0
    odom.pose.pose.orientation.y = 0
    odom.pose.pose.orientation.z = 0

    # Initialize wrench_est with default values
    wrench_est = WrenchStamped()
    wrench_est.wrench.force.x = 0
    wrench_est.wrench.force.y = 0
    wrench_est.wrench.force.z = 0
    wrench_est.wrench.torque.x = 0
    wrench_est.wrench.torque.y = 0
    wrench_est.wrench.torque.z = 0

    # Initialize Fxd with a default value
    Fxd = 0.0

    # Initialize motor_vel and errors as Float32MultiArray
    motor_vel = Float32MultiArray()
    motor_vel.data = [0] * 8  # 8 motor velocities

    errors = Float32MultiArray()
    errors.data = [0] * 6  # 6 error values

    rospy.init_node('flight_controller_atquad')

    # Publishers
    rotors_pub = rospy.Publisher('/ndt2/cmd/motor_vel', Float32MultiArray, queue_size=10)
    error_pub = rospy.Publisher('errors', Float32MultiArray, queue_size=10)
    tilt_1_pub = rospy.Publisher('/tilt_motor_1/command', Float64, queue_size=10)
    tilt_2_pub = rospy.Publisher('/tilt_motor_2/command', Float64, queue_size=10)
    tilt_3_pub = rospy.Publisher('/tilt_motor_3/command', Float64, queue_size=10)
    tilt_4_pub = rospy.Publisher('/tilt_motor_4/command', Float64, queue_size=10)
    wrench_pub = rospy.Publisher('/ndt2/cmd/wrench', WrenchStamped, queue_size=10)

    # Subscribers
    rospy.Subscriber('/references', Float32MultiArray, references_callback)
    rospy.Subscriber('/ndt2/odometry', Odometry, odometry_callback)
    rospy.Subscriber('/wrench_estimation', WrenchStamped, wrench_estimation_callback)

    loop_rate = rospy.Rate(1000)

    # Parameters
    eps = np.array([-1, 1, -1, 1])
    S = np.array([np.pi / 4., 3 * np.pi / 4., -3 * np.pi / 4., -np.pi / 4.])

    # Allocator matrix F
    F = np.zeros((6, 8))
    F[0, 4:] = s(S)
    F[1, 4:] = -c(S)
    F[2, :4] = 1
    F[3, :4] = 0.183847763 * s(S)  # l * s(S)
    F[4, :4] = -0.183847763 * c(S)  # -l * c(S)
    F[5, :4] = -eps * (8.54858e-05 / 1.75e-04)  # -eps * sig
    F[3, 4:] = -s(S) * eps * (8.54858e-05 / 1.75e-04)
    F[4, 4:] = c(S) * eps * (8.54858e-05 / 1.75e-04)
    F[5, 4:] = -0.183847763  # -l

    invF = np.linalg.pinv(F)

    vx, vy, vz = 0.0, 0.0, 0.0
    vphi, vthe, vpsi = 0.0, 0.0, 0.0

    while not rospy.is_shutdown():
        take_off = references.data[18]
        sw = references.data[19]

        # References
        xd = db(references.data[0], 1e-2)
        yd = db(references.data[1], 1e-2)
        zd = db(references.data[2], 1e-2)
        phid = db(references.data[3], 1e-2)
        thed = db(references.data[4], 1e-2)
        psid = db(references.data[5], 1e-2)

        # Feedback
        x = odom.pose.pose.position.x
        y = -odom.pose.pose.position.y
        z = -odom.pose.pose.position.z
        dx = odom.twist.twist.linear.x
        dy = -odom.twist.twist.linear.y
        dz = -odom.twist.twist.linear.z

        # Orientation
        quat = [odom.pose.pose.orientation.w, odom.pose.pose.orientation.x, 
                odom.pose.pose.orientation.y, -odom.pose.pose.orientation.z]
        _orientation = R2XYZ(QuatToMat(quat))

        phi, the, psi = _orientation

        # Error signals
        ex = xd - x
        ey = yd - y
        ez = zd - z
        dex = db(references.data[6], 1e-2) - dx
        dey = db(references.data[7], 1e-2) - dy
        dez = db(references.data[8], 1e-2) - dz
        ephi = phid - phi
        ethe = thed - the
        epsi = psid - psi

        # Control signals
        fx = (1 - sw) * (5 * np.tanh(ex) + 4 * dex) + sw * Fxd
        fy = -(5.5 * np.tanh(ey) + 9.0 * dey)
        fz = 180 * np.tanh(ez) + 80 * dez
        tx = 2 * db(references.data[9], 1e-2)
        ty = -(2 * ethe + 10 * ethe)
        tz = 2 * db(references.data[11], 1e-2)

        u = np.array([fx, fy, fz, tx, ty, tz])

        # Wrench messages
        wrench_msg = WrenchStamped()
        wrench_msg.wrench.force.x = fx
        wrench_msg.wrench.force.y = fy
        wrench_msg.wrench.force.z = fz
        wrench_msg.wrench.torque.x = tx
        wrench_msg.wrench.torque.y = ty
        wrench_msg.wrench.torque.z = tz
        wrench_msg.header.stamp = rospy.Time.now()

        # Allocation and tilting
        f = np.dot(invF, u)
        T = np.sqrt(f[:4]**2 + f[4:]**2)
        a = np.arctan2(f[4:], np.abs(f[:4]))

        if take_off == 1.0:
            motor_vel.data = [sat(np.sqrt(T[i] / 8.54858e-05), 1100) for i in range(4)] + [0] * 4
        else:
            motor_vel.data = [0] * 8

        # Publish messages
        wrench_pub.publish(wrench_msg)
        rotors_pub.publish(motor_vel)

        # Save data
        save(xd, yd, zd, x, y, z, phi, the, psi, fx, wrench_est.wrench.force.x)

        loop_rate.sleep()

if __name__ == '__main__':
    main()
