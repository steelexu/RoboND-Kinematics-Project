#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from sympy.matrices import Matrix

import numpy as np

from numpy import array
import matplotlib.pyplot as plt

def individual_transformation_matrix( theta, alpha, a, d ):
    T = Matrix([[             cos(theta),             -sin(theta),            0,             a],
                   [ sin(theta)*cos(alpha),  cos(theta)*cos(alpha), -sin(alpha),-sin(alpha)*d],
                   [ sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha), cos(alpha)*d],
                   [                   0,                    0,            0,               1]])
    return T

def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        #angle_error = []





        dg=0.303
        s.update({ax:1.501})

        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()
            # Extract end-effector position and orientation from request
	    # px,py,pz = end-effector position
	    # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
     
            # Calculate joint angles using Geometric IK method
            R_rpy=tf.transformations.euler_matrix(yaw,pitch,roll,'rzyx')
            # get the wrist position
            wx=px-dg*R_rpy[0,0]
            wy=py-dg*R_rpy[1,0]
            wz=pz-dg*R_rpy[2,0]
            #r0_6x=R_rpy[:3,:3]*R_corr0[:3,:3].inv()*T6_G[:3,:3].inv()
            #prepare the numerical T0_6[:3,:3]
            r0_6x=R_rpy[:3,:3]*R_corr0[:3,:3]

            r=sqrt(wx**2+wy**2)

            cc=sqrt((r-a1)**2+(wz-d1)**2)
            #cc=cc.subs(s)

            cosbeta1=(a2**2+ax**2-cc**2)/(2*ax*a2)
            

            cosbeta2=(a2**2-ax**2+cc**2)/(2*cc*a2)
            

            theta1=atan2(wy,wx)
            theta3=(pi/2-atan2(sqrt(1-cosbeta1**2),cosbeta1)-asin(-a3/d4)).evalf(subs=s)     
            theta2=(pi/2-atan2(sqrt(1-cosbeta2**2),cosbeta2)-atan2(wz-d1,r-a1).subs(s)).evalf(subs=s)


            r0_3_all=T0_3.subs({q1:theta1,q2:theta2,q3:theta3})
            r0_3=r0_3_all[:3,:3]
            #r0_3=T0_3.subs({q1:theta1,q2:theta2,q3:theta3})[:3,:3]
            r3_6=r0_3.T.evalf(subs={})*r0_6x

            #theta4=atan2(r3_6[2,2],-1*r3_6[0,2])
            #theta6=atan2(r3_6[1,1],-1*r3_6[1,0])

            theta5=atan2(sqrt(r3_6[0,2]**2 + r3_6[2,2]**2), r3_6[1,2])
            if sin(theta5) < 0:
                theta4 = atan2(-r3_6[2,2], r3_6[0,2])
                theta6 = atan2(r3_6[1,1], -r3_6[1,0])
            else:
                theta4 = atan2(r3_6[2,2], -r3_6[0,2])
                theta6 = atan2(-r3_6[1,1], r3_6[1,0])
		


            # calculate the error
            print theta1,theta2,theta3,theta4,theta5,theta6
            T_total = r0_3_all*T3_G_corr.evalf(subs={q4:theta4,q5:theta5,q6:theta6,q7:0})

            ee_x_e = abs(T_total[0,3]-px)
            ee_y_e = abs(T_total[1,3]-py)
            ee_z_e = abs(T_total[2,3]-pz)
            ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
            print("ee_offset",ee_offset)
            ee__error_all.append(ee_offset)
            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":

    # Define DH param symbols
    # Joint angle symbols

    # Modified DH params
    q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i

    d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #d_i

    a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #a_i
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
    #dg=symbols('dg')
    cc=symbols('cc')
    ax=symbols('ax')
    s = {alpha0:       0,  a0:       0,  d1:       0.75,
         alpha1:   -pi/2,  a1:    0.35,  d2:      0,  q2:       q2-pi/2,
         alpha2:       0,  a2:    1.25,  d3:      0,  
         alpha3:   -pi/2,  a3:  -0.054,  d4:       0.96+0.54,
         alpha4:    pi/2,  a4:       0,  d5:      0,
         alpha5:   -pi/2,  a5:       0,  d6:      0,
         alpha6:       0,  a6:       0,  d7:      0.303,  q7:       0}


    # Define Modified DH Transformation matrix



    # Create individual transformation matrices

    T0_1 =individual_transformation_matrix(q1,alpha0,a0,d1)
    T0_1 = T0_1.subs(s)

    T1_2 =individual_transformation_matrix(q2,alpha1,a1,d2)
    T1_2 = T1_2.subs(s)

    T2_3 =individual_transformation_matrix(q3,alpha2,a2,d3)
    T2_3 =T2_3.subs(s)

    T0_2 = simplify(T0_1*T1_2)
    T0_3 = simplify(T0_2*T2_3)

    T6_G =individual_transformation_matrix(q7,alpha6,a6,d7)
    T6_G =T6_G.subs(s)

    #correction matrix
    R_z = Matrix([[  cos(pi),    -sin(pi),      0,    0],
                [  sin(pi),     cos(pi),      0,    0],
                [      0 ,             0 ,      1,    0],
                [      0 ,             0 ,      0,    1]])

    R_y = Matrix([[  cos(-pi/2),          0 , sin(-pi/2),     0],
                [         0 ,          1 ,        0 ,     0],
                [ -sin(-pi/2),          0 , cos(-pi/2),     0],
                [         0 ,          0 ,        0 ,     1]])
    #R_corr = simplify(R_z*R_y)
    R_corr0=Matrix([[0, 0,1,0],
                    [0, -1, 0,0],
                    [1, 0,0,0],
                    [0, 0,0,1]])


    T3_4 =individual_transformation_matrix(q4,alpha3,a3,d4)
    T3_4 = T3_4.subs(s)

    T4_5 =individual_transformation_matrix(q5,alpha4,a4,d5)
    T4_5 = T4_5.subs(s)

    T5_6 =individual_transformation_matrix(q6,alpha5,a5,d6)
    T5_6 = T5_6.subs(s)


    #T0_4 = simplify(T0_3*T3_4)

    #T0_5 = simplify(T0_4*T4_5)
    #T0_6 = simplify(T0_5*T5_6)
    #T0_G = simplify(T0_6*T6_G)
    #T3_6 = simplify(T3_4*T4_5*T5_6)
    #T_total = simplify(T0_3*T3_6*T6_G*R_corr0).evalf(subs={q1:theta1,q2:theta2,q3:theta3,q4:theta4,q5:theta5,q6:theta6,q7:0})
    T3_G_corr = simplify(T3_4*T4_5*T5_6*T6_G*R_corr0)
    ee__error_all = []


    IK_server()
    print("ee_offset",ee__error_all)
    plt.plot(ee__error_all)
    plt.show()
