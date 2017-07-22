#!/usr/bin/env python
import rospy
import tf

import numpy as np
from numpy import array
from sympy import symbols, cos, sin, pi, simplify, sqrt, atan2
from sympy.matrices import Matrix

# Create symbols for joint varialbles
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') #d_i
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') #a_i
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')


# Define KUKA KR210 DH Parameters
s = {alpha0:       0,  a0:       0,  d1:       0.75,
     alpha1:   -pi/2,  a1:    0.35,  d2:          0,  q2:       q2-pi/2,
     alpha2:       0,  a2:    1.25,  d3:          0,  
     alpha3:   -pi/2,  a3:  -0.054,  d4:       0.96+0.54,
     alpha4:    pi/2,  a4:       0,  d5:          0,
     alpha5:   -pi/2,  a5:       0,  d6:          0,
     alpha6:       0,  a6:       0,  d7:      0.303,  q7:       0}

# Create individual Homogeneous transformation matrices
#base_link to link1
T0_1 = Matrix([[             cos(q1),             -sin(q1),            0,             a0],
               [ sin(q1)*cos(alpha0),  cos(q1)*cos(alpha0), -sin(alpha0),-sin(alpha0)*d1],
               [ sin(q1)*sin(alpha0),  cos(q1)*sin(alpha0),  cos(alpha0), cos(alpha0)*d1],
               [                   0,                    0,            0,               1]])
T0_1 = T0_1.subs(s)

T1_2 = Matrix([[             cos(q2),             -sin(q2),            0,             a1],
               [ sin(q2)*cos(alpha1),  cos(q2)*cos(alpha1), -sin(alpha1),-sin(alpha1)*d2],
               [ sin(q2)*sin(alpha1),  cos(q2)*sin(alpha1),  cos(alpha1), cos(alpha1)*d2],
               [                   0,                    0,            0,   1]])    
T1_2 = T1_2.subs(s)

T2_3 = Matrix([[             cos(q3),             -sin(q3),            0,             a2],
               [ sin(q3)*cos(alpha2),  cos(q3)*cos(alpha2), -sin(alpha2),-sin(alpha2)*d3],
               [ sin(q3)*sin(alpha2),  cos(q3)*sin(alpha2),  cos(alpha2), cos(alpha2)*d3],
               [                   0,                    0,            0,       1]])    
T2_3 = T2_3.subs(s)

T3_4 = Matrix([[             cos(q4),             -sin(q4),            0,             a3],
               [ sin(q4)*cos(alpha3),  cos(q4)*cos(alpha3), -sin(alpha3),-sin(alpha3)*d4],
               [ sin(q4)*sin(alpha3),  cos(q4)*sin(alpha3),  cos(alpha3), cos(alpha3)*d4],
               [                   0,                    0,            0,           1]])
T3_4 = T3_4.subs(s)

T4_5 = Matrix([[             cos(q5),             -sin(q5),            0,             a4],
               [ sin(q5)*cos(alpha4),  cos(q5)*cos(alpha4), -sin(alpha4),-sin(alpha4)*d5],
               [ sin(q5)*sin(alpha4),  cos(q5)*sin(alpha4),  cos(alpha4), cos(alpha4)*d5],
               [                   0,                    0,            0,           1]])  
T4_5 = T4_5.subs(s) 
 
T5_6 = Matrix([[             cos(q6),             -sin(q6),            0,             a5],
               [ sin(q6)*cos(alpha5),  cos(q6)*cos(alpha5), -sin(alpha5),-sin(alpha5)*d6],
               [ sin(q6)*sin(alpha5),  cos(q6)*sin(alpha5),  cos(alpha5), cos(alpha5)*d6],
               [                   0,                    0,            0,               1]])
T5_6 = T5_6.subs(s)

T6_G = Matrix([[             cos(q7),             -sin(q7),            0,             a6],
               [ sin(q7)*cos(alpha6),  cos(q7)*cos(alpha6), -sin(alpha6),-sin(alpha6)*d7],
               [ sin(q7)*sin(alpha6),  cos(q7)*sin(alpha6),  cos(alpha6), cos(alpha6)*d7],
               [                   0,                    0,            0,               1]])
T6_G = T6_G.subs(s)

#composition of Homogeneous Transforms
T0_2 = simplify(T0_1*T1_2)
T0_3 = simplify(T0_2*T2_3)
T0_4 = simplify(T0_3*T3_4)
T0_5 = simplify(T0_4*T4_5)
T0_6 = simplify(T0_5*T5_6)
T0_G = simplify(T0_6*T6_G)


T3_6 = simplify(T3_4*T4_5*T5_6)
#Correction Needed to Account of Orientation Difference Between Definition of Gripper Link in URDF versus DH Convention
R_z = Matrix([[  cos(np.pi),        -sin(np.pi),          0,        0],
              [  sin(np.pi),         cos(np.pi),          0,        0],
              [          0 ,                 0 ,          1,        0],
              [          0 ,                 0 ,          0,        1]])
R_y = Matrix([[  cos(-np.pi/2),              0 , sin(-np.pi/2),     0],
              [             0 ,              1 ,            0 ,     0],
              [ -sin(-np.pi/2),              0 , cos(-np.pi/2),     0],
              [             0 ,              0 ,            0 ,     1]])
R_corr = simplify(R_z*R_y)
#print("T0_1 = ",T0_1.evalf(subs={q1:1.83,q2:0,q3:0,q4:0,q5:0,q6:0,q7:0}))
#print("T0_2 = ",T0_2.evalf(subs={q1:1.83,q2:0,q3:0,q4:0,q5:0,q6:0,q7:0}))
#print("T0_3 = ",T0_3.evalf(subs={q1:1.83,q2:0,q3:0,q4:0,q5:0,q6:0,q7:0}))
print("T0_4 = ",T0_4.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))
print("T0_5 = ",T0_5.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))
#Total Homogeneous Transform Between Base_link and Gripper_link with Orientation Correction Applied
T_total = simplify(T0_G*R_corr)
alpha = atan2(T_total[1,0],T_total[0,0])  
beta  = atan2(-T_total[2,0],sqrt(T_total[1,0]**2+T_total[0,0]**2)) 
gamma =  atan2(T_total[2,1],T_total[2,2]) 
#roslaunch kuka_arm forward_kinematics.launch
tt36=T3_6.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0})
tt03=T0_3.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0})
tt06=T0_6.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0})

print("T0_6 = ",T0_6.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))
print("T0_3 = ",T0_3.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))
print("T3_6 = ",T3_6.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))
print("T3_6_symbol q5=0   ",simplify(T3_6.subs({q5:0})))
print("T3_6x = ",tt03.inv()*tt06)


print("T_total = ",T_total.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))
print(alpha.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))
print(beta.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))
print(gamma.evalf(subs={q1:1.82,q2:0.28,q3:0.25,q4:0,q5:0,q6:0,q7:0}))


print("T3_6-euler",tf.transformations.euler_from_matrix(np.array(tt36).astype(np.float64)))


print("T0_3_symbol = ",simplify(T0_3))

print("T3_6_symbol = ",simplify(T3_6))

print("T3_6_symbol = ",simplify(T3_6.subs({})))

# Extract end-effector position and orientation from request
#
#
#     
# Calculate joint angles using Geometric IK method
#
#
##################################################
####
# Create symbols



#rz(a)ry(b)rx(c) = Matrix([[ cos(a)*cos(b), cos(a)*sin(b)*sin(c)-sin(a)*cos(c), cos(a)*sin(b)*cos(c)+sin(a)*sin(c)],
#               [ sin(a)*cos(b), sin(a)*sin(b)*sin(c)+cos(a)*cos(c), sin(a)*sin(b)*cos(c)-cos(a)*sin(c)],
#               [ -sin(b), cos(b)*sin(c),  cos(b)*cos(c)]
#               ])

