## Project: Kinematics Pick & Place 

---


**Steps to complete the project:**  


1. Set up  ROS Workspace f.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode] 
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/misc1.png
[image3]: ./misc_images/final.png
[image2]: ./misc_images/theta2.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.   
You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

using T_total =T0_G*R_corr for [FK.py](./FK.py)
```python
s = {alpha0:       0,  a0:       0,  d1:       0.75,
     alpha1:   -pi/2,  a1:    0.35,  d2:          0,  q2:       q2-pi/2,
     alpha2:       0,  a2:    1.25,  d3:          0,  
     alpha3:   -pi/2,  a3:  -0.054,  d4:       0.96+0.54,
     alpha4:    pi/2,  a4:       0,  d5:          0,
     alpha5:   -pi/2,  a5:       0,  d6:          0,
     alpha6:       0,  a6:       0,  d7:      0.303,  q7:       0}
```

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

create a function for individual transformation matrix

```python
def individual_transformation_matrix( theta, alpha, a, d ):
    T = Matrix([[             cos(theta),             -sin(theta),            0,             a],
                   [ sin(theta)*cos(alpha),  cos(theta)*cos(alpha), -sin(alpha),-sin(alpha)*d],
                   [ sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha), cos(alpha)*d],
                   [                   0,                    0,            0,               1]])
    return T
T0_1 =individual_transformation_matrix(q1,alpha0,a0,d1)
```
....

for the homogeneous transform R_rpy, do with follow

```python

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
R_rpy=tf.transformations.euler_matrix(yaw,pitch,roll)
```



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

```python
T0_G*R_corr=T0_6*T6_G*R_corr = R_rpy

r0_6 = R_rpy[:3,:3]*R_corr0[:3,:3].inv()*T6_G[:3,:3].inv()
#(r0_6 is the rotation part evaluation of T0_6, T6_G[:3,:3] actually is I)

#I got wrist center according to the lesson
            wx=px-dg*R_rpy[0,0]
            wy=py-dg*R_rpy[1,0]
            wz=pz-dg*R_rpy[2,0]
theta1=atan2(wy,wx)
```
        
according  ![the triangle of q2 q3 q5 in the picture][image2], using cosine law calculating theta2,theta3

line from q3 to q5 is roughly as ax=1.501

```python
            r0_3=T0_3.subs({q1:theta1,q2:theta2,q3:theta3})[:3,:3]
            r3_6=r0_3.evalf(subs={}).inv()*r0_6
```
theat4-6 could be derived from r3_6

from FK code , I could print out
```python
('T3_6_symbol = ', Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.054],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5),    1.5],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5),      0],
[                                         0,                                          0,                0,      1]]))


r33/r13=-tan(theta4)
r22/r21=-tan(theta6)
theta5= atan2(sqrt(r13**2 + r33**2), r23)


```



### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

with above flow, IK_server.py is filled. theta2 has a little error, should be corrected by substracted with a small angle.

And just for fun, another example image:
![result of picking][image3]



