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
[image4]: ./misc_images/kuka-dh.png
[image5]: ./misc_images/9blue.png
[image6]: ./misc_images/dh-frame.png
[image7]: ./misc_images/9blue-2.png
[image8]: ./misc_images/error.png

## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.   
You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

according lesson2-13 's algo for DH param assignmen we can get dh frame as below ![dh frmae][image6]

it's necessary to point out:​​  X_i   represents a  common normal between each   both ​Z_i ,Z_i-1. For parallel or coincident   axes, we need do some arbitrary choice


* alpha_i-1 (twist angle) = angle between Z_i-1 and Z_i​​  measured about X_i-1 in a right-hand sense.
* a_i-1 (link length) = distance from Z_i-1 to Z_i​​ measured along  X_i-1
* d_i  (link offset) = signed distance from X_i-1 to X_i measured along Z_i​​
​* theta_i or q_i(joint angle) = angle between X_i-1 to X_i measured along Z_i​​ in a right-hand sense. it's a variable for joint here


via above rules, check with the urdf(difference is  x4 x5 x6  origin are all put on the joint5/wrist-center),  got following DH table, q2 has a initial offset -pi/2​ , 

```python
s = {alpha0:       0,  a0:       0,  d1:       0.75,
     alpha1:   -pi/2,  a1:    0.35,  d2:          0,  q2:       q2-pi/2,
     alpha2:       0,  a2:    1.25,  d3:          0,  
     alpha3:   -pi/2,  a3:  -0.054,  d4:       0.96+0.54,
     alpha4:    pi/2,  a4:       0,  d5:          0,
     alpha5:   -pi/2,  a5:       0,  d6:          0,
     alpha6:       0,  a6:       0,  d7:      0.303,  q7:       0}
```

gripper frame in urdf has a gap with DH params, so R_corr is to correct this

using T_total =T0_G*R_corr for end effecor(in gripper) pose [FK](./FK.py)

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

#####  create a function for individual transformation matrix

```python
def individual_transformation_matrix( theta, alpha, a, d ):
    T = Matrix([[             cos(theta),             -sin(theta),            0,             a],
                   [ sin(theta)*cos(alpha),  cos(theta)*cos(alpha), -sin(alpha),-sin(alpha)*d],
                   [ sin(theta)*sin(alpha),  cos(theta)*sin(alpha),  cos(alpha), cos(alpha)*d],
                   [                   0,                    0,            0,               1]])
    return T
T0_1 =individual_transformation_matrix(q1,alpha0,a0,d1)
T1_2 =individual_transformation_matrix(q2,alpha1,a1,d2)

```
....

for the homogeneous transform  between base_link and gripper_link

T0_G=T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G

with the correction, end-effector pose could be got from T0_G * R_corr  (R_corr = R_z * R_y ), which could be used in FK to check the error after all theta available


##### numerical homogeneous transform R_rpy

with only end-effector(gripper) pose, we can get the numerical homogeneous transform

```python

(roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
R_rpy=tf.transformations.euler_matrix(yaw,pitch,roll,'rzyx')
```



#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.



##### go to the position

from above symbolic and numerical  homogeneous transform , we can : 

```python
T0_G*R_corr=T0_6*T6_G*R_corr = R_rpy

T0_6 = R_rpy *(T6_G*R_corr).inv()

#output from FK.py
    R_corr0=Matrix([[0, 0, 1,0],
                    [0,-1, 0,0],
                    [1, 0, 0,0],
                    [0, 0, 0,1]])

r0_6 = R_rpy[:3,:3]*R_corr0[:3,:3].inv()
#(r0_6 is the rotation part evaluation of T0_6, T6_G[:3,:3] actually is Identity matrix/I)

#I got wrist center according to the lesson
w=p-dg*r0_6*[0 0 1].T = p-dg*R_rpy[:3,:3]*R_corr0[:3,:3].inv()*[0 0 1].T 
w= p-dg*R_rpy[:3,:3]*[1 0 0].T

            wx=px-dg*R_rpy[0,0]
            wy=py-dg*R_rpy[1,0]
            wz=pz-dg*R_rpy[2,0]

#as the same in the lesson ik of scara
theta1=atan2(wy,wx)

```
        
according  ![the triangle of q2 q3 q5 in the picture][image4], 
using cosine law calculating beta1, beta2(in the triangle of 2,3,5 ,above picture), and then got theta2,theta3 (line from  3 to  5 is roughly as ax=1.501=sqrt(a3 * a3+d4 * d4),angle is about 0.036)

theta2=pi/2-beta2-atan2(wz-d1,r-a1)

theat3=pi/2-beta3--asin(-a3/d4)=pi/2-beta3-0.036

##### find the orientation

```python
            r0_3=T0_3.subs({q1:theta1,q2:theta2,q3:theta3})[:3,:3]
            r3_6=r0_3.evalf(subs={}).inv()*r0_6
            #r3_6=r0_3.T.evalf(subs={})*r0_6x

#so we get numerical r3_6
r3_6=[[r11 r12 r13]
      [r21 r22 r23]
      [r31 r32 r33]]

```


from FK code , I could print out symbolical r3_6
```python
T3_6 = simplify(T3_4*T4_5*T5_6)

('T3_6_symbol = ', Matrix([
[-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4), -0.054],
[                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5),    1.5],
[-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5),      0],
[                                         0,                                          0,                0,      1]]))


#compare to the symbolic representation T3_6[:3,:3] with above numerical r3_6 ,theat4-6 could be derived as:

r33/r13=-tan(q4)#theta4
r22/r21=-tan(q6)#theta6

q5= atan2(sqrt(r13**2 + r33**2), r23) #theta5
#when cos(q5)=0(i.e, r23=0), q5 should be pi/2(i.e theta5=pi/2)

    if sin(theta5) < 0:
        theta4 = atan2(-r33, r13)
        theta6 = atan2(r22, -r21)
    else:
        theta4 = atan2(r33, -r13)
        theta6 = atan2(-r22, r21)


```


in other extreme condition, when sin(q5)=0(i.e, r23= 1 or -1), joint 4,5,6 in one line , above T3_6_symbol could be simplized further

```python
# when cos(q5)=1,q5=0, q4+q6=atan2(-r12, r11), we can define one of to be 0
r3_6[:3,:3]
    =[[c(q4+q6) -s(q4+q6)  0]
      [0	 0 	   1]
      [-s(q4+q6) -c(q4+q6) 0]]


# when cos(q5)=-1, q5=pi, q6-q4=atan2(r12, -r11), let q4 be 0
r3_6[:3,:3]
    =[[-c(q6-q4) s(q6-q4)  0]
      [0	 0 	   1]
      [-s(q6-q4) -c(q6-q4) 0]]

```





### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

##### with above theory, tried on Ik_debug.pk, then IK_server.py is filled. 
* calculate_ik service is initialed in ik_server() function, 
* handle_calculate_IK() will on demand to handle the end-effector req.poses, previous theta1-6 Analysis  will be implemented here. joint_trajectory_list is retured
* all symbolic computing,such as transformation of all joints is done once in the main() for the performance consideration, with the helper function individual_transformation_matrix()
* add the error collection in handler and plot it 

Run in ros, I got 9/10 hit! see alot blue are stacked which can be evident of correct end-effector position
![result of picking][image7]

better than my preivous result
![result of preivous picking][image3]

#### next 
* error analysis, now is around 4.5e-5, see ![error curve][image8]
* I heard that numpy has speed advantage than sympy, need a try.