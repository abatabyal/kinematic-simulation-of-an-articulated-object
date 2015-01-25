# kinematic-simulation-of-an-articulated-object

The objective of this project is to implement a kinematic simulation of an articulated object. The program generates
the joint angles required to move a planar n-link robot so that the robot’s hand follows a specified trajectory. The robot does not exceed its physical limitations in terms of its maximum allowable joint velocity norm. The program takes input two  files. The first file, which is called “arm”, contains the specifications for the robot
arm. The format for this file is as follows:
n ∥Δ∥
l1 1(0)
l2 2(0)
...
...
ln n(0)
where n is an integer that denotes the number of joints in the robot, ∥Δ∥ is a floating
point number that is the maximum change in configuration between frames, li is the ith
link length, and (0) is the starting joint configuration.
The second file, which is called “trajectory”, contains the desired end-effector trajectory
for the robot. The format for this file is as follows:
m
x(0) y(0)
x(1) y(1)
...
...
x(m) y(m)
where m is the number of desired positions specified for the robot and [x(i); y(i)] is the
desired position of the end-effector at the ith frame.
The output of the program should be a file called “angles” whose format is:
1(0) 2(0) : : : n(0)
1(1) 2(1) : : : n(1)
...
...
...
...
1(m) 2(m) : : : n(m)
