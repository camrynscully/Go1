# Unitree Go1
Rigid Body Kinematics for Unitree's Go1 quadruped robot. Including MATLAB code for the 
plotting and simulation of the DH frames, calculation of the Jacobian, and end-effector forces as well as C++ 
implementation for real-time integration with the robot.

## Contents
* `DH`
  * `dh_matrix.m` – computes the homogeneous transformation matrix provided the Denavit-Hartenberg (DH) parameters
  * `go1_DH.m` – main script, which plots the frames of each joint producing a kinematic model of the Go1
* `Data` : various text files containing motion capture data from OptiTrack, foot force data, as well as joint positions and torques throughout experiments
* `Force Plots` 
  * `plot_forces.m` – generates foot force plots
* `Motion Capture Plots`: contains the final baseline and new policy comparison figures across all five parameters
  * `plot_data.m` – generates plots of the position, velocity, acceleration, jerk, and orientation given the OptitTrack data for a specific experiment
* `end_effector_forces.cpp` – C++ code to calculate the end-effector forces in real time
* `go1_leg_simulation.m` – simulation of a single Go1 leg while walking
* `phase_plots.m` – produces phase plots of the Go1 provided joint positions while walking

## Kinematic Model
* The parameters from the first hip joint (1) to the end-effector (E) are identical for each leg, so the full DH table is presented for the Front Right Leg while these three rows are omitted from the other tables for simplicity.

| Front Right Leg | Front Left Leg |
|--|--|
|<table> <tr><th> Link </th><th> d </th> <th> $\theta$ </th> <th> a </th> <th> $\alpha$ </th></tr><tr><td> A &rarr; B </td><td> 0 </td> <td> $\frac{\\pi}{2}$ </td> <td> $-\frac{b_w}{2}$ </td> </td> <td> $\frac{\\pi}{2}$ </td></tr> <tr><td> B &rarr; 0 </td><td> $\frac{b_l}{2}$ </td> <td> 0 </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 0 &rarr; 1 </td><td> $-l_0$ </td> <td> $\theta_1$ </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 1 &rarr; 2 </td><td> 0 </td> <td> $-\frac{\\pi}{2}$ </td> <td> 0 </td> </td> <td> $-\frac{\\pi}{2}$ </td></tr> <tr><td> 2 &rarr; 3 </td><td> 0 </td> <td> $\theta_2$ </td> <td> $l_1$ </td> </td> <td> 0 </td></tr> <tr><td> 3 &rarr; E </td><td> 0 </td> <td> $\theta_3$ </td> <td> $l_2$ </td> </td> <td> 0 </td></tr> </table>| <table> <tr><th> Link </th><th> d </th> <th> $\theta$ </th> <th> a </th> <th> $\alpha$ </th></tr><tr><td> A &rarr; B </td> <td> 0 </td> <td> $\frac{\\pi}{2}$ </td> <td> $\frac{b_w}{2}$ </td> </td> <td> $\frac{\\pi}{2}$ </td></tr></tr> <tr><td> B &rarr; 0 </td><td> $\frac{b_l}{2}$ </td> <td> 0 </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 0 &rarr; 1 </td><td> $l_0$ </td> <td> $\theta_1$ </td> <td> 0 </td> </td> <td> 0 </td></tr> </table> |


| Rear Right Leg | Rear Left Leg |
|--|--|
|<table> <tr><th> Link </th><th> d </th> <th> $\theta$ </th> <th> a </th> <th> $\alpha$ </th></tr><tr><td> A &rarr; B </td><td> 0 </td> <td> $\frac{\\pi}{2}$ </td> <td> $\frac{b_w}{2}$ </td> </td> <td> $\frac{\\pi}{2}$ </td></tr> <tr><td> B &rarr; 0 </td><td> $-\frac{b_l}{2}$ </td> <td> 0 </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 0 &rarr; 1 </td><td> $-l_0$ </td> <td> $\theta_1$ </td> <td> 0 </td> </td> <td> 0 </td></tr>  </table>| <table> <tr><th> Link </th><th> d </th> <th> $\theta$ </th> <th> a </th> <th> $\alpha$ </th></tr><tr><td> A &rarr; B </td> <td> 0 </td> <td> $\frac{\\pi}{2}$ </td> <td> $-\frac{b_w}{2}$ </td> </td> <td> $\frac{\\pi}{2}$ </td></tr></tr> <tr><td> B &rarr; 0 </td><td> $-\frac{b_l}{2}$ </td> <td> 0 </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 0 &rarr; 1 </td><td> $l_0$ </td> <td> $\theta_1$ </td> <td> 0 </td> </td> <td> 0 </td></tr>   </table>|
