Rigid Body Kinematics for Unitree's Go1 quadruped robot. Including MATLAB code for the 
plotting and simulation of the DH frames, calculation of the Jacobian, and end-effector forces as well as C++ 
implementation for real-time integration with the robot.

# Unitree Go1


Denavit-Hartenberg (DH) Tables
* The parameters from the first hip joint (1) to the end-effector (E) are identical for each leg, so the full DH table is presented for the Front Right Leg while these three rows are omitted from the other tables for simplicity.

| Front Right Leg | Front Left Leg |
|--|--|
|<table> <tr><th> Link </th><th> d </th> <th> $\theta$ </th> <th> a </th> <th> $\alpha$ </th></tr><tr><td> A &rarr; B </td><td> 0 </td> <td> $\frac{\\pi}{2}$ </td> <td> $-\frac{b_w}{2}$ </td> </td> <td> $\frac{\\pi}{2}$ </td></tr> <tr><td> B &rarr; 0 </td><td> $\frac{b_l}{2}$ </td> <td> 0 </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 0 &rarr; 1 </td><td> $-l_0$ </td> <td> $\theta_1$ </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 1 &rarr; 2 </td><td> 0 </td> <td> $-\frac{\\pi}{2}$ </td> <td> 0 </td> </td> <td> $-\frac{\\pi}{2}$ </td></tr> <tr><td> 2 &rarr; 3 </td><td> 0 </td> <td> $\theta_2$ </td> <td> $l_1$ </td> </td> <td> 0 </td></tr> <tr><td> 3 &rarr; E </td><td> 0 </td> <td> $\theta_3$ </td> <td> $l_2$ </td> </td> <td> 0 </td></tr> </table>| <table> <tr><th> Link </th><th> d </th> <th> $\theta$ </th> <th> a </th> <th> $\alpha$ </th></tr><tr><td> A &rarr; B </td> <td> 0 </td> <td> $\frac{\\pi}{2}$ </td> <td> $\frac{b_w}{2}$ </td> </td> <td> $\frac{\\pi}{2}$ </td></tr></tr> <tr><td> B &rarr; 0 </td><td> $\frac{b_l}{2}$ </td> <td> 0 </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 0 &rarr; 1 </td><td> $l_0$ </td> <td> $\theta_1$ </td> <td> 0 </td> </td> <td> 0 </td></tr> </table> |


| Rear Right Leg | Rear Left Leg |
|--|--|
|<table> <tr><th> Link </th><th> d </th> <th> $\theta$ </th> <th> a </th> <th> $\alpha$ </th></tr><tr><td> A &rarr; B </td><td> 0 </td> <td> $\frac{\\pi}{2}$ </td> <td> $\frac{b_w}{2}$ </td> </td> <td> $\frac{\\pi}{2}$ </td></tr> <tr><td> B &rarr; 0 </td><td> $-\frac{b_l}{2}$ </td> <td> 0 </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 0 &rarr; 1 </td><td> $-l_0$ </td> <td> $\theta_1$ </td> <td> 0 </td> </td> <td> 0 </td></tr>  </table>| <table> <tr><th> Link </th><th> d </th> <th> $\theta$ </th> <th> a </th> <th> $\alpha$ </th></tr><tr><td> A &rarr; B </td> <td> 0 </td> <td> $\frac{\\pi}{2}$ </td> <td> $-\frac{b_w}{2}$ </td> </td> <td> $\frac{\\pi}{2}$ </td></tr></tr> <tr><td> B &rarr; 0 </td><td> $-\frac{b_l}{2}$ </td> <td> 0 </td> <td> 0 </td> </td> <td> 0 </td></tr> <tr><td> 0 &rarr; 1 </td><td> $l_0$ </td> <td> $\theta_1$ </td> <td> 0 </td> </td> <td> 0 </td></tr>   </table>|
