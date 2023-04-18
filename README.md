# Geometric Quadcopter Control
This is a multi-threaded quadcopter state estimation and control stack that accounts for the structure of underlying state Lie groups to maximize flight capabilities and dynamic performance. The concurrent execution mode can be employed to make this feasible to run on embedded processors.

The control design that is implemented exhibits asymptoic stablity for a large variety of target poses, unlike other controllers that are linearized around a hovering state. State estimation is achieved via an Extended Kalman Filter on the manifold SE(3). Trajectories can be minimally specified in x, y, z, yaw due to the differential flatness of quadcopter dynamics. 

### Dependencies: 

This project requires a compiler that supports C++20. The following libraries are used: 
- Eigen3
- manif 
- Webots (optional for simulation)

### Build:
- The library has a demo that can be tested against the webots simulation by building the executable 'test'
- It is also possible to build just the static library with the target 'quad-ctrl'

### Credits:
- The tracking controller on SE(3) is derrived in this work: https://mathweb.ucsd.edu/~mleok/pdf/LeLeMc2010_quadrotor.pdf
- As far as I can tell the method of EKF on manifolds is rigirously developed originally in this paper: https://arxiv.org/pdf/2102.03804.pdf. However, my implementation is not based off of this work directly.
- The paper "A micro Lie theory for state estimation in robotics" was tremendously helpful throughout the implementation of this project: https://arxiv.org/abs/1812.01537
- I found the MIT Visual Navigation course was generally helpful: https://vnav.mit.edu/. 