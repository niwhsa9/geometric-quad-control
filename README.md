# Geometric Quadcopter Control
Quadcopter state estimation and control stack that utilizes the structure of underlying state Lie groups to maximize flight capabilities.

State estimation is achieved via an Extended Kalman Filter on the manifold SE(3). Trajectories can be minimally specified in x, y, z, yaw due to the differential flatness of the dynamics. State estimation and controller are run in seperate threads.

Dependencies: 
- Eigen3
- manif 
- Webots (optional for simulation)