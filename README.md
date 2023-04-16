# Geometric Quadcopter Control
This is a multi-threaded quadcopter state estimation and control stack that utilizes the structure of underlying state Lie groups to maximize flight capabilities and dynamic performance.

The control design that is implemented is nearly globally-stable, unlike other controllers that are linearized around a hovering state. 

State estimation is achieved via an Extended Kalman Filter on the manifold SE(3).

Trajectories can be minimally specified in x, y, z, yaw due to the differential flatness of quadcopter dynamics. 

Dependencies: 
- Eigen3
- manif 
- Webots (optional for simulation)

Build:
- The library can be tested against the webots simulation by building the executable 'test'
- It is also possible to build just the library with the target 'quad-ctrl'