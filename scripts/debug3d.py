import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

def draw_pose(ax, pos, quat):
   r = np.array(R.from_quat(quat).as_matrix())
   x_ax = np.array([pos, pos + r[:, 0]])
   y_ax = np.array([pos, pos + r[:, 1]])
   z_ax = np.array([pos, pos + r[:, 2]])
   ax.plot(x_ax[:, 0], x_ax[:, 1], x_ax[:, 2], 'r')
   ax.plot(y_ax[:, 0], y_ax[:, 1], y_ax[:, 2], 'g')
   ax.plot(z_ax[:, 0], z_ax[:, 1], z_ax[:, 2], 'b')

def draw_axis(ax, pos, v):
    x_ax = np.array([pos, pos + v])
    ax.plot(x_ax[:, 0], x_ax[:, 1], x_ax[:, 2], 'k')

ax = plt.figure().add_subplot(projection='3d')

draw_pose(ax, np.array([0, 0, 0]), [0, 0, 0, 1])
#draw_pose(ax, np.array([10, 10, 10]), [0, 0, 1, 0])

#draw_axis(ax, np.array([0, 0, 0]), [0.5, 0.5, 0.5])
draw_axis(ax, np.array([0, 0, 0]), [-0.323, 0.0601, 0.837])
plt.show()