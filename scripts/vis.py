import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

POS_ONLY=False

def draw_pose(ax, pos, quat, scale=0.1):
   r = np.array(R.from_quat(quat).as_matrix())
   x_ax = np.array([pos, pos + r[:, 0] * scale])
   y_ax = np.array([pos, pos + r[:, 1] * scale])
   z_ax = np.array([pos, pos + r[:, 2] * scale])
   ax.plot(x_ax[:, 0], x_ax[:, 1], x_ax[:, 2], 'r')
   ax.plot(y_ax[:, 0], y_ax[:, 1], y_ax[:, 2], 'g')
   ax.plot(z_ax[:, 0], z_ax[:, 1], z_ax[:, 2], 'b')

def draw_axis(ax, pos, v):
    x_ax = np.array([pos, pos + v])
    ax.plot(x_ax[:, 0], x_ax[:, 1], x_ax[:, 2], 'k')

if POS_ONLY:
    data = np.loadtxt("data.txt", delimiter=" ", usecols=(1,2,3,5,6,7))
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    ax.plot(data[:, 3], data[:, 4], data[:, 5], marker = 'x', color='red')
    ax.plot(data[:, 0], data[:, 1], data[:, 2], marker = 'o', color='blue')
else:
    data = np.loadtxt("data.txt", delimiter=" ", usecols=(1,2,3,4,5,6,7))
    fig = plt.figure()
    ax = fig.add_subplot(111, projection = '3d')
    for row in data[::5]:
        draw_pose(ax, row[4:], row[:4])

plt.show()