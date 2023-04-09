import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt("data.txt", delimiter=" ", usecols=(1,2,3,5,6,7))

print(data[0])

#//fig = plt.figure();
fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')

#ax2 = fig.add_subplot(111, projection = '3d')
ax.plot(data[:, 3], data[:, 4], data[:, 5], marker = 'x', color='red')

ax.plot(data[:, 0], data[:, 1], data[:, 2], marker = 'o', color='blue')
plt.show()