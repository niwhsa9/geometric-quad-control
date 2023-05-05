import numpy as np
import matplotlib.pyplot as plt
import fileinput

data_lst = []

fig = plt.figure()
ax = fig.add_subplot(111, projection = '3d')
cnt = 0 
for line in fileinput.input():
    cnt+=1
    print(line)
    if(cnt % 25 != 0):
        continue
    tokens = line.split(" ")
    data_lst.append(np.array([ float(tokens[1]), float(tokens[2]), float(tokens[3]),
                           float(tokens[5]), float(tokens[6]), float(tokens[7]) 
                          ]))
    data = np.array(data_lst)

    #ax.plot(data[:, 3], data[:, 4], data[:, 5], marker = 'x', color='red')
    ax.plot(data[:, 0], data[:, 1], data[:, 2], marker = 'o', color='blue')
    plt.pause(0.0001)

plt.show()