from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
import numpy as np


with open("../build/output.txt", 'r') as fid:
    fdata = fid.readlines()

pp = []
pp2 = []
for linedata in fdata:
    if (linedata == '#\n'):        
        if (pp == []) :
            pass
        else :
            pp2.append(pp)

        pp = []
    else :
        p = linedata[:-1].split(',')
        point = [float(x) for x in p]
        pp.append(point)

fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')    
for pp in pp2:
    ps = np.array(pp)
    ax.plot(ps[:,0], ps[:,1], ps[:,2])
    ax.scatter(ps[:,0], ps[:,1], ps[:,2], s=10, marker='s')

plt.show() 
