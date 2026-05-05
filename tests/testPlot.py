import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d


ax = plt.axes(projection="3d")

p1 = np.array([0, 0, 0])
p2 = np.array([3, 5, 2])

vec1x = np.linspace(p1[0], p2[0], 100)
vec1y = np.linspace(p1[1], p2[1], 100)
vec1z = np.linspace(p1[2], p2[2], 100)

ax.plot(vec1x, vec1y, vec1z)
ax.set_title("test vec")

plt.show()