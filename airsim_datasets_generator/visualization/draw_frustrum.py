# from matplotlib import pyplot as plt
# from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
# import numpy as np

# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')

# # vertices of a pyramid
# v = np.array([[-1, -1, -1], [1, -1, -1], [1, 1, -1],  [-1, 1, -1], [0, 0, 1]])
# ax.scatter3D(v[:, 0], v[:, 1], v[:, 2])

# # generate list of sides' polygons of our pyramid
# verts = [ [v[0],v[1],v[4]], [v[0],v[3],v[4]],
#  [v[2],v[1],v[4]], [v[2],v[3],v[4]], [v[0],v[1],v[2],v[3]]]

# # plot sides
# ax.add_collection3d(Poly3DCollection(verts,
#  facecolors='cyan', linewidths=2, edgecolors='r', alpha=.25))

# plt.show()
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt


# https://github.com/IBM/opencv-power/blob/master/samples/python/camera_calibration_show_extrinsics.py

Axes3D(
        plt.figure('Sideways pyramid')
).plot(
    xs=[0, 0, 0, 0, 0, 1, 0, 0, 1, 0],
    ys=[0, 1, 1, 0, 0, 0.5, 1, 0, 0.5, 1],
    zs=[0, 0, 1, 1, 0, 0.5, 1, 1, 0.5, 0],
)

plt.show()