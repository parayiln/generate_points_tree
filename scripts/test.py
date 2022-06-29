import numpy as np
import open3d as o3d
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
import mplcursors

def onclick(event):
    print(event.xdata, event.ydata, event.zdata)

def main():
    cloud = o3d.io.read_point_cloud("tree.ply") # Read the point cloud
    print(cloud)
    # o3d.visualization.draw_geometries([cloud])
    tree_points=np.asarray(cloud.points)
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    # print(tree_points)
    for i in range(len(tree_points)):
        ax.scatter(tree_points[i][0],tree_points[i][1],tree_points[i][2],marker=1)
    plt.show()

if __name__ == "__main__":
    main()
