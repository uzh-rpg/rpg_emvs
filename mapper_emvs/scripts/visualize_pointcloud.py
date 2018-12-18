# -*- coding: utf-8 -*-
"""
Load a PCD point cloud and visualize as a 3D scatter plot.

To run, install [pypcd](https://github.com/dimatura/pypcd) as follows:

    pip install pypcd

"""
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 unused import
import matplotlib.pyplot as plt
import numpy as np
import argparse
import pypcd


def set_aspect_ratio_equal(Xr, Yr, Zr, ax):
    """
    Set the apect ratio to 'equal'
    Code borrowed from: https://stackoverflow.com/a/21765085
    """
    max_range = np.array([Xr.max() - Xr.min(),
                          Yr.max() - Yr.min(),
                          Zr.max() - Zr.min()]).max() / 2.0
    mid_x = (Xr.max() + Xr.min()) * 0.5
    mid_y = (Yr.max() + Yr.min()) * 0.5
    mid_z = (Zr.max() + Zr.min()) * 0.5
    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)


if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Plot a reconstructed point cloud in 3D')
    parser.add_argument('-i', '--input', default='pointcloud.pcd', type=str,
                        help='path to the PCD file (default: pointcloud.pcd)')
    args = parser.parse_args()

    
    pc = pypcd.PointCloud.from_path(args.input)
    X, Y, Z, I = pc.pc_data['x'], pc.pc_data['y'], pc.pc_data['z'], pc.pc_data['intensity']
    
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_aspect('equal')
    
    # Rename the axis so that the Zr axis coincides with -X
    Xr, Yr, Zr = Z, -X, -Y
    
    # Plot point cloud
    ax.scatter(Xr, Yr, Zr, c=I, linewidths=0)
    ax.view_init(elev=30, azim=-150)
    ax.set_xlabel('Z')
    ax.set_ylabel('-X')
    ax.set_zlabel('-Y')
    set_aspect_ratio_equal(Xr, Yr, Zr, ax)
    
    plt.show()