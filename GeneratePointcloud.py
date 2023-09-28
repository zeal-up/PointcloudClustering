import numpy as np
import open3d as o3d
from copy import deepcopy

from scipy.spatial.transform import Rotation as R

def generate_cubic_pointcloud(point_num):
    """generate point cloud which randomly sampled from a cubic.
    The cubic's center is [0,0,0] and with 1 meter edge length.


    Args:
        point_num (int): the number of points in the generated point cloud

    Returns:
        np.ndarray: the generated point cloud. With shape (point_num, 3)
    """

    x_sampled = np.random.rand(point_num, 1) - 0.5
    y_sampled = np.random.rand(point_num, 1) - 0.5
    z_sampled = np.random.rand(point_num, 1) - 0.5
    point_cloud = np.concatenate([x_sampled, y_sampled, z_sampled], axis=1)

    return point_cloud

def generate_plane_pointcloud(point_num):
    """generate point cloud which randomly sampled from a plane.
    The plane's center is [0,0,0] and with 1 meter edge length.


    Args:
        point_num (int): the number of points in the generated point cloud

    Returns:
        np.ndarray: the generated point cloud. With shape (point_num, 3)
    """

    x_sampled = np.random.rand(point_num, 1) - 0.5
    y_sampled = np.random.rand(point_num, 1) - 0.5
    z_sampled = np.random.rand(point_num, 1) - 0.5
    z_sampled *= 0.05
    point_cloud = np.concatenate([x_sampled, y_sampled, z_sampled], axis=1)

    return point_cloud

def randomly_rotate(pointcloud):
    """randomly rotate the point cloud around z axis

    Args:
        pointcloud (np.ndarray): the point cloud to be rotated. With shape (point_num, 3)

    Returns:
        np.ndarray: the rotated point cloud. With shape (point_num, 3)
    """

    yaw = np.random.rand() * 2 * np.pi
    pitch = np.random.rand() * 2 * np.pi
    roll = np.random.rand() * 2 * np.pi
    
    r = R.from_euler('zyx', [yaw, pitch, roll])
    r_mat = r.as_matrix()
    pointcloud = (r_mat @ pointcloud.T).T    

    return pointcloud

if __name__ == '__main__':
    # point_cloud = generate_cubic_pointcloud(20000)
    point_cloud = generate_plane_pointcloud(20000)
    point_cloud = randomly_rotate(point_cloud)

    point_cloud += np.array([10,10,10])

    # visualize the point cloud with open3d
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(point_cloud)

    # deepcopy pcd to as pcd_trans
    pcd_trans = deepcopy(pcd)

    # translate pcd_trans with [1, 1, 1]
    pcd_trans.translate([1.2, 1.2, 1.2])

    # merge pcd and pcd_trans as one o3d point cloud
    pcd_merge_points = np.concatenate((pcd.points, pcd_trans.points), axis=0)
    pcd_merge = o3d.geometry.PointCloud()
    pcd_merge.points = o3d.utility.Vector3dVector(pcd_merge_points)
    o3d.visualization.draw_geometries([pcd_merge])

    # save pcd_merge to a .ply file
    o3d.io.write_point_cloud("pcd_merge.pcd", pcd_merge)
