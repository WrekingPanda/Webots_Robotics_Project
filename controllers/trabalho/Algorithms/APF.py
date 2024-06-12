import math

import numpy as np
from matplotlib import pyplot as plt, patches
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

from controller import Lidar, LidarPoint, GPS, Supervisor
from controllers.trabalho.utils import warp_robot



def compute_attractive_force(robot_position: np.ndarray, final_position: np.ndarray) -> np.ndarray:
    dist: float = np.linalg.norm(final_position - robot_position)
    force: np.ndarray = (final_position - robot_position) / dist
    return force

def compute_repulsive_force(lidar_cloud: [LidarPoint]) -> np.ndarray:
    d_max: float = 0.2
    window_size: int = 5  # must be odd

    # Find the local minima
    local_minima: [LidarPoint] = []
    # Add copies of the first readings to the end of the list to make comparisons more straightforward
    n_readings: int = len(lidar_cloud)
    for i in range(window_size // 2):
        lidar_cloud.append(lidar_cloud[i])

    for i in range(window_size // 2, n_readings + (window_size // 2)):
        # Determine if the reading is below the max repulsion distance. If not, skip the iteration.
        point: LidarPoint = lidar_cloud[i]
        dist: float = math.hypot(point.x, point.y)
        if dist > d_max:
            continue

        # Test if the reading is a local minimum
        neighbor_distances: [float] = [math.hypot(point.x, point.y) for point in
                                       lidar_cloud[i - (window_size // 2): i + (window_size // 2)]]
        if dist > min(neighbor_distances):
            continue

        local_minima.append(point)



    # Compute the resulting force
    force: np.ndarray = np.array([0.0, 0.0])
    for point in local_minima:
        dist: float = math.hypot(point.x, point.y)
        if dist > d_max:
            continue
        force -= ((1/dist) - (1/d_max)) * np.array([point.x, point.y]) / dist

    return force


def compute_resulting_force(robot_position: np.ndarray, final_position: np.ndarray, lidar_cloud: [LidarPoint]) -> np.ndarray:
    alpha: float = 1.0
    beta: float = 0.25  # 0.025

    af: np.ndarray = compute_attractive_force(np.array(robot_position), np.array(final_position))

    rf: np.ndarray = compute_repulsive_force(lidar_cloud)

    return alpha*af + beta*rf

def draw_quiver_plots(supervisor: Supervisor, final_position: (float, float), obstacle_cloud: np.ndarray, fast=False):
    lidar: Lidar = supervisor.getDevice('lidar')
    gps: GPS = supervisor.getDevice('gps')

    n_xy_divisions: int = 42
    # We assume that the x and y coordinates in any point of the map are positive
    max_x: float = 2.0
    max_y: float = 2.0

    min_x: float = max_x / n_xy_divisions / 2.0
    x_increment: float = max_x / n_xy_divisions
    min_y: float = max_y / n_xy_divisions / 2.0
    y_increment: float = max_y / n_xy_divisions

    x_values = np.arange(min_x, max_x, x_increment)
    y_values = np.arange(min_y, max_y, y_increment)

    X, Y = np.meshgrid(x_values, y_values)
    afx = np.zeros_like(X)
    afy = np.zeros_like(Y)
    rfx = np.zeros_like(X)
    rfy = np.zeros_like(Y)
    fx = np.zeros_like(X)
    fy = np.zeros_like(Y)
    for i in range(len(x_values)):
        for j in range(len(y_values)):
            x: float = X[i][j]
            y: float = Y[i][j]
            warp_robot(supervisor, "EPUCK", (x, y))
            supervisor.step()
            gps_readings = gps.getValues()
            robot_position = (gps_readings[0], gps_readings[1])

            attractive_force: np.ndarray = compute_attractive_force(np.array(robot_position), np.array(final_position))
            afx[i][j] = attractive_force[0]
            afy[i][j] = -attractive_force[1]

            repulsive_force: np.ndarray = compute_repulsive_force(lidar.getPointCloud())
            rfx[i][j] = repulsive_force[0]
            rfy[i][j] = -repulsive_force[1]

            resulting_force: np.ndarray = compute_resulting_force(np.array(robot_position), np.array(final_position), lidar.getPointCloud())
            fx[i][j] = resulting_force[0]
            fy[i][j] = -resulting_force[1]

    if not fast:
        # Create the PolyCollection for the obstacles, for the plots
        pat: [patches.Rectangle] = []
        for point in obstacle_cloud:
            pat.append(Rectangle((point[0], point[1]), 0.001, 0.001,
                                 linewidth=1, edgecolor='black', facecolor='none'))

        # Draw the quiver plots
        fig, ax = plt.subplots()
        ax.quiver(X, Y, afx, afy)
        col: PatchCollection = PatchCollection(pat)
        col.set_edgecolor('black')
        col.set_linewidth(1)
        ax.add_collection(col)
        ax.set_title('Attractive forces')
        plt.show()

        fig, ax = plt.subplots()
        ax.quiver(X, Y, rfx, rfy)
        col: PatchCollection = PatchCollection(pat)
        col.set_edgecolor('black')
        col.set_linewidth(1)
        ax.add_collection(col)
        ax.set_title('Repulsive forces')
        plt.show()

        fig, ax = plt.subplots()
        ax.quiver(X, Y, fx, fy)
        col: PatchCollection = PatchCollection(pat)
        col.set_edgecolor('black')
        col.set_linewidth(1)
        ax.add_collection(col)
        ax.set_title('Resulting force field')
        plt.show()