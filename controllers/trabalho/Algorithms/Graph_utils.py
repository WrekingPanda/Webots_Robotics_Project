import csv
import math
import time
from typing import Union, Dict

from scipy.spatial import KDTree

import numpy as np


from controllers.trabalho.Algorithms.metric_graph import VertexInfo, MetricGraph
from controllers.trabalho.utils import move_robot_to, is_collision_free_line, is_collision_free_point, warp_robot








def find_closest_accessible_vertex(x: float, y: float, vertices_info: [VertexInfo], obstacle_cloud: np.ndarray) -> Union[VertexInfo, None]:
    closest_accessible_vertex: Union[VertexInfo, None] = None
    closest_distance: float = math.inf
    for vertex_info in vertices_info:
        distance: float = math.hypot(x - vertex_info.x, y - vertex_info.y)
        if distance >= closest_distance:
            continue
        if is_collision_free_line(x, y, vertex_info.x, vertex_info.y, obstacle_cloud):
            closest_accessible_vertex = vertex_info
            closest_distance = distance
    return closest_accessible_vertex


def create_grid_graph(initial_pos: (float, float), final_pos: (float, float), obstacle_cloud: np.ndarray) -> MetricGraph:
    grid_graph = MetricGraph()

    # Add the grid vertices
    n_xy_divisions: int = 20
    # We assume that the x and y coordinates in any point of the map are positive
    max_x: float = 2
    max_y: float = 2

    min_x_offset: float = max_x / n_xy_divisions / 2.0
    x_increment: float = max_x / n_xy_divisions
    min_y_offset: float = max_y / n_xy_divisions / 2.0
    y_increment: float = max_y / n_xy_divisions
    cur_index: int = 0
    for i in range(n_xy_divisions):
        x: float = min_x_offset + i * x_increment
        for j in range(n_xy_divisions):
            y: float = min_y_offset + j * y_increment
            # Add a vertex to point (x,y)
            grid_graph.add_vertex(cur_index, (x, y), 'blue')
            cur_index += 1

    # Add the initial and final vertices
    additional_points: [(float, float)] = [initial_pos, final_pos]
    for point in additional_points:
        grid_graph.add_vertex(cur_index, point, 'green')
        cur_index += 1

    # Connect the initial point to the closest point in the grid using edges
    closest_vertex_to_initial: Union[VertexInfo, None] = find_closest_accessible_vertex(initial_pos[0], initial_pos[1], grid_graph.vertices_info[:-2], obstacle_cloud)
    if closest_vertex_to_initial is None:
        print("Initial position ", initial_pos, " is not accessible by any point in the grid.")
        return grid_graph
    grid_graph.add_edge(len(grid_graph.vertices_info) - 2, closest_vertex_to_initial.id,
                        math.hypot(initial_pos[0] - closest_vertex_to_initial.x, initial_pos[1] - closest_vertex_to_initial.y))
    grid_graph.add_edge(closest_vertex_to_initial.id, len(grid_graph.vertices_info) - 2,
                        math.hypot(initial_pos[0] - closest_vertex_to_initial.x,
                                   initial_pos[1] - closest_vertex_to_initial.y))

    # Connect the final point to the closest point in the grid using edges
    closest_vertex_to_final: Union[VertexInfo, None] = find_closest_accessible_vertex(final_pos[0], final_pos[1], grid_graph.vertices_info[:-2], obstacle_cloud)
    if closest_vertex_to_final is None:
        print("Final position ", final_pos, " is not accessible by any point in the grid.")
        return grid_graph
    grid_graph.add_edge(closest_vertex_to_final.id, len(grid_graph.vertices_info) - 1,
                        math.hypot(final_pos[0] - closest_vertex_to_final.x,
                                   final_pos[1] - closest_vertex_to_final.y))
    grid_graph.add_edge(len(grid_graph.vertices_info) - 1, closest_vertex_to_final.id,
                        math.hypot(final_pos[0] - closest_vertex_to_final.x,
                                   final_pos[1] - closest_vertex_to_final.y))

    # Add the grid edges
    cur_index = 0
    for i in range(n_xy_divisions):
        for j in range(n_xy_divisions):
            # Add an edge with the left neighbor
            if i > 0 and is_collision_free_line(grid_graph.vertices_info[cur_index].x, grid_graph.vertices_info[cur_index].y, grid_graph.vertices_info[cur_index - n_xy_divisions].x, grid_graph.vertices_info[cur_index - n_xy_divisions].y, obstacle_cloud):
                grid_graph.add_edge(cur_index, cur_index - n_xy_divisions, x_increment)
                grid_graph.add_edge(cur_index - n_xy_divisions, cur_index, x_increment)
            # Add an edge with the top neighbor
            if j > 0 and is_collision_free_line(grid_graph.vertices_info[cur_index].x, grid_graph.vertices_info[cur_index].y, grid_graph.vertices_info[cur_index - 1].x, grid_graph.vertices_info[cur_index - 1].y, obstacle_cloud):
                grid_graph.add_edge(cur_index, cur_index - 1, y_increment)
                grid_graph.add_edge(cur_index - 1, cur_index, y_increment)
            cur_index += 1

    return grid_graph


def create_prm_graph(initial_pos: (float, float), final_pos: (float, float), obstacle_cloud: np.ndarray) -> MetricGraph:
    prm_graph = MetricGraph()
    n_samples = 500
    neighbors = 7
    max_x: float = 1.9
    max_y: float = 1.9
    points = []
    count = 0
    while count < n_samples:
        random_position: (float, float) = (np.random.uniform(0, max_x), np.random.uniform(0, max_y))
        if is_collision_free_point(random_position[0], random_position[1], obstacle_cloud):
            points.append(np.array([random_position[0], random_position[1]]))
            prm_graph.add_vertex(count, random_position, 'blue')
            count += 1
    additional_points: [(float, float)] = [initial_pos, final_pos]
    for point in additional_points:
        points.append(np.array([point[0], point[1]]))
        prm_graph.add_vertex(count, point, 'green')
        count += 1
    points_kd_tree = KDTree(points)
    for index1, point1 in enumerate(points):
        distances, indices = points_kd_tree.query(point1, k=neighbors + 1)
        for distance, index2 in zip(distances[1:], indices[1:]):
            point2 = points[index2]
            if is_collision_free_line(point1[0], point1[1],
                                          point2[0], point2[1], obstacle_cloud):
                prm_graph.add_edge(index1, index2, distance)
                prm_graph.add_edge(index2, index1, distance)
    return prm_graph