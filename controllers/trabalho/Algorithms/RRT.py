import math
import random

import numpy as np

from controllers.trabalho.Algorithms.metric_graph import MetricGraph, VertexInfo
from controllers.trabalho.utils import is_collision_free_point, is_collision_free_line








def create_rrt(initial_position: (float, float), final_position: (float, float), obstacle_cloud: np.ndarray) -> (bool, MetricGraph):
    max_iterations: int = 1000
    incremental_distance: float = 0.25
    # We assume that the x and y coordinates in any point of the map are positive
    max_x: float = 2.0
    max_y: float = 2.0

    rrt_graph = MetricGraph()

    # Add the initial position vertex
    cur_index: int = 0
    rrt_graph.add_vertex(cur_index, (initial_position[0], initial_position[1]), 'blue')
    cur_index += 1

    # Check if the final vertex is directly accessible.
    # If so, add a vertex for the final vertex, create an edge to it and stop the algorithm.
    last_vertex_dist_to_final: float = math.hypot(rrt_graph.vertices_info[-1].x - final_position[0],
                                                  rrt_graph.vertices_info[-1].y - final_position[1])
    if is_collision_free_line(rrt_graph.vertices_info[-1].x, rrt_graph.vertices_info[-1].y,
                              final_position[0], final_position[1],
                              obstacle_cloud):
        # Add the final position vertex
        rrt_graph.add_vertex(cur_index, (final_position[0], final_position[1]), 'blue')
        cur_index += 1

        rrt_graph.add_edge(len(rrt_graph.vertices_info) - 2, len(rrt_graph.vertices_info) - 1, last_vertex_dist_to_final, True)
        return True, rrt_graph

    for k in range(max_iterations):
        while True:
            # Generate a random free position and check that it doesn't collide with the obstacles.
            # If it does, generate a new point.
            random_position: (float, float) = (random.uniform(0, max_x), random.uniform(0, max_y))
            if not is_collision_free_point(random_position[0], random_position[1], obstacle_cloud):
                continue

            # Find the nearest vertex to it in the graph.
            closest_accessible_vertex: VertexInfo = rrt_graph.vertices_info[0]
            closest_distance: float = math.hypot(random_position[0] - rrt_graph.vertices_info[0].x,
                                                 random_position[1] - rrt_graph.vertices_info[0].y)
            for vertex_info in rrt_graph.vertices_info:
                distance: float = math.hypot(random_position[0] - vertex_info.x,
                                             random_position[1] - vertex_info.y)
                if distance < closest_distance:
                    closest_accessible_vertex = vertex_info
                    closest_distance = distance

            # Define a new position that is in the direction of the random free position
            new_distance: float = min(incremental_distance, closest_distance)
            new_position: (float, float) = (
                closest_accessible_vertex.x + new_distance * (random_position[0] - closest_accessible_vertex.x)/closest_distance,
                closest_accessible_vertex.y + new_distance * (random_position[1] - closest_accessible_vertex.y)/closest_distance,
            )

            # Check if the position is accessible from the nearest vertex, if not, skip to the next iteration
            if not is_collision_free_line(closest_accessible_vertex.x, closest_accessible_vertex.y,
                                          new_position[0], new_position[1], obstacle_cloud):
                continue

            # Add the new vertex to the graph
            rrt_graph.add_vertex(cur_index, (new_position[0], new_position[1]), 'blue')
            cur_index += 1

            # Add an edge between the new vertex and the closest vertex to the graph
            rrt_graph.add_edge(closest_accessible_vertex.id, len(rrt_graph.vertices_info) - 1, new_distance, True)

            # Check if the last added vertex is directly accessible.
            # If so, add a vertex for the final vertex, create an edge to it and stop the algorithm.
            if is_collision_free_line(rrt_graph.vertices_info[-1].x, rrt_graph.vertices_info[-1].y, final_position[0], final_position[1],
                                      obstacle_cloud):
                # Add the final position vertex
                rrt_graph.add_vertex(cur_index, (final_position[0], final_position[1]), 'blue')
                cur_index += 1

                last_vertex_dist_to_final: float = math.hypot(rrt_graph.vertices_info[-1].x - final_position[0],
                                                              rrt_graph.vertices_info[-1].y - final_position[1])
                rrt_graph.add_edge(len(rrt_graph.vertices_info) - 2, len(rrt_graph.vertices_info) - 1, last_vertex_dist_to_final, True)
                return True, rrt_graph
            break

    return False, rrt_graph

def create_rrt_star(initial_position: (float, float), final_position: (float, float), obstacle_cloud: np.ndarray) -> (bool, MetricGraph):
    max_iterations: int = 500
    incremental_distance: float = 0.25
    # We assume that the x and y coordinates in any point of the map are positive
    max_x: float = 2.0
    max_y: float = 2.0

    rrt_graph = MetricGraph()

    # Add the initial position vertex
    cur_index: int = 0
    rrt_graph.add_vertex(cur_index, (initial_position[0], initial_position[1]), 'blue', 0)
    cur_index += 1

    # Check if the final vertex is directly accessible.
    # If so, add a vertex for the final vertex, create an edge to it and stop the algorithm.
    last_vertex_dist_to_final: float = math.hypot(rrt_graph.vertices_info[-1].x - final_position[0],
                                                  rrt_graph.vertices_info[-1].y - final_position[1])
    if is_collision_free_line(rrt_graph.vertices_info[-1].x, rrt_graph.vertices_info[-1].y,
                              final_position[0], final_position[1],
                              obstacle_cloud):
        # Add the final position vertex
        rrt_graph.add_vertex(cur_index, (final_position[0], final_position[1]), 'blue',math.hypot(rrt_graph.vertices_info[-1].x - final_position[0],
                                                  rrt_graph.vertices_info[-1].y - final_position[1]))
        cur_index += 1

        rrt_graph.add_edge(len(rrt_graph.vertices_info) - 2, len(rrt_graph.vertices_info) - 1, last_vertex_dist_to_final, True)
        return True, rrt_graph

    for k in range(max_iterations):
        while True:
            # Generate a random free position and check that it doesn't collide with the obstacles.
            # If it does, generate a new point.
            random_position: (float, float) = (random.uniform(0, max_x), random.uniform(0, max_y))
            if not is_collision_free_point(random_position[0], random_position[1], obstacle_cloud):
                continue

            # Find the nearest vertex to it in the graph.
            closest_accessible_vertex: VertexInfo = rrt_graph.vertices_info[0]
            closest_distance: float = math.hypot(random_position[0] - rrt_graph.vertices_info[0].x,
                                                 random_position[1] - rrt_graph.vertices_info[0].y)
            for vertex_info in rrt_graph.vertices_info:
                distance: float = math.hypot(random_position[0] - vertex_info.x,
                                             random_position[1] - vertex_info.y)
                if distance < closest_distance:
                    closest_accessible_vertex = vertex_info
                    closest_distance = distance

            # Define a new position that is in the direction of the random free position
            new_distance: float = min(incremental_distance, closest_distance)
            new_position: (float, float) = (
                closest_accessible_vertex.x + new_distance * (random_position[0] - closest_accessible_vertex.x)/closest_distance,
                closest_accessible_vertex.y + new_distance * (random_position[1] - closest_accessible_vertex.y)/closest_distance,
            )

            # Check if the position is accessible from the nearest vertex, if not, skip to the next iteration
            if not is_collision_free_line(closest_accessible_vertex.x, closest_accessible_vertex.y,
                                          new_position[0], new_position[1], obstacle_cloud):
                continue

            # Add the new vertex to the graph
            rrt_graph.add_vertex(cur_index, (new_position[0], new_position[1]), 'blue',new_distance)
            cur_index += 1

            # Find the vertex with least cost and find the close vertices
            close = []
            least_cost_vertex: VertexInfo = rrt_graph.vertices_info[closest_accessible_vertex.id]
            least_cost: float = rrt_graph.vertices_info[closest_accessible_vertex.id].cost + math.hypot(new_position[0] - closest_accessible_vertex.x,
                                                 new_position[1] - closest_accessible_vertex.y)
            for i in range(len(rrt_graph.vertices_info) - 1):
                vertex_info = rrt_graph.vertices_info[i]
                dist = math.hypot(new_position[0] - vertex_info.x,
                                                 new_position[1] - vertex_info.y)
                if dist <= 2*incremental_distance:
                    cost = vertex_info.cost + dist
                    close.append(vertex_info)
                    if cost < least_cost and is_collision_free_line(vertex_info.x, vertex_info.y,
                                          new_position[0], new_position[1], obstacle_cloud):
                        least_cost_vertex = vertex_info
                        least_cost = cost

            new_distance: float = math.hypot(new_position[0] - least_cost_vertex.x,
                                                 new_position[1] - least_cost_vertex.y)

            # Add an edge between the new vertex and the closest vertex to the graph
            rrt_graph.add_edge(least_cost_vertex.id, len(rrt_graph.vertices_info) - 1, new_distance, True)
            rrt_graph.vertices_info[-1].cost = least_cost_vertex.cost + new_distance

            # Check if the last added vertex is directly accessible.
            # If so, add a vertex for the final vertex, create an edge to it and stop the algorithm.
            if is_collision_free_line(rrt_graph.vertices_info[-1].x, rrt_graph.vertices_info[-1].y, final_position[0], final_position[1],
                                      obstacle_cloud):
                # Add the final position vertex
                rrt_graph.add_vertex(cur_index, (final_position[0], final_position[1]), 'blue',math.hypot(rrt_graph.vertices_info[-1].x - final_position[0],
                                                  rrt_graph.vertices_info[-1].y - final_position[1]))
                cur_index += 1

                last_vertex_dist_to_final: float = math.hypot(rrt_graph.vertices_info[-1].x - final_position[0],
                                                              rrt_graph.vertices_info[-1].y - final_position[1])
                rrt_graph.add_edge(len(rrt_graph.vertices_info) - 2, len(rrt_graph.vertices_info) - 1, last_vertex_dist_to_final, True)
                return True, rrt_graph

            #Rewiring
            for vertex_info in close:
                dist = math.hypot(new_position[0] - vertex_info.x, new_position[1] - vertex_info.y)
                if rrt_graph.vertices_info[-1].cost + dist < vertex_info.cost and is_collision_free_line(vertex_info.x, vertex_info.y,
                                          new_position[0], new_position[1], obstacle_cloud):
                    vertex_info.cost = rrt_graph.vertices_info[-1].cost + dist
                    rrt_graph.add_edge(vertex_info.id, len(rrt_graph.vertices_info)-1, dist, True)
            break

    return False, rrt_graph