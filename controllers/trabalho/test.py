import csv
import math
import time
from typing import Union, Dict

from scipy.spatial import KDTree

from matplotlib import pyplot as plt, patches
from matplotlib.collections import PatchCollection
from matplotlib.patches import Rectangle

from controller import Robot, Lidar, LidarPoint, Compass, GPS, Supervisor
import numpy as np

from controllers.trabalho.Algorithms.graph import Graph
from controllers.trabalho.Algorithms.metric_graph import VertexInfo, MetricGraph
from controllers.trabalho.Algorithms.vertex_edge import Vertex
from controllers.trabalho.utils import move_robot_to, is_collision_free_line, is_collision_free_point, warp_robot
from controllers.trabalho.Algorithms.Graph_utils import create_grid_graph, create_prm_graph, find_closest_accessible_vertex
from controllers.trabalho.Algorithms.RRT import create_rrt, create_rrt_star
from controllers.trabalho.Algorithms.APF import compute_repulsive_force, compute_attractive_force, compute_resulting_force, draw_quiver_plots
import networkx as nx


import random

def main(fast=False) -> None:
    supervisor: Supervisor = Supervisor()
    algorithms = ['A*', 'Dijkstra', 'D*', 'Bidirectional', 'RRT', 'RRT*','APF']
    distances = {}
    tempos = {}
    time_los = 0
    time_global = 0
    count = 0
    # robot: Robot = Robot()
    custom_maps_filepath: str = '../../worlds/custom_maps/'
    map_name = 'VCorner'
    obstacle_points_filename: str = custom_maps_filepath + map_name + '_points.csv'
    final_position: (float, float) = (1.76, 1.76)

    timestep: int = int(supervisor.getBasicTimeStep())  # in ms

    lidar: Lidar = supervisor.getDevice('lidar')
    lidar.enable(timestep)
    lidar.enablePointCloud()

    compass: Compass = supervisor.getDevice('compass')
    compass.enable(timestep)

    gps: GPS = supervisor.getDevice('gps')
    gps.enable(timestep)
    supervisor.step()

    # Read the robot's initial pose
    gps_readings: [float] = gps.getValues()
    robot_position: (float, float) = (gps_readings[0], gps_readings[1])
    start_position = (gps_readings[0], gps_readings[1])
    compass_readings: [float] = compass.getValues()
    robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])

    # Read the obstacles point cloud
    obstacle_points: [(float, float, float)] = []
    with open(obstacle_points_filename, 'r') as file:
        csvreader = csv.reader(file)
        for row in csvreader:
            obstacle_points.append([float(row[0]), float(row[1]), 0.0])
    obstacle_cloud: np.ndarray = np.asarray(obstacle_points)

    # Check that the final position is in the free space
    if not is_collision_free_point(final_position[0], final_position[1], obstacle_cloud):
        print("Final position ", final_position, " is colliding with an obstacle")
        return
    graphs = {}
    for algorithm in algorithms:
        warp_robot(supervisor, "EPUCK", start_position, True)
        if algorithm == 'A*' or algorithm == 'Dijkstra' or algorithm == 'D*' or algorithm == 'Bidirectional':  # Graph search
            for graph_type in ['grid', 'PRM']:
                warp_robot(supervisor, "EPUCK", start_position, True)
                # Create the graph and find the shortest path
                if algorithm == 'A*':
                    if graph_type == 'grid':
                        graphs['grid'] = create_grid_graph(start_position, final_position, obstacle_cloud)
                    else:
                        graphs['PRM'] = create_prm_graph(start_position, final_position, obstacle_cloud)
                graph = graphs[graph_type]
                vertex_positions: Dict[int, (float, float)] = nx.get_node_attributes(graph.visual_graph, 'pos')
                vertex_distances: Dict[int, float] = {}
                for (id, position) in vertex_positions.items():
                    vertex_distances[id] = math.hypot(position[0] - final_position[0],
                                                      position[1] - final_position[1])

                def dist_func(v: Vertex):
                    return vertex_distances[v.id]

                start = time.time()
                if algorithm == 'A*':
                    graph.a_star(len(graph.vertex_set) - 2, dist_func, len(graph.vertex_set) - 1)
                    end = time.time()
                    print(f"Elapsed time for {algorithm} using {graph_type} : ", end - start, " seconds")
                    tempos[algorithm + '_' + graph_type] = end - start
                    path = graph.get_path(len(graph.vertex_set) - 2, len(graph.vertex_set) - 1)
                    distance = graph.get_path_dist(path)
                    # grid_graph.a_star(len(grid_graph.vertex_set) - 2, dist_func, len(grid_graph.vertex_set) - 1)
                # grid_graph.d_star(len(grid_graph.vertex_set) - 2, len(grid_graph.vertex_set) - 1)
                # grid_graph.dijkstra(len(grid_graph.vertex_set) - 2, len(grid_graph.vertex_set) - 1)
                elif algorithm == 'Dijkstra':
                    graph.dijkstra(len(graph.vertex_set) - 2, len(graph.vertex_set) - 1)
                    end = time.time()
                    print(f"Elapsed time for {algorithm} using {graph_type} : ", end - start, " seconds")
                    tempos[algorithm + '_' + graph_type] = end - start
                    path= graph.get_path(len(graph.vertex_set) - 2, len(graph.vertex_set) - 1)
                    distance = graph.get_path_dist(path)
                elif algorithm == 'D*':
                    graph.d_star(len(graph.vertex_set) - 2, len(graph.vertex_set) - 1)
                    end = time.time()
                    print(f"Elapsed time for {algorithm} using {graph_type} : ", end - start, " seconds")
                    tempos[algorithm + '_' + graph_type] = end - start
                    path = graph.get_path_reverse(len(graph.vertex_set) - 2, len(graph.vertex_set) - 1)
                    distance = graph.get_path_dist(path)
                elif algorithm == 'Bidirectional':
                    path = graph.get_path_bidirectional_dijkstra(len(graph.vertex_set) - 2,
                                                                 len(graph.vertex_set) - 1)
                    distance = graph.get_path_dist(path)
                    end = time.time()
                    print(f"Elapsed time for {algorithm} using {graph_type} : ", end - start, " seconds")
                    tempos[algorithm + '_' + graph_type] = end - start

                start_prun_los_time = time.time()
                path_los = graph.get_pruned_path_los(len(graph.vertex_set) - 2, len(graph.vertex_set) - 1,
                                                               obstacle_cloud, path)
                distance_los = graph.get_path_dist(path_los)
                start_prun_global_time = time.time()
                print("Elapsed time for Pruning LOS : ", start_prun_global_time - start_prun_los_time, " seconds")
                time_los += start_prun_global_time - start_prun_los_time
                path_global = graph.get_pruned_path_global(len(graph.vertex_set) - 2, len(graph.vertex_set) - 1,
                                                                  obstacle_cloud, path)
                distance_global=graph.get_path_dist(path_global)
                end_pruning_time = time.time()
                print("Elapsed time for Pruning Global : ", end_pruning_time - start_prun_global_time, " seconds")
                time_global += end_pruning_time - start_prun_global_time
                count+=1
                distances[algorithm+'_'+graph_type+'_original'] = distance
                distances[algorithm+'_'+graph_type+'_los'] = distance_los
                distances[algorithm + '_' + graph_type + '_global'] = distance_global

                if not fast:
                    new_vertex_colors: dict = {}
                    for vertex in path_global:
                        new_vertex_colors[vertex.id] = "red"
                    for vertex in graph.vertex_set:
                        if vertex.dist == math.inf:
                            new_vertex_colors[vertex.id] = "orange"
                    nx.set_node_attributes(graph.visual_graph, new_vertex_colors, 'color')

                    # Create the PolyCollection for the obstacles, for the plot
                    pat: [patches.Rectangle] = []
                    for point in obstacle_cloud:
                        pat.append(Rectangle((point[0], point[1]), 0.001, 0.001,
                                             linewidth=1, edgecolor='black', facecolor='none'))

                    # Show the graph
                    fig, ax = plt.subplots()
                    nx.draw_networkx(graph.visual_graph, vertex_positions, node_size=10,
                                     node_color=nx.get_node_attributes(graph.visual_graph, 'color').values(),
                                     with_labels=False)
                    col: PatchCollection = PatchCollection(pat)
                    col.set_edgecolor('black')
                    col.set_linewidth(1)
                    ax.add_collection(col)
                    plt.show()

                    # Move the robot through the path
                    for vertex in path:
                        # Read the ground-truth (correct robot pose)
                        supervisor.step()
                        gps_readings: [float] = gps.getValues()
                        robot_position: (float, float) = (gps_readings[0], gps_readings[1])
                        compass_readings: [float] = compass.getValues()
                        robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])

                        move_robot_to(supervisor, robot_position, robot_orientation, vertex_positions[vertex.id], 0.1,
                                      math.pi)

        elif algorithm == 'RRT' or algorithm == 'RRT*':
            start = time.time()
            if algorithm == 'RRT':
                found_path, rrt_graph = create_rrt(start_position, final_position, obstacle_cloud)
            else:
                found_path, rrt_graph = create_rrt_star(start_position, final_position, obstacle_cloud)
            end = time.time()
            print(f"Elapsed time for {algorithm} : ", end - start, " seconds")
            tempos[algorithm] = end-start
            if not found_path:
                print("No path found")
                return
            vertex_positions: Dict[int, (float, float)] = nx.get_node_attributes(rrt_graph.visual_graph, 'pos')
            path = rrt_graph.get_path(0, len(rrt_graph.vertex_set) - 1)
            distance = rrt_graph.get_path_dist(path)
            start_prun_los_time = time.time()
            path_los = rrt_graph.get_pruned_path_los(len(rrt_graph.vertex_set) - 2,
                                                               len(rrt_graph.vertex_set) - 1,obstacle_cloud, path)
            distance_los = rrt_graph.get_path_dist(path_los)
            start_prun_global_time = time.time()
            print("Elapsed time for Pruning LOS : ", start_prun_global_time - start_prun_los_time, " seconds")
            time_los += start_prun_global_time - start_prun_los_time
            path_global = rrt_graph.get_pruned_path_global(len(rrt_graph.vertex_set) - 2,
                                                                  len(rrt_graph.vertex_set) - 1,obstacle_cloud, path)
            distance_global = rrt_graph.get_path_dist(path_global)
            end_pruning_time = time.time()
            print("Elapsed time for Pruning Global : ", end_pruning_time - start_prun_global_time, " seconds")
            time_global += end_pruning_time - start_prun_global_time
            count += 1
            distances[algorithm + '_original'] = distance
            distances[algorithm + '_los'] = distance_los
            distances[algorithm + '_global'] = distance_global

            if not fast:
                # Mark with a new color the unvisited vertices and the ones in the path
                new_vertex_colors: dict = {}
                for vertex in path:
                    new_vertex_colors[vertex.id] = "red"
                nx.set_node_attributes(rrt_graph.visual_graph, new_vertex_colors, 'color')
                # Create the PolyCollection for the obstacles, for the plot
                pat: [patches.Rectangle] = []
                for point in obstacle_cloud:
                    pat.append(Rectangle((point[0], point[1]), 0.001, 0.001,
                                         linewidth=1, edgecolor='black', facecolor='none'))

                # Show the graph
                fig, ax = plt.subplots()
                nx.draw_networkx(rrt_graph.visual_graph, vertex_positions, node_size=10,
                                 node_color=nx.get_node_attributes(rrt_graph.visual_graph, 'color').values(),
                                 with_labels=False)
                col: PatchCollection = PatchCollection(pat)
                col.set_edgecolor('black')
                col.set_linewidth(1)
                ax.add_collection(col)
                plt.show()

                # Move the robot through the path
                for vertex in path:
                    # Read the ground-truth (correct robot pose)
                    supervisor.step()
                    gps_readings: [float] = gps.getValues()
                    robot_position: (float, float) = (gps_readings[0], gps_readings[1])
                    compass_readings: [float] = compass.getValues()
                    robot_orientation: float = math.atan2(compass_readings[0], compass_readings[1])

                    move_robot_to(supervisor, robot_position, robot_orientation, vertex_positions[vertex.id], 0.1,
                                  math.pi)


        elif algorithm == 'APF':
            max_distance_to_goal: float = 0.01

            start = time.time()
            draw_quiver_plots(supervisor, final_position, obstacle_cloud, fast)
            end = time.time()
            print("Elapsed time for APF : ", end - start, " seconds")
            tempos[algorithm] = end - start
            # Move the robot back to the initial position
            warp_robot(supervisor, "EPUCK", robot_position)
            distance = 0

            while math.hypot(robot_position[0] - final_position[0],
                             robot_position[1] - final_position[1]) > max_distance_to_goal:
                supervisor.step()
                resulting_force: np.ndarray = compute_resulting_force(np.array(robot_position),
                                                                      np.array(final_position),
                                                                      lidar.getPointCloud())
                force_norm = np.linalg.norm(resulting_force)
                if force_norm <= max_distance_to_goal:
                    break  # End the run because the resulting force is too low
                step_distance: float = 0.01
                new_position: (float, float) = (robot_position[0] + step_distance * resulting_force[0] / force_norm,
                                                robot_position[1] + step_distance * resulting_force[1] / force_norm)
                move_robot_to(supervisor, robot_position, robot_orientation, new_position, 0.1, math.pi)
                distance += math.hypot(robot_position[0] - new_position[0],
                           robot_position[1] - new_position[1])
                gps_readings = gps.getValues()
                robot_position = (gps_readings[0], gps_readings[1])
                compass_readings = compass.getValues()
                robot_orientation = math.atan2(compass_readings[0], compass_readings[1])

            distances[algorithm] = distance
    min = 200000
    for k in distances.keys():
        if distances[k]<min and distances[k] > 0:
            min = distances[k]
    for k in distances.keys():
        if distances[k]>0:
            distances[k] = min/distances[k]
    print(f'Distances: {distances}')
    print(f'Times: {tempos}')
    print(f'Time for Los Pruning: {time_los/count}')
    print(f'Time for Global Pruning: {time_global/count}')

if __name__ == '__main__':
    fast = False
    main(fast)



