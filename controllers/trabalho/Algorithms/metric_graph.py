"""
A simple implementation of a metric graph, including a utility graph for visualization.
By: Gonçalo Leão
"""
from __future__ import annotations
import math
from typing import Union, Callable

import networkx as nx

from controllers.trabalho.Algorithms.graph import Graph
from controllers.trabalho.Algorithms.vertex_edge import Vertex, Edge
from controllers.trabalho.Algorithms.mutable_priority_queue import MutablePriorityQueue
from controllers.trabalho.utils import is_collision_free_line


class VertexInfo:
    def __init__(self, id: int, x: float, y: float, cost = 0):
        self.id: int = id
        self.x: float = x
        self.y: float = y
        self.cost = cost

class MetricGraph(Graph):
    def __init__(self):
        self.vertex_set: [Vertex] = []
        self.vertices_info: [VertexInfo] = []
        self.visual_graph: nx.Graph = nx.Graph()

    def add_vertex(self, id: int, pos: (float, float), color: str, cost = 0) -> bool:
        if super().add_vertex(id):
            self.vertices_info.append(VertexInfo(id, pos[0], pos[1], cost))
            self.visual_graph.add_node(id, pos=pos, color=color)
            return True
        return False


    def add_edge(self, origin: int, dest: int, weight: float, set_as_path: bool = False) -> bool:
        if super().add_edge(origin, dest, weight, set_as_path):
            self.visual_graph.add_edge(origin, dest)
            return True
        return False

    def get_pruned_path_los(self, origin: int, dest: int, obstacle_cloud, path_v=None) -> [Vertex]:
        pruned_path = []
        if path_v is None:
            path = self.get_path(origin, dest)
        else:
            path = path_v
        print(f"Original path size: {len(path)}")
        if len(path) < 2:
            return []
        pruned_path.append(path[0])
        s_index = 0
        e_index = 1
        while path[e_index].id != dest:
            if is_collision_free_line(self.vertices_info[path[s_index].id].x, self.vertices_info[path[s_index].id].y, self.vertices_info[path[e_index].id].x, self.vertices_info[path[e_index].id].y, obstacle_cloud):
                e_index += 1
            else:
                s_index = e_index - 1
                pruned_path.append(path[e_index-1])
        pruned_path.append(path[-1])
        print(f"Pruned path size: {len(pruned_path)}")
        return pruned_path

    def get_pruned_path_global(self, origin: int, dest: int, obstacle_cloud, path_v=None) -> [Vertex]:
        pruned_path = []
        if path_v is None:
            path = self.get_path(origin, dest)
        else:
            path = path_v
        print(f"Original path size: {len(path)}")
        if len(path) < 2:
            return []
        pruned_path.append(path[0])
        s_index = 0
        e_index = len(path) - 1
        while pruned_path[-1].id != dest:
            if is_collision_free_line(self.vertices_info[path[s_index].id].x, self.vertices_info[path[s_index].id].y, self.vertices_info[path[e_index].id].x, self.vertices_info[path[e_index].id].y, obstacle_cloud):
                pruned_path.append(path[e_index])
                s_index = e_index
                e_index = len(path) - 1
            else:
                e_index -= 1
        print(f"Pruned path size: {len(pruned_path)}")
        return pruned_path

