from __future__ import annotations

from typing import Union


class Vertex:
    def __init__(self, id: int):
        self.id: int = id  # identifier
        self.adj: [Edge] = []  # outgoing edges

        # auxiliary fields
        self.dist: float = 0  # actual distance from the source node to this node
        self.path: Union[Edge, None] = None
        self.cost: float = 0  # cost function of the vertex for A*
        self.queue_index = 0  # required by MutablePriorityQueue
        self.queue_index_f = 0
        self.queue_index_b = 0


    def add_edge(self, dest: Vertex, weight: float) -> Edge:
        new_edge: Edge = Edge(self, dest, weight)
        self.adj.append(new_edge)
        return new_edge


    def remove_edge(self, dest: Vertex):
        for e in self.adj:
            if e.dest == dest:
                self.adj.remove(e)
                return

    def __lt__(self, other: Vertex) -> bool:
        return self.cost < other.cost


class Edge:
    def __init__(self, origin: Vertex, dest: Vertex, weight: float):
        self.origin: Vertex = origin  # origin vertex
        self.dest: Vertex = dest  # destination vertex
        self.weight: float = weight  # edge weight
