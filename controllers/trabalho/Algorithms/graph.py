"""
A simple implementation of a graph, including a relax operation for single source shortest-path algorithms.
By: Gonçalo Leão
"""
from __future__ import annotations
import math
from typing import Union, Callable

from controllers.trabalho.Algorithms.vertex_edge import Vertex, Edge
from controllers.trabalho.Algorithms.mutable_priority_queue import MutablePriorityQueue



class Graph:
    def __init__(self):
        self.vertex_set: [Vertex] = []

    # Finds a vertex with a given ID.
    def find_vertex(self, id: int) -> Union[Vertex, None]:
        for v in self.vertex_set:
            if v.id == id:
                return v
        return None

    # Adds a vertex with a given content or info to a graph.
    # Returns true if successful, and false if a vertex with that content already exists.
    def add_vertex(self, id: int) -> bool:
        if self.find_vertex(id) is not None:
            return False
        self.vertex_set.append(Vertex(id))
        return True

    # Adds an edge to the graph, given the contents of the origin and
    # destination vertices and the edge weight(w).
    # Returns true if successful, and false if the source or destination vertex does not exist.
    def add_edge(self, origin: int, dest: int, weight: float, set_as_path: bool = False) -> bool:
        v1: Vertex = self.find_vertex(origin)
        v2: Vertex = self.find_vertex(dest)
        if v1 is None or v2 is None:
            return False
        new_edge: Edge = v1.add_edge(v2, weight)
        if set_as_path:
            v2.path = new_edge
        return True

    def merge(self, other: Graph) -> None:
        self.vertex_set.extend(other.vertex_set)

    def dijkstra(self, origin: int, dest: int = -1) -> bool:
        return self.a_star(origin, lambda _: 0, dest)

    # Arguments:
    # origin: Source node ID
    # heuristic_cost_function: function that estimates the distance from the vertex argument to the goal
    # dest: Destination node ID (-1 to search for every accessible node)
    def a_star(self, origin: int, heuristic_cost_function: Callable[[Vertex], float], dest: int = -1) -> bool:
        # Initialize the vertices
        for v in self.vertex_set:
            v.dist = math.inf
            v.path = None

        # Retrieve initial vertex s and set its dist to 0
        s: Vertex = self.find_vertex(origin)
        s.dist = 0

        # Create the priority queue and add s
        q: MutablePriorityQueue = MutablePriorityQueue()
        q.insert(s)

        # Process the vertices
        while not q.empty():
            v: Vertex = q.extract_min()
            if v.id == dest:  # the destination was found so the search can stop
                return True
            for e in v.adj:
                old_dist: float = e.dest.dist
                if self.relax(e, heuristic_cost_function):  # a shorter path was found
                    if old_dist == math.inf:  # new vertex was found
                        q.insert(e.dest)
                    else:  # a shorter path to an unprocessed vertex was found (unprocessed = vertex still in the queue)
                        q.decrease_key(e.dest)
        return False


    def d_star(self, origin: int, dest: int = -1) -> bool:
        # Initialize the vertices
        for v in self.vertex_set:
            v.dist = math.inf
            v.path = None

        end: Vertex = self.find_vertex(dest)
        end.dist = 0

        q: MutablePriorityQueue = MutablePriorityQueue()
        q.insert(end)

        while not q.empty():
            v: Vertex = q.extract_min()
            if v.id == origin:  # the destination was found so the search can stop
                return True
            for e in v.adj:
                old_dist: float = e.dest.dist
                if e.origin.dist + e.weight < e.dest.dist:
                    e.dest.dist = e.origin.dist + e.weight
                    e.dest.path = e
                    e.dest.cost = e.dest.dist
                    if old_dist == math.inf:  # new vertex was found
                        q.insert(e.dest)
                    else:  # a shorter path to an unprocessed vertex was found (unprocessed = vertex still in the queue)
                        q.decrease_key(e.dest)
        return False


    def bidirectional_dijkstra(self, origin: int, dest: int = -1):
        mu = math.inf

        # Initialize the vertices
        for v in self.vertex_set:
            v.dist = math.inf
            v.path = None
            v.queue_index = 0

        pf = [None] * len(self.vertex_set)
        pb = [None] * len(self.vertex_set)

        df = [math.inf] * len(self.vertex_set)
        db = [math.inf] * len(self.vertex_set)

        df[origin] = 0
        db[dest] = 0

        start: Vertex = self.find_vertex(origin)

        end: Vertex = self.find_vertex(dest)



        q_forward: MutablePriorityQueue = MutablePriorityQueue()
        q_forward.insert(start)

        q_backwards: MutablePriorityQueue = MutablePriorityQueue()
        q_backwards.insert(end)

        # Process the vertices
        while (not q_forward.empty()) and (not q_backwards.empty()):
            v: Vertex = q_forward.extract_min(bi=True,f=True, d=df)
            u: Vertex = q_backwards.extract_min(bi=True,f=False, d=db)



            for e in v.adj:
                old_dist: float = df[e.dest.id]
                if df[e.dest.id] > df[v.id] + e.weight:
                    df[e.dest.id] = df[v.id] + e.weight
                    pf[e.dest.id] = v
                    if old_dist == math.inf:
                        q_forward.insert(e.dest, bi=True, f=True, d=df)
                    else:
                        q_forward.decrease_key(e.dest, bi=True,f=True,d=df)
                    if db[e.dest.id] != math.inf and df[v.id] + e.weight + db[e.dest.id] < mu:
                        mu = df[v.id] + e.weight + db[e.dest.id]
                        answer_backwards = e.dest.id
                        answer_forward = v.id

            for e in u.adj:
                old_dist: float = db[e.dest.id]
                if db[e.dest.id] > db[u.id] + e.weight:
                    db[e.dest.id] = db[u.id] + e.weight
                    pb[e.dest.id] = u
                    if old_dist == math.inf:
                        q_backwards.insert(e.dest, bi=True, f=False, d=db)
                    else:
                        q_backwards.decrease_key(e.dest, bi=True,f=False,d=db)
                    if df[e.dest.id] != math.inf and db[u.id] + e.weight + df[e.dest.id] < mu:
                        mu = db[u.id] + e.weight + df[e.dest.id]
                        answer_backwards = u.id
                        answer_forward = e.dest.id

            if df[v.id] + db[u.id] > mu:
                return [True, answer_forward, answer_backwards, pf, pb]

        return [False]

    def get_path_bidirectional_dijkstra(self, origin: int, dest: int = -1):
        forward: [Vertex] = []
        backwards: [Vertex] = []

        r = self.bidirectional_dijkstra(origin, dest)

        if r[0]:
            v: Vertex = self.find_vertex(r[1])
            forward.append(v)
            while r[3][v.id] is not None:
                v = r[3][v.id]
                forward.append(v)
            forward.reverse()

            v: Vertex = self.find_vertex(r[2])
            backwards.append(v)
            while r[4][v.id] is not None:
                v = r[4][v.id]
                backwards.append(v)
        return forward+backwards

    def relax(self, edge: Edge, heuristic_cost_function: Callable[[Vertex], float]) -> bool:
        if edge.origin.dist + edge.weight < edge.dest.dist:
            # Update edge.dest.dist, edge.dest.path and edge.dest.cost
            edge.dest.dist = edge.origin.dist + edge.weight
            edge.dest.path = edge
            edge.dest.cost = edge.dest.dist + heuristic_cost_function(edge.dest)
            return True
        else:
            return False

    def get_path(self, origin: int, dest: int) -> [Vertex]:
        path: [Vertex] = []

        # Add the destination vertex to the path
        v: Vertex = self.find_vertex(dest)
        if v is None or v.dist == math.inf:  # missing or disconnected vertex
            return []
        path.append(v)

        # Follow the path in reverse order and add each vertex to the path
        while v.path is not None:
            v = v.path.origin
            path.append(v)
        # Reverse the path
        path.reverse()

        # Check that the origin vertex was found, if not return []
        if len(path) == 0 or path[0].id != origin:
            return []
        return path

    def get_path_reverse(self, origin: int, dest: int) -> [Vertex]:
        path: [Vertex] = []

        # Add the destination vertex to the path
        v: Vertex = self.find_vertex(origin)
        if v is None or v.dist == math.inf:  # missing or disconnected vertex
            return []
        path.append(v)

        # Follow the path in reverse order and add each vertex to the path
        while v.path is not None:
            v = v.path.origin
            path.append(v)

        # Check that the origin vertex was found, if not return []
        if len(path) == 0 or path[0].id != origin:
            return []
        return path



