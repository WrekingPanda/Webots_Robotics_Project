"""
A simple implementation of a mutable priority queue, required by the Dijkstra and A* algorithms.
This structure allows for both the decrease-key and extract-min operations to run in O(log(N)) time.
By: Gonçalo Leão
"""
from typing import TypeVar, Generic

T = TypeVar('T')
# type T must have: (i) accessible field int queue_index; (ii) operator< defined


# Returns the index of the parent of the i-th element.
def parent(i: int) -> int:
    return i // 2


# Returns the index of the left-child of the i-th element.
def left_child(i: int) -> int:
    return i * 2


class MutablePriorityQueue(Generic[T]):
    def __init__(self) -> None:
        self.heap: [T] = [None]  # indices will be used starting in 1 to facilitate parent / child calculations

    # Checks if the queue is empty.
    # Temporal complexity: O(1)
    def empty(self) -> bool:
        return len(self.heap) == 1

    # Removes the element with the smallest key from the queue, according to the < operator of type T.
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def extract_min(self,bi = False, f = False, d=None) -> T:
        x: T = self.heap[1]
        self.heap[1] = self.heap[-1]
        self.heap.pop()
        if len(self.heap) > 1:
            self.heapify_down(1,bi,f, d)
        if bi:
            if f:
                x.queue_index_f = 0
            else:
                x.queue_index_b = 0
        else:
            x.queue_index = 0
        return x

    # Inserts a new element in the queue.
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def insert(self, x: T, bi = False, f = False, d=None) -> None:
        self.heap.append(x)
        self.heapify_up(len(self.heap) - 1, bi, f, d)

    # Updates an existing element of the queue, so that it has a smaller key, according to the < operator of type T.
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def decrease_key(self, x: T, bi = False, f = False, d=None) -> None:
        if bi:
            if f:
                self.heapify_up(x.queue_index_f, bi, f, d)
            else:
                self.heapify_up(x.queue_index_b, bi, f, d)
        else:
            self.heapify_up(x.queue_index, bi, f, d)

    # Moves the element at index i further up the queue, to reflect its correct key placement (smallest key elements first).
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def heapify_up(self, i: int, bi = False, f = False, d=None) -> None:
        x: T = self.heap[i]
        if bi:
            while i > 1 and d[x.id] < d[self.heap[parent(i)].id]:
                self.set(i, self.heap[parent(i)], bi, f)
                i = parent(i)
            self.set(i, x, bi, f)
        else:
            while i > 1 and x < self.heap[parent(i)]:
                self.set(i, self.heap[parent(i)], bi, f)
                i = parent(i)
            self.set(i, x, bi, f)

    # Moves the element at index i further down the queue, to reflect its correct key placement (smallest key elements first).
    # Temporal complexity: O(log(N)), where N is the number of elements in the queue.
    def heapify_down(self, i: int, bi = False, f = False, d=None) -> None:
        x: T = self.heap[i]
        while True:
            k: int = left_child(i)
            if k >= len(self.heap):
                break  # stop because i-th element has no children
            if bi:
                if (k+1 < len(self.heap)) and d[self.heap[k+1].id] < d[self.heap[k].id]:
                    k += 1  # right child of i is the smallest and should switch with i, rather than the left-child, which is done by default in each iteration
                if not (d[self.heap[k].id] < d[x.id]):
                    break  # stop because child is not smaller than the i-th element
                self.set(i, self.heap[k], bi, f)
                i = k
            else:
                if (k+1 < len(self.heap)) and self.heap[k+1] < self.heap[k]:
                    k += 1  # right child of i is the smallest and should switch with i, rather than the left-child, which is done by default in each iteration
                if not (self.heap[k] < x):
                    break  # stop because child is not smaller than the i-th element
                self.set(i, self.heap[k], bi, f)
                i = k
        self.set(i, x, bi, f)

    # Sets the i-th element of the queue to be x.
    # Temporal complexity: O(1)
    def set(self, i: int, x: T, bi = False, f = False) -> None:
        self.heap[i] = x
        if bi:
            if f:
                x.queue_index_f = i
            else:
                x.queue_index_b = i
        else:
            x.queue_index = i


