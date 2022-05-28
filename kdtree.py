# -*- coding: utf-8 -*-

''' From https://pypi.org/project/KdQuery/
'''

"""Kd-tree implementation.

This module defines one possible kd-tree structure implementation and a
general method to find the nearest node for any kd-tree implementation.

"""
import math
from collections import deque, namedtuple
import heapq


def condition_interval(value, inf, sup, distance_in_path):
    """Checks if value belongs to the interval [inf - distance_in_path, sup + distance_in_path].
    """
    return inf - distance_in_path < value < sup + distance_in_path


def euclid_distance(point1, point2):
    return math.sqrt(sum([math.pow(point1[i] - point2[i], 2)
                          for i in range(len(point1))]))


Node = namedtuple('Node', 'point region axis active left right data')

"""Internal representation of a node.
The tree is represented by a list of node. Each node is associated to a
point in the k-dimensional space, is inside a region, devises this region
in two parts according to an axis, has two child nodes and stores some
data.

"""


class KDTree:
    """Kd-tree.
    """

    def __init__(self, k=2, capacity=100000, limits=None):
        self.node_list = [None] * capacity
        self.size = 0
        self.next_identifier = 0
        self.k = k

        # The region of the space where all the points are
        self.region = limits if limits is not None \
            else [[-math.inf, math.inf]] * k

    def __len__(self):
        return self.size

    def __iter__(self):
        return (node for node in self.node_list
                if node is not None and node.active)

    def get_node(self, node_id):
        return self.node_list[node_id]

    def deactivate(self, node_id):
        """Deactivates the node corresponding to node_id, which means that
        it can never be the output of a nearest_point query.        """
        node = self.node_list[node_id]
        self.node_list[node_id] = node._replace(active=False)

    def insert(self, point, data=None):
        """Insert a new node in the tree."""
        assert len(point) == self.k

        if self.size == 0:
            if self.region is None:
                self.region = [[-math.inf, math.inf]] * self.k
            axis = 0
            return self.new_node(point, self.region, axis, data)

        # Iteratively descends by one leaf
        current_id = 0
        while True:
            parent_node = self.node_list[current_id]
            axis = parent_node.axis
            if point[axis] < parent_node.point[axis]:
                next_id, left = parent_node.left, True
            else:
                next_id, left = parent_node.right, False

            if next_id is None:
                break

            current_id = next_id

        # Get the region delimited by the parent node
        region = parent_node.region[:]
        region[axis] = parent_node.region[axis][:]

        # Limit to the child's region
        limit = parent_node.point[axis]

        # Update reference to the new node
        if left:
            self.node_list[current_id] = parent_node._replace(left=self.size)
            region[axis][1] = limit
        else:
            self.node_list[current_id] = parent_node._replace(right=self.size)
            region[axis][0] = limit

        return self.new_node(point, region, (axis + 1) % self.k, data)

    def new_node(self, point, region, axis, data):
        node = Node(point, region, axis, True, None, None, data)

        # Identifier to new node
        node_id = self.next_identifier
        self.node_list[node_id] = node

        self.size += 1
        self.next_identifier += 1

        return node_id

    def find_nearest_point(self, query, dist_fun=euclid_distance):
        """Find the point in the tree that minimizes the distance to the query."""
        def get_properties(node_id):
            return self.node_list[node_id][:6]

        return nearest_point(query, 0, get_properties, dist_fun)

    def find_points_within_radius(self, query, radius, dist_fun=euclid_distance):
        def get_properties(node_id):
            return self.node_list[node_id][:6]
        return neighbor_within_radius(query, radius, 0, get_properties, dist_fun)


def neighbor_within_radius(query, radius, root_id, get_properties, dist_fun=euclid_distance):
    k = len(query)
    neighbors = []

    # stack_node: stack of identifiers to nodes within a region that
    # contains the query.
    # stack_look: stack of identifiers to nodes within a region that
    # does not contains the query.
    stack_node = deque([root_id])
    stack_look = deque()

    while stack_node or stack_look:

        if stack_node:
            node_id = stack_node.pop()
            look_node = False
        else:
            node_id = stack_look.pop()
            look_node = True

        point, region, axis, active, left, right = get_properties(node_id)

        # Should consider this node?
        # As it is within a region that does not contains the query, maybe
        # there is no chance to find a closer node in this region
        if look_node:
            inside_region = True
            for i in range(k):
                inside_region &= condition_interval(query[i], region[i][0],
                                                    region[i][1], radius)
            if not inside_region:
                continue

        # Update the distance only if the node is active.
        if active:
            node_distance = dist_fun(query, point)
            if node_distance <= radius and node_distance != 0:
                neighbors.append((-node_distance, node_id))

        if query[axis] < point[axis]:
            side_node = left
            side_look = right
        else:
            side_node = right
            side_look = left

        if side_node is not None:
            stack_node.append(side_node)

        if side_look is not None:
            stack_look.append(side_look)

    heapq.heapify(neighbors)
    return [item[1] for item in neighbors]


def nearest_point(query, root_id, get_properties, dist_fun=euclid_distance):
    """Find the point in the tree that minimizes the distance to the query.

    This method implements the nearest_point query for any structure
    implementing a kd-tree. The only requirement is a function capable to
    extract the relevant properties from a node representation of the
    particular implementation.    """

    k = len(query)
    distance_in_path = math.inf

    nearest_node_id = None

    # stack_node: stack of identifiers to nodes within a region that
    # contains the query.
    # stack_look: stack of identifiers to nodes within a region that
    # does not contains the query.
    stack_node = deque([root_id])
    stack_look = deque()

    while stack_node or stack_look:

        if stack_node:
            node_id = stack_node.pop()
            look_node = False
        else:
            node_id = stack_look.pop()
            look_node = True

        point, region, axis, active, left, right = get_properties(node_id)

        # Should consider this node?
        # As it is within a region that does not contains the query, maybe
        # there is no chance to find a closer node in this region
        if look_node:
            inside_region = True
            for i in range(k):
                inside_region &= condition_interval(query[i], region[i][0],
                                                    region[i][1], distance_in_path)
            if not inside_region:
                continue

        # Update the distance only if the node is active.
        if active:
            node_distance = dist_fun(query, point)
            if nearest_node_id is None or distance_in_path > node_distance:
                nearest_node_id = node_id
                distance_in_path = node_distance

        if query[axis] < point[axis]:
            side_node = left
            side_look = right
        else:
            side_node = right
            side_look = left

        if side_node is not None:
            stack_node.append(side_node)

        if side_look is not None:
            stack_look.append(side_look)

    return nearest_node_id, distance_in_path
