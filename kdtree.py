# -*- coding: utf-8 -*-


''' From https://pypi.org/project/KdQuery/
'''


"""Kd-tree implementation.
"""
import math
from collections import deque, namedtuple
import heapq


def is_in_interval(value, inf, sup, dist):
    """Checks if value belongs to the interval [inf - dist, sup + dist].
    """
    return inf - dist < value < sup + dist


def distance_euclidean(point1, point2):
    return math.sqrt(sum([math.pow(point1[i] - point2[i], 2)
                          for i in range(len(point1))]))


Node = namedtuple('Node', 'point region axis active left right data')
"""Internal representation of a node.

The tree is represented by a list of node. Each node is associated to a
point in the k-dimensional space, is inside a region, devises this region
in two parts according to an axis, has two child nodes and stores some
data.

"""


class KDTREE:
    """Kd-tree implementation.

    This class defines one implementation of a kd-tree using a python list to
    save the methods from recursion.

    Args:
        k (int, optional): The number of dimensions of the space.
        capacity (int, optional): The maximum number of nodes in the tree.
        limits (:obj:`list` of :obj:`list` of float or int, optional): A list
            of size k, where each list contains two numbers defining the limits
            of the region which all the nodes will be. If none is passed as
            argument, the region will be all the space, that is, ]-inf, inf[
            for each one of the k axis.

    Attributes:
        node_list (:obj:`list` of :obj:Node): The list of nodes.
        size (int): The number of active nodes in the list.
        next_identifier (int): The identifier of the next node to be inserted
            in the list.
        k (int): The number of dimensions of the space.
        region (:obj:`list` of :obj:`list` of float or int, optional): A list
            of size k, where each list contains two numbers defining the limits
            of the region which all the nodes belong to. If none is passed as
            argument, the region will be all the space, that is, ]-inf, inf[
            for each one of the k axis.

    """

    def __init__(self, k=2, capacity=100000, limits=None):
        self.list_of_nodes = [None] * capacity
        self.size = 0
        self.nxt_identifier = 0
        self.k = k

        # The region of the space where all the points are
        self.regn = limits if limits is not None \
            else [[-math.inf, math.inf]] * k

    def __len__(self):
        return self.size

    def __iter__(self):
        return (node for node in self.list_of_nodes
                if node is not None and node.active)

    def find_node(self, node_id):
        return self.list_of_nodes[node_id]

    def deactiv(self, node_id):
        """Deactivate the node identified by node_id.

        Deactivates the node corresponding to node_id, which means that
        it can never be the output of a nearest_point query.

        Note:
            The node is not removed from the tree, its data is steel available.

        Args:
            node_id (int): The node identifier (given to the user after
                its insertion).

        """
        node = self.list_of_nodes[node_id]
        self.list_of_nodes[node_id] = node._replace(active=False)

    def insert_node(self, point, data=None):
        """Insert a new node in the tree.

        Args:
            point (:obj:`tuple` of float or int): Stores the position of the
                node.
            data (:obj, optional): The information stored by the node.

        Returns:
            int: The identifier of the new node.

        Example:
            >>> tree = Tree(4, 800)
            >>> point = (3, 7)
            >>> data = {'name': Fresnel, 'label': blue, 'speed': 98.2}
            >>> node_id = tree.insert(point, data)

        """
        assert len(point) == self.k

        if self.size == 0:
            if self.regn is None:
                self.regn = [[-math.inf, math.inf]] * self.k
            ax = 0
            return self.new_next_nodes(point, self.regn, ax, data)

        # Iteratively descends to one leaf
        present_id = 0
        while True:
            parent_node_value = self.list_of_nodes[present_id]
            ax = parent_node_value.axis
            if point[ax] < parent_node_value.point[ax]:
                next_to_id, left = parent_node_value.left, True
            else:
                next_to_id, left = parent_node_value.right, False

            if next_to_id is None:
                break

            present_id = next_to_id

        # Get the region delimited by the parent node
        region = parent_node_value.region[:]
        region[ax] = parent_node_value.region[ax][:]

        # Limit to the child's region
        lim = parent_node_value.point[ax]

        # Update reference to the new node
        if left:
            self.list_of_nodes[present_id] = parent_node_value._replace(left=self.size)
            region[ax][1] = lim
        else:
            self.list_of_nodes[present_id] = parent_node_value._replace(right=self.size)
            region[ax][0] = lim

        return self.new_next_nodes(point, region, (ax + 1) % self.k, data)

    def new_next_nodes(self, point, region, axis, data):
        node = Node(point, region, axis, True, None, None, data)

        # Identifier to new node
        node_id = self.nxt_identifier
        self.list_of_nodes[node_id] = node

        self.size += 1
        self.nxt_identifier += 1

        return node_id

    def find_closest_point(self, query, dist_fun=distance_euclidean):
        """Find the point in the tree that minimizes the distance to the query.

        Args:
            query (:obj:`tuple` of float or int): Stores the position of the
                node.
            dist_fun (:obj:`function`, optional): The distance function,
                euclidean distance by default.

        Returns:
            :obj:`tuple`: Tuple of length 2, where the first element is the
                identifier of the nearest node, the second is the distance
                to the query.

        Example:
            >>> tree = Tree(2, 3)
            >>> tree.insert((0, 0))
            >>> tree.insert((3, 5))
            >>> tree.insert((-1, 7))
            >>> query = (-1, 8)
            >>> nearest_node_id, dist = tree.find_nearest_point(query)
            >>> dist
            1

        """
        def find_props(node_id):
            return self.list_of_nodes[node_id][:6]

        return closest__point(query, 0, find_props, dist_fun)

    def get_points_within_distance(self, query, radius, dist_fun=distance_euclidean):
        def get_properties(node_id):
            return self.list_of_nodes[node_id][:6]
        return neigh_in_rad(query, radius, 0, get_properties, dist_fun)


def neigh_in_rad(query, radius, root_id, get_properties, dist_fun=distance_euclidean):
    k = len(query)
    neighbors = []

    # stack_node: stack of identifiers to nodes within a region that
    # contains the query.
    # stack_look: stack of identifiers to nodes within a region that
    # does not contains the query.
    st_node = deque([root_id])
    st_look = deque()

    while st_node or st_look:

        if st_node:
            node_id = st_node.pop()
            look_node = False
        else:
            node_id = st_look.pop()
            look_node = True

        point, region, axis, it_is_active, left, right = get_properties(node_id)

        # Should consider this node?
        # As it is within a region that does not contains the query, maybe
        # there is no chance to find a closer node in this region
        if look_node:
            inside_the_region = True
            for i in range(k):
                inside_the_region &= is_in_interval(query[i], region[i][0],
                                                    region[i][1], radius)
            if not inside_the_region:
                continue

        # Update the distance only if the node is active.
        if it_is_active:
            node_distance = dist_fun(query, point)
            if node_distance <= radius and node_distance != 0:
                neighbors.append((-node_distance, node_id))

        if query[axis] < point[axis]:
            sd_node = left
            sd_look = right
        else:
            sd_node = right
            sd_look = left

        if sd_node is not None:
            st_node.append(sd_node)

        if sd_look is not None:
            st_look.append(sd_look)

    heapq.heapify(neighbors)
    return [item[1] for item in neighbors]


def closest__point(query, root_id, get_properties, dist_fun=distance_euclidean):
    """Find the point in the tree that minimizes the distance to the query.

    This method implements the nearest_point query for any structure
    implementing a kd-tree. The only requirement is a function capable to
    extract the relevant properties from a node representation of the
    particular implementation.

    Args:
        query (:obj:`tuple` of float or int): Stores the position of the
            node.
        root_id (:obj): The identifier of the root in the kd-tree
            implementation.
        get_properties (:obj:`function`): The function to extract the
            relevant properties from a node, namely its point, region,
            axis, left child identifier, right child identifier and
            if it is active. If the implementation does not uses
            the active attribute the function should return always True.
        dist_fun (:obj:`function`, optional): The distance function,
            euclidean distance by default.

    Returns:
        :obj:`tuple`: Tuple of length 2, where the first element is the
            identifier of the nearest node, the second is the distance
            to the query.

    """

    k = len(query)
    dist = math.inf

    closest_node_id = None

    # stack_node: stack of identifiers to nodes within a region that
    # contains the query.
    # stack_look: stack of identifiers to nodes within a region that
    # does not contains the query.
    st_node = deque([root_id])
    stack_look = deque()

    while st_node or stack_look:

        if st_node:
            node_id = st_node.pop()
            look_node = False
        else:
            node_id = stack_look.pop()
            look_node = True

        point, region, axis, active, left, right = get_properties(node_id)

        # Should consider this node?
        # As it is within a region that does not contains the query, maybe
        # there is no chance to find a closer node in this region
        if look_node:
            within_region = True
            for i in range(k):
                within_region &= is_in_interval(query[i], region[i][0],
                                                    region[i][1], dist)
            if not within_region:
                continue

        # Update the distance only if the node is active.
        if active:
            node_dist = dist_fun(query, point)
            if closest_node_id is None or dist > node_dist:
                closest_node_id = node_id
                dist = node_dist

        if query[axis] < point[axis]:
            sd_node = left
            sd_look = right
        else:
            sd_node = right
            sd_look = left

        if sd_node is not None:
            st_node.append(sd_node)

        if sd_look is not None:
            stack_look.append(sd_look)

    return closest_node_id, dist
