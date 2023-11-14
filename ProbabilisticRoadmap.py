import sys
from collections import deque

import shapely
import random
import math

# Author: Ben Williams '25
# Date: November 10th, 2023


class ProbabilisticRoadmap:
    def __init__(self, workspace, num_vertices):
        self.workspace = workspace
        self.vertices = self.instantiate_random_vertices(num_vertices)
        self.graph = self.create_near_neighbor_graph()

    # Create the random vertices on the roadmap, such that they do not land on a polygon
    # Points are in c-space and are all theta values
    def instantiate_random_vertices(self, num_points):
        dimensions = len(self.workspace.joints)
        points = []
        # Make random points until we have enough valid ones on the workspace
        while len(points) < num_points:
            new_point = tuple([random.random() * 2 * math.pi for _ in range(dimensions)])
            if not self.workspace.is_collision_state(new_point):
                points.append(new_point)

        return points

    # Create a graph from the vertices by creating edges between near neighbors
    # Optional Parameter: Degree - The number of edges each vertex has, otherwise will default to a log value
    def create_near_neighbor_graph(self, degree=None):
        graph = dict()

        # We need to create at least a logarithmic number of edges from each point
        if not degree:
            degree = math.ceil(math.log(len(self.vertices), math.e))

        for vertex in self.vertices:
            graph[vertex] = self.get_k_nearest_neighbors(vertex, degree)

        return graph

    # Returns a list of the k nearest neighbors in "euclidean" distance between the thetas
    # Does minor interpolation to reduce collisions
    def get_k_nearest_neighbors(self, point, k):
        near_neighbors = [None for _ in range(k)]
        near_neighbors_distances = [math.inf for _ in range(k)]

        for i in range(len(self.vertices)):
            if self.vertices[i] == point:
                continue

            distance = self.angular_euclidean_distance(point, self.vertices[i])

            if distance < near_neighbors_distances[k - 1]:
                if self.collision_interpolation(point, self.vertices[i], 5):
                    continue
                near_neighbors.pop(k - 1)
                near_neighbors_distances.pop(k - 1)
                near_neighbors.insert(0, self.vertices[i])
                near_neighbors_distances.insert(0, distance)

        # We may have less than k reachable neighbors (after interpolation)
        for i in range(len(near_neighbors) - 1, -1, -1):
            if near_neighbors[i] is None:
                del near_neighbors[i]

        return near_neighbors

    # Returns True if there is a collision found between the two edges, False otherwise
    def collision_interpolation(self, point_1, point_2, num_interpolation_points):
        angle_steps = []
        # Find the amount to interpolate by for each dimension of the point
        for i in range(len(point_1)):
            difference = min(abs(point_1[i] - point_2[i]), abs(2 * math.pi - point_1[i] - point_2[i]))
            angle_steps.append(difference / (num_interpolation_points + 1))

        # Interpolate
        for interpolation_step in range(1, num_interpolation_points + 1):
            interpolated_point = [point_1[i] + angle_steps[i] * interpolation_step for i in range(len(point_1))]
            if self.workspace.is_collision_state(interpolated_point):
                return True

        # No collision found
        return False

    # Given a start and a goal as tuples of the angles in c-space, find a path between the two using bfs
    def find_path_bfs(self, start, goal):
        backchain = dict()
        queue = deque()

        start_neighbor = self.get_k_nearest_neighbors(start, 1)[0]
        backchain[start_neighbor] = start
        backchain[start] = None
        end_neighbor = self.get_k_nearest_neighbors(goal, 1)[0]

        # Perform BFS
        queue.append(start_neighbor)
        while len(queue) > 0:
            curr_vertex = queue.popleft()
            if curr_vertex == end_neighbor:
                break

            for neighbor in self.graph[curr_vertex]:
                if neighbor not in backchain.keys():
                    backchain[neighbor] = curr_vertex
                    queue.append(neighbor)

        # If end neighbor was never reached, there is no solution in the PRM
        if end_neighbor not in backchain.keys():
            print("Solution not found")
            return None

        # Backchain
        path = [goal]
        curr_vertex = end_neighbor
        while curr_vertex:
            path.append(curr_vertex)
            curr_vertex = backchain[curr_vertex]
        path.reverse()

        return path

    @staticmethod
    # Returns the euclidean distance of two n dimensional angular points
    # This considers the fact that we can wrap around (from 0 to 2pi and 2pi to 0)
    def angular_euclidean_distance(point1, point2):
        sum_of_squares_subtracted = 0
        for i in range(len(point1)):
            counter_clockwise = (point1[i] - point2[i]) ** 2
            clockwise = (2 * math.pi - abs(point1[i] - point2[i])) ** 2
            sum_of_squares_subtracted += min(counter_clockwise, clockwise)
        return math.sqrt(sum_of_squares_subtracted)


if __name__ == "__main__":
    angles_1 = [0.5, 0.5]
    angles_2 = [1, 1]
    angles_3 = [0.5, 6]
    print(ProbabilisticRoadmap.angular_euclidean_distance(angles_1, angles_2))
    print(ProbabilisticRoadmap.angular_euclidean_distance(angles_1, angles_3))
    print(ProbabilisticRoadmap.angular_euclidean_distance(angles_3, angles_1))

