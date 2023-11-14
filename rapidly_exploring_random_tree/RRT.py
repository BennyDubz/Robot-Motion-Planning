# using shapely for collision detection

from CarWorkspace import CarWorkspace

import math
import random

import planar_trajectory
from shapely.geometry import Polygon, Point


# An RRT vertex built for the car motion
class RRTVertex:
    # State is the "q" from the planar trajectory, consisting of an x, y, and theta coordinate
    def __init__(self, tree, state, parent=None, parent_move=None):
        self.tree = tree
        self.state = state
        self.parent = parent
        # The move the parent made to get here
        self.parent_move = parent_move
        self.children = []

    # Returns the distance between this vertex and an x, y coordinate
    def euclidean_distance(self, other_location):
        pt_1 = (other_location[0] - self.state[0]) ** 2
        pt_2 = (other_location[1] - self.state[1]) ** 2
        return math.sqrt(pt_1 + pt_2)

    # Expands the tree from this vertex by creating RRTVertices with +/- 1 velocity and {-1, 0, 1} omegas
    # Returns True if the tree can be expanded from here, False if this vertex already has children
    def expand_tree(self):
        if self.children:
            return False

        # All possible moves
        for move in planar_trajectory.controls_rs:
            avg_angle = self.state[2] + (move[2] / 2)
            change = (math.cos(avg_angle) * move[0], math.sin(avg_angle) * move[0], move[2])
            new_state = tuple([self.state[i] + change[i] for i in range(3)])

            # Ignore collision states
            if self.tree.workspace.is_collision_simplified(new_state[0], new_state[1]):
                continue

            # Ignore repetitive states
            if new_state in self.tree.vertex_states:
                continue

            # Check that we are within the bounds of the board
            # if not (self.tree.min_x <= new_state[0] <= self.tree.max_x):
            #     continue
            # if not (self.tree.min_y <= new_state[1] <= self.tree.max_y):
            #     continue

            new_node = RRTVertex(self.tree, new_state, self, move)
            self.children.append(new_node)

        # The possibility that all moves from here lead to states that have already been seen
        if not self.children:
            return False

        return True

    def __str__(self):
        return "Vertex at location: " + str((self.state[0], self.state[1])) + " with angle " + str(self.state[2])


class RRT:
    # Takes a start and goal state of the car, as well as the range for the tree to consider
    def __init__(self, car_workspace, num_vertices, start, min_x, min_y, max_x, max_y, goal=None):
        self.start = RRTVertex(self, tuple(start))
        self.workspace = car_workspace
        self.min_x = min_x
        self.max_x = max_x
        self.min_y = min_y
        self.max_y = max_y
        self.vertices = [self.start]
        self.vertex_states = {self.start.state}
        self.create_tree(num_vertices, goal)
        self.goal = goal

    def create_tree(self, max_vertices, goal):
        rand_goal_greed = 0.01

        while len(self.vertices) < max_vertices:
            rand_point = self.get_random_point()

            # Optional chance for greedy searching
            if goal:
                if random.random() < rand_goal_greed:
                    nearest_vertices = self.get_nearest_vertices((goal[0], goal[1]))
                    # Check for success and break out early
                    for near_vertex in nearest_vertices:
                        if near_vertex.euclidean_distance(goal) < 0.5:
                            return
                else:
                    nearest_vertices = self.get_nearest_vertices(rand_point)
            else:
                nearest_vertices = self.get_nearest_vertices(rand_point)

            for near_vertex in nearest_vertices:
                # If we can expand the tree here
                if near_vertex.expand_tree():
                    # Add new children to the vertex list
                    self.vertices += near_vertex.children
                    for child in near_vertex.children:
                        self.vertex_states.add(child.state)

    def get_random_point(self):
        rand_x = random.uniform(self.min_x, self.max_x)
        rand_y = random.uniform(self.min_y, self.max_y)
        return rand_x, rand_y

    # Returns the list of the nearest vertex or overlapping vertices given a point
    # May be more than one vertex if a location has overlapping x, y coordinates with different orientations
    def get_nearest_vertices(self, point):
        curr_min = math.inf
        curr_best = []
        for vertex in self.vertices:
            distance = vertex.euclidean_distance(point)
            if distance < curr_min:
                curr_min = distance
                curr_best = [vertex]
            elif distance == curr_min:
                curr_best.append(vertex)
        return curr_best

    # Given a goal location and orientation, return a path that gets closest to that spot
    def get_path(self, goal):
        nearest_vertices = self.get_nearest_vertices((goal[0], goal[1]))
        min_angle_dif = math.inf
        best_vertex = None

        # Pick the vertex with the closest orientation
        for near_vertex in nearest_vertices:
            angle_diff = abs(goal[2] - near_vertex.state[2])
            if angle_diff < min_angle_dif:
                best_vertex = near_vertex

        # Get the sequence of moves
        move_path = []
        curr_vertex = best_vertex
        while curr_vertex.parent_move is not None:
            move_path.append(curr_vertex.parent_move)
            curr_vertex = curr_vertex.parent
        move_path.reverse()
        return move_path

    # Returns a solution in the form of the indices of the controls and the durations
    # In the format for the PlanarTrajectory
    def get_solution(self, goal):
        path = self.get_path(goal)

        # Used to help keep track of the durations
        index_dict = dict()
        for i in range(len(planar_trajectory.controls_rs)):
            index_dict[tuple(planar_trajectory.controls_rs[i])] = i

        # Initialize lists
        control_sequence = [index_dict[tuple(path[0])]]
        durations = [1]
        curr_index = 0
        tuple_path = [tuple(path[i]) for i in range(len(path))]

        # Construct the control sequence and durations
        for i in range(1, len(tuple_path)):
            if index_dict[tuple_path[i]] == control_sequence[curr_index]:
                durations[curr_index] += 1
            else:
                control_sequence.append(index_dict[tuple_path[i]])
                durations.append(1)
                curr_index += 1

        return control_sequence, durations


if __name__ == "__main__":
    car_w = CarWorkspace([])
    #tree = RRT(car_w, (0, 0, 0), -20, -20, 20, 20)

    # g = (8.5, 8.5, 2.5)
    # print(tree.get_path(g))
    # print(tree.get_solution(g))


# poly = Polygon(((0, 0), (0, 1), (1, 1), (1, 0)))
# point = Point(2, .2)
#
# print(poly)
# print(poly.contains(point))
