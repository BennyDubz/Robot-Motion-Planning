import matplotlib.patches
import shapely
import matplotlib.pyplot as plt
import matplotlib.path as path
import numpy as np
import math
# Author: Ben Williams '25
# Date: November 10th, 2023


# A class defining the workspace of various robot joints and obstacles
class Workspace:
    # Parameter - Joints are a list of integers representing the length of the robot arm
    #   in sequential order of how the joints connect to one another
    # Parameter - Obstacles is an arbitrarily ordered list of shape objects from the shapely library
    def __init__(self, joints, obstacles):
        self.joints = joints
        self.obstacles = obstacles

    # Given a list of angles for each joint, calculate all the x, y coordinates for the arm locations
    def get_joints_coordinates(self, thetas):
        # We know the first joint starts at 0, 0, so we can calculate the first points coordinates easily
        locations = [(self.joints[0] * math.cos(thetas[0]), self.joints[0] * math.sin(thetas[0]))]

        # Calculate the rest of the joints coordinates
        for joint_index in range(1, len(self.joints)):
            cumulative_theta = 0
            # Joint locations also depend on previous angles
            for j in range(0, joint_index + 1):
                cumulative_theta += thetas[j]
                # We want to only consider the range [0, 2pi]
                cumulative_theta = cumulative_theta % (2 * math.pi)
            joint_x = locations[joint_index - 1][0] + self.joints[joint_index] * math.cos(cumulative_theta)
            joint_y = locations[joint_index - 1][1] + self.joints[joint_index] * math.sin(cumulative_theta)
            locations.append((joint_x, joint_y))

        return locations

    # Given a list of thetas, returns True if the robot joints would collide with a shape
    def is_collision_state(self, thetas):
        curr_locations = self.get_joints_coordinates(thetas)
        robot_arms = shapely.LineString(curr_locations)
        for obstacle in self.obstacles:
            if robot_arms.intersects(obstacle):
                return True
        return False

    # Shows the current position of the workspace
    def graphics_workspace(self, thetas):
        line_locations = [(0, 0)] + self.get_joints_coordinates(thetas)
        x_coords = [line_locations[i][0] for i in range(len(line_locations))]
        y_coords = [line_locations[i][1] for i in range(len(line_locations))]
        fig, ax = plt.subplots()
        plt.plot(x_coords, y_coords, color="red", marker="o")
        # The maximum extent the robot system could reach
        max_extention = sum(self.joints)
        plt.xlim(-max_extention, max_extention)
        plt.ylim(-max_extention, max_extention)

        for shape in self.obstacles:
            shape_x, shape_y = shape.exterior.xy
            vertex_list = np.array([(shape_x[i], shape_y[i]) for i in range(len(shape_x))])
            shape_path = path.Path(vertex_list)
            patch = matplotlib.patches.PathPatch(shape_path)
            ax.add_patch(patch)
        plt.show()


if __name__ == "__main__":
    robot_arms = [4, 3, 2]
    box = shapely.geometry.box(0, 2, 2, 4)
    poly_points = [[0, 4], [3, 5], [3, 7], [2, 6], [0, 4]]
    poly = shapely.geometry.Polygon(poly_points)
    angles = [0.75, 0.35, 0.75]
    wk = Workspace(robot_arms, [box, poly])
    wk.graphics_workspace(angles)
    print(wk.is_collision_state(angles))

