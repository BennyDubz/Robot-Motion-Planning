from ProbabilisticRoadmap import ProbabilisticRoadmap
from Workspace import Workspace
import shapely
import numpy as np

# Author: Ben Williams '25
# Date: November 11th, 2023

# Uncomment sections of code to test different maps

########################################
# No obstacles
########################################
# two_R_arm = [5, 3]
# no_obstacle_2R = Workspace(two_R_arm, [])
#
# prm_no_obstacle_2R = ProbabilisticRoadmap(no_obstacle_2R, 200)
# path = prm_no_obstacle_2R.find_path_bfs((0.75, 0.5), (0.6, 0.4))
# for thetas in path:
#     no_obstacle_2R.graphics_workspace(thetas)

########################################
# One obstacle
########################################
# two_R_arm = [5, 3]
# box = shapely.geometry.box(-1, 5.5, 2, 6.5)
# one_obstacle_2R = Workspace(two_R_arm, [box])
# prm_one_ob_2R = ProbabilisticRoadmap(one_obstacle_2R, 200)
# path = prm_one_ob_2R.find_path_bfs((0.75, 0.5), (1.6, 2.4))
# for thetas in path:
#     one_obstacle_2R.graphics_workspace(thetas)

########################################
# Three R Arm, four boxes
########################################
# three_R_arm = [4, 2, 3]
# box1 = shapely.geometry.box(-1, 5.5, 2, 6.5)
# box2 = shapely.geometry.box(-1, -6.5, 2, -5.5)
# box3 = shapely.geometry.box(5.5, -1, 6.5, 2)
# box4 = shapely.geometry.box(-6.5, -1, -5.5, 2)
#
# four_box_3R = Workspace(three_R_arm, [box1, box2, box3, box4])
# prm_four_box_3R = ProbabilisticRoadmap(four_box_3R, 1000)
# four_box_3R.graphics_workspace((4, 1, 4))
# path = prm_four_box_3R.find_path_bfs((0.75, 0.5, 5), (4, 1, 4))
# for thetas in path:
#     four_box_3R.graphics_workspace(thetas)

########################################
# Four R Arm, many obstacles
########################################
four_R_arm = [3, 2, 2, 3]
hex_coords = np.array([[-5, -5], [-4, -4], [-3, -4], [-2, -5], [-2, -6], [-3, -7], [-4, -7], [-5, -6], [-5, -5]])
hexagon = shapely.geometry.Polygon(hex_coords)
circle1 = shapely.geometry.Point(5, 5).buffer(2)
circle2 = shapely.geometry.Point(-5, 0).buffer(1)
rectangle = shapely.geometry.box(2, -6, 3.5, -1)
thin_rectangle = shapely.geometry.box(-2, 4, -1.75, 7)
complex_workspace = Workspace(four_R_arm, [hexagon, circle1, circle2, rectangle, thin_rectangle])
complex_prm = ProbabilisticRoadmap(complex_workspace, 2000)
path = complex_prm.find_path_bfs((1, 5, 6, 6), (3, 5, 1, 1))
for thetas in path:
    complex_workspace.graphics_workspace(thetas)



