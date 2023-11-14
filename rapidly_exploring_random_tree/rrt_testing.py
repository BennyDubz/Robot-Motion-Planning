from RRT import RRT
from CarWorkspace import CarWorkspace
from planar_trajectory import PlanarTrajectory, controls_rs
from planar_display import animate_trajectory, save_movie
import shapely

##############################
# No obstacles
##############################
no_obstacles_car_workspace = CarWorkspace([])
start_empty = [0, 0, 0]
no_obstacles_RRT = RRT(no_obstacles_car_workspace, 1000, start_empty, -10, -10, 10, 10)
goal_empty = [3, -3, 1.5]
control_seq, durations = no_obstacles_RRT.get_solution(goal_empty)
no_ob_PT = PlanarTrajectory(controls_rs, start_empty[0], start_empty[1], start_empty[2], control_seq, durations)
print(len(no_obstacles_RRT.vertices))

anim = animate_trajectory(no_ob_PT, no_obstacles_car_workspace, -10, -10, 10, 10)
anim = save_movie(anim, "../animations/no_obstacles_animation.mp4")

##############################
# More complex map
##############################
box_1 = shapely.geometry.box(-3, -2, 3, 0)
box_2 = shapely.geometry.box(3, 2, 6, 4)
box_3 = shapely.geometry.box(-3, -6, 3, -4)
box_4 = shapely.geometry.box(-7, -6, -6, 6)

few_boxes_workspace = CarWorkspace([box_1, box_2, box_3, box_4])
start_few_boxes = [2, 2, 4]
goal_few_boxes = [-8, 0, 4.5]
# few_boxes_RRT = RRT(few_boxes_workspace, 20000, start_few_boxes, -10, -10, 10, 10)
few_boxes_RRT = RRT(few_boxes_workspace, 20000, start_few_boxes, -10, -10, 10, 10, goal_few_boxes)
print("Num vertices in RRT: ", len(few_boxes_RRT.vertices))
control_seq, durations = few_boxes_RRT.get_solution(goal_few_boxes)

few_box_PT = PlanarTrajectory(controls_rs, start_few_boxes[0], start_few_boxes[1], start_few_boxes[2], control_seq,
                              durations)

print("Car final config", few_box_PT.config_at_t(sum(durations) - 0.01))

anim = animate_trajectory(few_box_PT, few_boxes_workspace, -10, -10, 10, 10)
save_movie(anim, "../animations/few_boxes_animation1.mp4")

