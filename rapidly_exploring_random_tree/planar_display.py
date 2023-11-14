import shapely

from planar_trajectory import *
from planarsim import *
from matplotlib import patches as patches
from matplotlib import path as path
from matplotlib import pyplot as plt
from matplotlib import animation

from matplotlib.animation import FFMpegWriter

import numpy as np


class CarShapes:
    def __init__(self):
        # points in the local coordinate frame
        self.points = [[0, -.25], [.75, -.25], [1, 0], [.75, .25], [0, .25]]
        self.q = [0, 0, 0]    # initial configuration

        self.update_patch()

    def compute_transformed(self):
        T = transform_from_config(self.q)

        transformed_points = []

        for point in self.points:
            homog_point = [point[0], point[1], 1]
            transformed_point = T @ homog_point
            tp = [transformed_point[0], transformed_point[1]]
            transformed_points.append(tp)

        return transformed_points


    def set_configuration(self, q):
        self.q = q
        self.update_patch()

    def update_patch(self):
        transformed_points = self.compute_transformed()
        self.car_poly = plt.Polygon(transformed_points, edgecolor='gray', facecolor="lightgray")

    def draw(self, ax):
        ax.add_patch(self.car_poly)


class TrajectoryView:
    def __init__(self, traj):
        self.traj = traj


    def draw(self, ax, final_t):

        car = CarShapes()
        car.set_configuration(self.traj.config_at_t(final_t))
        car.draw(ax)

        traj_x, traj_y, traj_theta = self.traj.linspace(.03, 0, final_t)

        ax.plot(traj_x, traj_y, color='black')


def draw1():

    traj = PlanarTrajectory(controls_rs, 3, 0, .5, [0, 3, 5, 3], [1.0, 2.0, 2.0, 2.0])

    tview = TrajectoryView(traj)

    ax = plt.axes()

    tview.draw(ax, 6.99)


def animate1():

    time_step = .05
    traj = PlanarTrajectory(controls_rs, 0, 0, .5, [0, 3, 5, 3], [1.0, 2.0, 2.0, 2.0])

    fig = plt.figure(figsize=(8, 4.5))
    fig.tight_layout()

    ax = plt.axes()

    def animate(i):
        ax.clear()
        ax.set_xlim(-1, 10)
        ax.set_ylim(-3, 3)
        ax.set_aspect('equal')

        tview = TrajectoryView(traj)
        tview.draw(ax, i * .05)
        #print(f"hello: {i}")

    num_frames = traj.end_time / time_step
    anim = animation.FuncAnimation(fig, animate, frames=140, interval=20, repeat=False)

    plt.show()


def animate_trajectory(traj, car_workspace, min_x, min_y, max_x, max_y):
    time_step = 0.05

    fig = plt.figure(figsize=(8, 4.5))
    fig.tight_layout()

    ax = plt.axes()

    def animate(i):
        ax.clear()
        # Draw shapes
        for shape in car_workspace.obstacles:
            shape_x, shape_y = shape.exterior.xy
            vertex_list = np.array([(shape_x[i], shape_y[i]) for i in range(len(shape_x))])
            shape_path = path.Path(vertex_list)
            patch = patches.PathPatch(shape_path)
            ax.add_patch(patch)

        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_aspect('equal')

        tview = TrajectoryView(traj)
        tview.draw(ax, i * .05)

    num_frames = int(traj.end_time / time_step)
    anim = animation.FuncAnimation(fig, animate, frames=num_frames, interval=20, repeat=False)

    plt.show()
    return anim


def draw_rcs(ax, controls, q):
    rcs = compute_worldframe_rcs(controls, q)
    #print(rcs)
    for rc in rcs:
        if(rc[2] > 0):
            circle= plt.Circle((rc[0], rc[1]), radius=.1)
            ax.add_patch(circle)


def save_movie(anim, filename):
    writer = FFMpegWriter(fps=30, metadata=dict(artist='Ben Williams'), bitrate=1800)
    anim.save(filename, writer=writer)


if __name__ == '__main__':

    # standard size plot with 16:9 ratio to match youtube, do
    #  not change
    fig = plt.figure(figsize=(8, 4.5))
    fig.tight_layout()

    # draw1()
    animate1()

    plt.show()
