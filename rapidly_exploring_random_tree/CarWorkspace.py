import shapely

# Author: Ben Williams '25
# Date: November 13th, 2023


class CarWorkspace:
    # Takes shapely shapes as obstacles
    def __init__(self, obstacles):
        self.obstacles = obstacles

    # car is CarShapes object
    def is_car_collision(self, car):
        # Get all the cars actual x, y coordinates
        car_points = car.compute_transformed()
        # Add the first point again to make a complete polygon
        car_poly = shapely.geometry.Polygon(car_points + car_points[0])
        for obstacle in self.obstacles:
            if car_poly.intersects(obstacle):
                return True

        return False

    # Rather than take a car object, we just take a point and consider the car to be a circle
    # This will give false-positive collisions, but not false negatives
    def is_collision_simplified(self, x, y):
        car_approximation = shapely.geometry.Point(x, y).buffer(1.05)
        for obstacle in self.obstacles:
            if car_approximation.intersects(obstacle):
                return True

        return False





