import json
import numpy as np
import matplotlib.pyplot as plt
import os
from scipy.interpolate import splprep, splev

class Track:
    def __init__(self, vehicle_name, track_name, method_name):
        cwd = os.getcwd()
        base_path = os.path.join(cwd, "data", "plots", vehicle_name, track_name, method_name)

        self.left_bound_x, self.left_bound_y = self.load_path_from_json(os.path.join(base_path, "left.json"))
        self.right_bound_x, self.right_bound_y = self.load_path_from_json(os.path.join(base_path, "right.json"))
        self.path_x, self.path_y = self.load_path_from_json(os.path.join(base_path, "path.json"))

        self.left_bound = self.create_spline(self.left_bound_x, self.left_bound_y) # left bound spline
        self.right_bound = self.create_spline(self.right_bound_x, self.right_bound_y) # right bound spline
        self.optimal_path = self.create_spline(self.path_x, self.path_y) # optimal path spline

        # line parameters:
        self.A = 1
        self.B = 1
        self.C = 1
        # point:
        self.point = (self.path_x[5], self.path_y[5])
        # self.plot()
        self.plotting_spline(self.optimal_path)
        self.plotting_spline(self.left_bound)
        self.plotting_spline(self.right_bound)
        plt.show()
        # self.get_tangent_to_spline(self.optimal_path, self.point)
    
    def load_path_from_json(self, filepath):
        """Load path data from a JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        name = data["name"]
        x = np.array(data["path"]["x"])
        y = np.array(data["path"]["y"])
        
        return x, y
    
    def create_spline(self, x, y):
        """Create a spline from given x and y coordinates."""
        tck, u = splprep([x, y], s=0)
        return tck

    def plot(self):
        plt.figure()
        plt.scatter(self.left_bound_x, self.left_bound_y, color='tab:blue', marker='.')
        plt.scatter(self.right_bound_x, self.right_bound_y, color='tab:orange', marker='.')
        plt.scatter(self.path_x, self.path_y, color='tab:green', marker='.')
        plt.show()

    def plotting_spline(self, spline):
        """Plot the spline."""
        u_fine = np.linspace(0, 1, 1000)
        x_fine, y_fine = splev(u_fine, spline)
        plt.plot(x_fine, y_fine, 'tab:red')
        # plt.show()

    def get_tangent_to_spline(self, spline, point):
        """Get the tangent to the spline at a given point."""
        x, y = point
        u = np.linspace(0, 1, 1000)
        tangent_x, tangent_y = splev(u, spline, der=1)
        # print(tangent)
        return (tangent_x, tangent_y)
    
    def plot_tangent(self, point, tangent):
        """Plot the tangent at a given point."""
        x, y = point
        dx, dy = tangent
        plt.quiver(x, y, dx, dy, color='tab:purple')
        # plt.show()
    
    def plot_line_and_tangent(self, point, A, B, C, A_tangent, B_tangent, C_tangent):
        pass
    
    def find_dist_to_bound(self, s, side : str="left"):


        def tangent_line(x0, y0, dx, dy):
            """
            Finds a line tangent to a curve at a given point
            
            Parameters:
            x0, y0 - the given point
            dx, dy - derivatives of a curve in respect to the parameter u

            Returns:
            - a tuple (A, B, C) in general line form: Ax + By + C = 0
            """
            if dx == 0:
                return (0, 1, -x0)
            else:
                slope = dy/dx
                return (-1, slope, y0-slope*x0)
        
        def perpendicular_line(line : tuple, x0, y0):
            """
            Finds a line perpendicular to a line at a given point 
            
            Parameters:
            x0, y0 - the given point
            line - a tuple (A, B, C) in general line form: Ax + By + C = 0

            Returns:
            - a tuple (A, B, C) in general line form: Ax + By + C = 0 of the perpendicular line
            """
            A, B, C = line
            assert A != 0 or B != 0

            if A == 0:
                perp_line = (0, 1, -y0)
            elif B == 0:
                perp_line = (1, 0, -x0)
            else:
                slope = B / A
                perp_line = (-slope, 1, -y0 + slope*x0)
            return perp_line
            
        assert isinstance(s, (int, float)) # s should be a number, not a list, tuple, etc..
        assert side in ["left", "right"]

        u = self.optimal_path.find_u_given_s(s)
        
        # Find x, y, dx, dy of the spline
        tck = self.optimal_path.spline
        x0, y0 = splev(u, tck)
        dx, dy = splev(u, tck, der=1)
        
        # find perpendicular_line
        tangent = tangent_line(x0, y0, dx, dy)
        perpendicular_line = (tangent, x0, y0)
        
        bound = self.left_bound if side == "left" else self.right_bound

        A, B, C = perpendicular_line
        denom = np.sqrt(A**2 + B**2)
    

        # Sort the points on bound by distance to the line
        # points = bound.get_sample_points()
        # points.sort(key = lambda())
        # for x, y in bound.get_sample_points():
        #     dist = abs(A * x + B * y + C) / denom



    def find_intersections(self, point, A, B, C, bound):
        """
        Find the two closest intersection points of the bound spline with the line Ax + By + C = 0.

        Parameters:
        point - the point on the optimal path
        A, B, C - coefficients of the line equation
        bound - the spline representing the bound (left or right)

        Returns:
        Two closest intersection points as tuples (x, y)
        """
        x0, y0 = point
        u_fine = np.linspace(0, 1, 1000)
        x_fine, y_fine = splev(u_fine, bound)

        distances = np.abs(A * x_fine + B * y_fine + C) / np.sqrt(A**2 + B**2)
        sorted_indices = np.argsort(distances)

        # Get the two closest points
        closest_points = [(x_fine[i], y_fine[i]) for i in sorted_indices[:2]]

        closest_point = min(closest_points, key=lambda p: np.hypot(p[0] - x0, p[1] - y0))
        distance = np.hypot(closest_point[0] - x0, closest_point[1] - y0)

        return distance


track = Track("Mazda MX-5", "buckmore", "curvature")