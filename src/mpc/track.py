import json
import numpy as np
import matplotlib.pyplot as plt
import os
import casadi as ca
from scipy.interpolate import splprep, splev

from path import ControllerReferencePath

class Track:
    def __init__(self, vehicle_name, track_name, method_name, n_samples):
        cwd = os.getcwd()
        base_path = os.path.join(cwd, "data", "plots", vehicle_name, track_name, method_name)

        self.left_bound_x, self.left_bound_y = self.load_path_from_json(os.path.join(base_path, "left.json"))
        self.right_bound_x, self.right_bound_y = self.load_path_from_json(os.path.join(base_path, "right.json"))
        self.path_x, self.path_y = self.load_path_from_json(os.path.join(base_path, "path.json"))
        self.widths = self.load_path_from_json(os.path.join(base_path, "widths.json"))

        self.n_samples = n_samples
        self.left_bound = ControllerReferencePath(np.array([self.left_bound_x, self.left_bound_y]), closed=True, n_samples=self.n_samples)
        self.right_bound = ControllerReferencePath(np.array([self.right_bound_x, self.right_bound_y]), closed=True, n_samples=self.n_samples)
        self.optimal_path = ControllerReferencePath(np.array([self.path_x, self.path_y]), closed=True, n_samples=self.n_samples)

        # Create distance lookup tables, for distance form bands
        self.bound_dist_table = {
            "left": self.create_distance_table(side="left"), 
            "right": self.create_distance_table(side="right")
        }

        self.plotting_spline(self.optimal_path)
        self.plotting_spline(self.left_bound)
        self.plotting_spline(self.right_bound)
        self.plot()
        #plt.show()
        # self.get_tangent_to_spline(self.optimal_path, self.point)
    
    def load_path_from_json(self, filepath):
        """Load path data from a JSON file."""
        with open(filepath, 'r') as f:
            data = json.load(f)
        
        name = data["name"]
        if name == "widths":
            return np.array(data["width"])
        else:
            x = np.array(data["path"]["x"])
            y = np.array(data["path"]["y"])
            return x, y
    
    def plot(self):
        plt.scatter(self.left_bound_x, self.left_bound_y, color='tab:blue', marker='.')
        plt.scatter(self.right_bound_x, self.right_bound_y, color='tab:orange', marker='.')
        plt.scatter(self.path_x, self.path_y, color='tab:green', marker='.')

    def plotting_spline(self, spline):
        """Plot the spline."""
        x_fine, y_fine = splev(spline.u_sampled, spline.spline)
        plt.plot(x_fine, y_fine, 'tab:red')

    def find_dist_to_band(self, s, side : str="left"):
        """
        Find distance between optimal trajectory and a given band (left/right)

        Parameters:
        s - the given point on a curve (as arc-length), if s is an casadi expression, returns an expression for distance.
        side - ["left", "right"] to choose the band

        Returns:
        - distance to a chosen band
        """
        def find_tangent_line(x0, y0, dx, dy):
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
        
        def find_perpendicular_line(line : tuple, x0, y0):
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


        assert side in ["left", "right"]

        if isinstance(s, ca.SX):
            y_values = self.bound_dist_table[side]
            x_values = self.optimal_path.arc_lengths_sampled
            expr = self.optimal_path.piecewise_linear_interpolation(s, x_values, y_values)
            return expr
        
        u = self.optimal_path.find_u_given_s(s)
        # Find x, y, dx, dy of the spline
        tck = self.optimal_path.spline
        x0, y0 = splev(u, tck)
        dx, dy = splev(u, tck, der=1)
        # find perpendicular_line
        tangent = find_tangent_line(x0, y0, dx, dy)
        perpendicular_line = find_perpendicular_line(tangent, x0, y0)
        A, B, C = perpendicular_line
        # Get sampled points on a given band
        bound = self.left_bound if side == "left" else self.right_bound
        x, y = bound.get_sample_points()

        # Sort points by the distance from line
        distances = np.abs(A * x + B * y + C) / np.sqrt(A**2 + B**2)
        sorted_indices = np.argsort(distances)

        # Get the two closest points
        closest_points = [(x[i], y[i]) for i in sorted_indices[:]]

        # Choose the one closer to the (x0, y0) point on the curve
        width = 10
        closest_point = None
        min_distance = float('inf')
        for point in closest_points:
            dist = np.hypot(point[0] - x0, point[1] - y0)
            if dist < min_distance and dist <= width: 
                min_distance = dist
                closest_point = point

        if closest_point is None:
            raise ValueError("No point found within the radius of {width}")

        distance = min_distance

        return distance

    def create_distance_table(self, side : str="left"):
        """
        Creates lookup tables for distance from optimal path to left and right band
        """
        assert side in ["left", "right"]

        return [self.find_dist_to_band(s, side=side) for s in self.optimal_path.arc_lengths_sampled]
