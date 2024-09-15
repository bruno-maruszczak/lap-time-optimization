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

    def get_distance_to_track(self, A, B, C, edge):
        """
        Used to get the distance from vehicle trajectory to the edges of the track.
        Points are chosen as closest to a line tangent to the trajectory at a given point
        Returns the distance from a vehicle trajectory at given point
        """
        x, y = point

        # spline
        spline = self.mid.spline
    
track = Track("Mazda MX-5", "buckmore", "curvature")