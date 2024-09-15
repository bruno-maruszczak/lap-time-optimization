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

        self.plot()
        # and so on..
    
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
    
track = Track("Mazda MX-5", "buckmore", "curvature")