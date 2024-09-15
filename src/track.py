import json
import numpy as np
from path import Path
from utils import define_corners, idx_modulo, is_closed
import matplotlib.pyplot as plt


class Track:
    """Represents a track with boundaries defined by a series of cones."""

    def __init__(self, json_path=None, left=None, right=None, track_width=None):
        """Create track from cone coordinates."""
        if json_path is None:
            self.left = left
            self.right = right
        else:
            if track_width > 1.0:
                track_width = 1.0
            elif track_width < 0.001:
                track_width = 0.001
            self.track_width = 1.0 - track_width
            self.read_cones(json_path)
        self.closed = is_closed(self.left, self.right)
        self.size = self.left[0].size - int(self.closed)
        self.diffs = self.right - self.left
        # print(f"left: {self.left[0][0]} \nright: {self.right[0][0]} \ndiffs: {self.diffs[0][0]}")
        self.mid = Path(self.control_points(np.full(self.size, 0.5)), self.closed)
        self.length = self.mid.dists[-1]
        
        ##########################################################
        # for Bayesian:
        self.widths = np.sqrt(self.diffs[0]**2 + self.diffs[1]**2)
        self.mid_controls = self.mid.controls
        # Taking every 5th control point
        self.mid_controls_decongested = [[], []]
        self.left_decongested = [[], []]
        self.widths_decongested = []
        self.diffs_decongested = [[], []]
        
        for i in range(0, len(self.mid.controls[0]), 3):
            self.mid_controls_decongested[0].append(self.mid.controls[0][i])
            self.mid_controls_decongested[1].append(self.mid.controls[1][i])
            self.diffs_decongested[0].append(self.diffs[0][i])
            self.diffs_decongested[1].append(self.diffs[1][i])
            self.left_decongested[0].append(self.left[0][i])
            self.left_decongested[1].append(self.left[1][i])
            self.widths_decongested.append(self.widths[i])
        self.diffs_decongested = np.array(self.diffs_decongested)
        self.left_decongested = np.array(self.left_decongested)
        #########################################################
    
    def read_cones(self, path):
        """Read cone coordinates from a JSON file."""
        track_data = json.load(open(path))
        self.name = track_data["name"]
        self.left = np.array([track_data["left"]["x"], track_data["left"]["y"]])
        self.right = np.array([track_data["right"]["x"], track_data["right"]["y"]])
        print("[ Imported {} ]".format(self.name))
        self.new_left = self.new_left_cones(self.left, self.right, self.track_width)
        self.new_right = self.new_right_cones(self.left, self.right, self.track_width)
        # plt.figure()
        # plt.scatter(self.left[0], self.left[1], c='r')
        # plt.scatter(self.right[0], self.right[1], c='g')
        # plt.scatter(self.new_left[0], self.new_left[1], c='b')
        # plt.scatter(self.new_right[0], self.new_right[1], c='y')
        # plt.savefig('plot.png')
        self.old_left = self.left
        self.old_right = self.right
        self.left = self.new_left
        self.right = self.new_right
    

    def avg_curvature(self, s):
        """Return the average of curvatures at the given sample distances."""
        k = self.mid.curvature(s)
        return np.sum(k) / s.size

    def corners(self, s, k_min, proximity, length):
        """Determine location of corners on this track."""
        return define_corners(self.mid, s, k_min, proximity, length)

    def control_points(self, alphas):
        """Translate alpha values to control point coordinates."""
        if self.closed:
            alphas = np.append(alphas, alphas[0])
        i = np.nonzero(alphas != -1)[0]
        return self.left[:, i] + (alphas[i] * self.diffs[:, i])
    
    def control_points_bayesian(self, alphas):
        """Translate alpha values to control point coordinates."""
        if self.closed:
            alphas = np.append(alphas, alphas[0])
        i = np.nonzero(alphas != -1)[0]
        return self.left_decongested[:, i] + (alphas[i] * self.diffs_decongested[:,i])
    
    def new_left_cones(self, old_left, old_right, track_width):
        """Return new left cones based on old left cones and track width parameter."""
        new_left = np.zeros((2, old_left[0].size))
        for i in range(old_left[0].size):
            left = old_left[:, i]
            right = old_right[:, i]
            diff = right - left
            # norm = np.sqrt(diff[0]**2 + diff[1]**2)
            new_diff = track_width * diff / 2
            new_left[:, i] = left + new_diff
        return new_left
    
    def new_right_cones(self, old_left, old_right, track_width):
        """Return new right cones based on old right cones and track width parameter."""
        new_right = np.zeros((2, old_right[0].size))
        for i in range(old_right[0].size):
            left = old_left[:, i]
            right = old_right[:, i]
            diff = left - right
            # norm = np.sqrt(diff[0]**2 + diff[1]**2)
            new_diff = track_width * diff / 2
            new_right[:, i] = right + new_diff
        return new_right

    
        