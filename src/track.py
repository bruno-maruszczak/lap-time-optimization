import json
import numpy as np
from path import Path
from utils import define_corners, idx_modulo, is_closed


class Track:
    """Represents a track with boundaries defined by a series of cones."""

    def __init__(self, json_path=None, left=None, right=None):
        """Create track from cone coordinates."""
        if json_path is None:
            self.left = left
            self.right = right
        else:
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
        self.widths_decongested = []
        
        for i in range(0, len(self.mid.controls[0]), 5):
            self.mid_controls_decongested[0].append(self.mid.controls[0][i])
            self.mid_controls_decongested[1].append(self.mid.controls[1][i])
            self.widths_decongested.append(self.widths[i])
        #########################################################
    
    def read_cones(self, path):
        """Read cone coordinates from a JSON file."""
        track_data = json.load(open(path))
        self.name = track_data["name"]
        self.left = np.array([track_data["left"]["x"], track_data["left"]["y"]])
        self.right = np.array([track_data["right"]["x"], track_data["right"]["y"]])
        print("[ Imported {} ]".format(self.name))
    

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
