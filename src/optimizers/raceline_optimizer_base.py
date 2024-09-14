import math
import numpy as np
import os
import time
from multiprocessing import Pool

from scipy.optimize import Bounds, minimize, minimize_scalar
from scipy.interpolate import splev, splprep
from scipy.stats import norm

from track import Track


from matplotlib import pyplot as plt



from track import Track
from models.vehicle_base import VehicleBase
from path import Path
from plot import plot_path


class RacelineOptimizer:
    """
    Abstract class that stores the geometry and dynamics of a path, handling optimisation of the
    racing line.
    """

    def __init__(self, track: Track, vehicle: VehicleBase):
        """Store track and velocity profile of the vehicle and initialise a centerline path."""
        self.track = track
        self.vehicle = vehicle
        self.alphas = None
        self.s = None
        self.velocity = None
        self.path = None
        self.ns = None

    def update_raceline_control_points(self, alphas):
        """Update control points and the resulting path."""
        self.alphas = alphas
        self.path = Path(self.track.control_points(alphas), self.track.closed)
        self.s = np.linspace(0, self.path.length, self.ns)
    
    def update_raceline_velocity_profile(self):
        self.velocity = self.vehicle.get_velocity_profile(self.path, self.s)

    def lap_time(self) -> float:
        np.sum(np.diff(self.s) / self.velocity)

    def solve(self) -> float:
        pass
