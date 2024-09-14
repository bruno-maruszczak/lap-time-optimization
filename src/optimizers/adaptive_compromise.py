from optimizers.raceline_optimizer_base import RacelineOptimizer
from track import Track
from path import Path
from models.vehicle_base import VehicleBase
import time
import math
import scipy.optimize as opt
import numpy as np
from optimizers.curvature_and_length_compromise import CurvatureAndLengthCompromiseOptimizer

class AdaptiveCompromiseOptimizer(RacelineOptimizer):
    
    def __init__(self, track: Track, vehicle: VehicleBase, eps_min = 0.0, eps_max = 0.2):
        super().__init__(track, vehicle)
        self.eps_min = eps_min
        self.eps_max = eps_max
        self.epsilon = None
        self.path = None
        self.alphas = None
        self.ns = math.ceil(track.length)
        self.update_raceline_control_points(np.full(track.size, 0.5))
    
    def solve(self):
        """
        Determine the optimal compromise weight to
        produce a path.
        """

        def objfun(eps):
            optimizer = CurvatureAndLengthCompromiseOptimizer(self.track, self.vehicle, eps)
            optimizer.solve()
            velocities = self.vehicle.get_velocity_profile(optimizer.path, optimizer.s)
            t = np.sum(np.diff(optimizer.s) / velocities)
            if self.epsilon_history.size > 0:
                self.epsilon_history = np.vstack(
                    (self.epsilon_history, [eps, t]))
            else:
                self.epsilon_history = np.array([eps, t])
            return t

        self.epsilon_history = np.array([])
        t0 = time.time()
        res = opt.minimize_scalar(
            fun=objfun,
            method='bounded',
            bounds=(self.eps_min, self.eps_max)
        )
        self.epsilon = res.x

        optimizer = CurvatureAndLengthCompromiseOptimizer(self.track, self.vehicle, self.epsilon)
        optimizer.solve()
        self.update_raceline_control_points(optimizer.alphas)

        end = time.time()
        return end - t0