from optimizers.raceline_optimizer_base import RacelineOptimizer
from track import Track
from path import Path
from models.vehicle_base import VehicleBase
import time
import math
import scipy.optimize as opt
import numpy as np

class DirectLaptimeMinimizationOptimizer(RacelineOptimizer):
    
    def __init__(self, track: Track, vehicle: VehicleBase):
        super().__init__(track, vehicle)
        self.ns = math.ceil(track.length)
        self.update_raceline_control_points(np.full(track.size, 0.5))

    def optimize(self):
        """
        Generate a path that directly minimises lap time.
        """

        def objfun(alphas):
            self.update_raceline_control_points(alphas)
            return np.sum(np.diff(self.s) / self.velocity)

        t0 = time.time()
        res = opt.minimize(
            fun=objfun,
            x0=np.full(self.track.size, 0.5),
            method='L-BFGS-B',
            bounds=opt.Bounds(0.0, 1.0)
        )
        self.update_raceline_control_points(res.x)

        return time.time() - t0