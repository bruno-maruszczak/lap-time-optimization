from optimizers.raceline_optimizer_base import RacelineOptimizer
from track import Track
from path import Path
from models.vehicle_base import VehicleBase
import time
import math
import scipy.optimize as opt
import numpy as np

class CurvatureMinimizationOptimizer(RacelineOptimizer):
    
    def __init__(self, track: Track, vehicle: VehicleBase):
        super().__init__(track, vehicle)
        self.path = None
        self.alphas = None
        self.s = None
        self.ns = math.ceil(track.length)
        self.update_raceline_control_points(np.full(track.size, 0.5))


    def update_raceline_control_points(self, alphas):
        """Update control points and the resulting path."""
        self.alphas = alphas
        self.path = Path(self.track.control_points(alphas), self.track.closed)
        # Sample every metre
        self.s = np.linspace(0, self.path.length, self.ns)

    def optimize(self):
        """Generate a path minimising curvature."""

        def objfun(alphas):
            self.update_raceline_control_points(alphas)
            return self.path.gamma2(self.s)

        t0 = time.time()
        res = opt.minimize(
            fun=objfun,
            x0=np.full(self.track.size, 0.5),
            method='L-BFGS-B',
            bounds=opt.Bounds(0.0, 1.0)
        )
        self.update_raceline_control_points(res.x)
        return time.time() - t0