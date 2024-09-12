from optimizers.raceline_optimizer_base import RacelineOptimizer
from track import Track
from path import Path
from models.vehicle_base import VehicleBase
import time
import math
import os
import scipy.optimize as opt
import numpy as np
from functools import partial
from multiprocessing import Pool
from utils import define_corners, idx_modulo
from raceline_optimizer_base import RacelineOptimizer
from adaptive_compromise import AdaptiveCompromiseOptimizer

class DirectLaptimeMinimizationOptimizer(RacelineOptimizer):
    
    def __init__(self, track: Track, vehicle: VehicleBase):
        super().__init__(track, vehicle)
        self.ns = math.ceil(track.length)
        self.update_raceline_control_points(np.full(track.size, 0.5))


    def optimise_sectors(self, k_min, proximity, length):
        """
        Generate a path that optimises the path through each sector, and merges
        the results along intervening straights.
        """

        # Define sectors
        t0 = time.time()
        corners, _ = self.track.corners(self.s, k_min, proximity, length)

        # Optimise path for each sector in parallel
        nc = corners.shape[0]
        pool = Pool(os.cpu_count() - 1)
        alphas = pool.map(
            partial(optimise_sector_compromise, corners=corners, traj=self),
            range(nc)
        )
        pool.close()

        # Merge sectors and update trajectory
        alphas = np.sum(alphas, axis=0)
        self.update(alphas)
        return time.time() - t0
    
def optimise_sector_compromise(i, corners, traj: RacelineOptimizer):
    """
    Builds a new Track for the given corner sequence, and optimises the path
    through it by the compromise method.
    """

    # Represent sector as new Track
    nc = corners.shape[0]
    n = traj.track.size
    a = corners[(i-1) % nc, 1]  # Sector start
    b = corners[i, 0]         # Corner entry
    c = corners[i, 1]         # Corner exit
    d = corners[(i+1) % nc, 0]  # Sector end
    idxs = idx_modulo(a, d, n)
    sector = AdaptiveCompromiseOptimizer(
        Track(left=traj.track.left[:, idxs], right=traj.track.right[:, idxs]),
        traj.vehicle
    )

    # Optimise path through sector
    sector.optimize()

    # Weight alphas for merging across straights
    weights = np.ones((d-a) % n)
    weights[:(b-a) % n] = np.linspace(0, 1, (b-a) % n)
    weights[(c-a) % n:] = np.linspace(1, 0, (d-c) % n)
    alphas = np.zeros(n)
    alphas[idxs] = sector.alphas * weights

    # Report and plot sector results
    print("  Sector {:d}: eps={:.4f}".format(
        i, sector.epsilon
    ))
    # plot_path(
    #   "./plots/" + traj.track.name + "_sector" + str(i) + ".png",
    #   sector.track.left, sector.track.right, sector.path.position(sector.s)
    # )

    return alphas