import math
import numpy as np
import os
import time
from functools import partial
from multiprocessing import Pool
from scipy.optimize import Bounds, minimize, minimize_scalar
from scipy.interpolate import splev, splprep
from track import Track
from utils import define_corners, idx_modulo
from velocity import VelocityProfile
from matplotlib import pyplot as plt

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern

from track import Track
from vehicle import Vehicle
from path import Path
from plot import plot_path

class Trajectory:
    """
    Stores the geometry and dynamics of a path, handling optimisation of the
    racing line. Samples are taken every metre.
    """

    def __init__(self, track: 'Track', vehicle: 'Vehicle'):
        """Store track and vehicle and initialise a centerline path."""
        self.track = track
        self.ns = math.ceil(track.length)
        self.update(np.full(track.size, 0.5))
        self.vehicle = vehicle
        self.velocity = None
        
        # for Bayesian
        self.length = track.length
        self.mid_waipoints = track.mid_controls
        self.widths = track.widths
        self.mid_controls_decongested = track.mid_controls_decongested
        self.widths_decongested = track.widths_decongested

    def update(self, alphas):
        """Update control points and the resulting path."""
        self.alphas = alphas
        self.path = Path(self.track.control_points(alphas), self.track.closed)
        # Sample every metre
        self.s = np.linspace(0, self.path.length, self.ns)

    def update_velocity(self):
        """Generate a new velocity profile for the current path."""
        s = self.s[:-1]
        s_max = self.path.length if self.track.closed else None
        k = self.path.curvature(s)
        self.velocity = VelocityProfile(self.vehicle, s, k, s_max)

    def lap_time(self):
        """Calculate lap time from the velocity profile."""
        # print(self.s)
        time_sum = np.sum(np.diff(self.s) / self.velocity.v)
        print(time_sum)
        return time_sum
    
    ##################################################
    # for Bayesian optimisation
    def update_Bayesian(self, waypoints):
        """Update control points and the resulting path."""
        
        print(f"update Bayesian: ")
        self.path = Path(waypoints, self.track.closed)
        # Sample every metre
        self.s = np.linspace(0, self.path.length, self.ns)
        
        """Generate a new velocity profile for the current path."""
        s = self.s[:-1]
        s_max = self.path.length if self.track.closed else None
        k = self.path.curvature(s)
        self.velocity = VelocityProfile(self.vehicle, s, k, s_max)
    
    
    def calcMinTime(self, waypoints):
        """Minimum time to traverse on a fixed trajectory for Bayesian optimisation."""
        # Method for Bayesian optimization
        
        # Fit cubic splines on the waypoints
        # waypoints are values in between the cones
        tck, u = splprep([waypoints[0], waypoints[1]])
        
        # Re-sample waypoints with finer discretization
        u_fine = np.linspace(0, self.length, num=2*len(waypoints[0]))
        x_fine, y_fine = splev(u_fine, tck)
        
        # Update the length of track and velocities
        self.update_Bayesian(waypoints)
        # Step 4: Return minimum time to traverse on (x_fine, y_fine)
        
        return self.lap_time()
    
    def move_xy_by_distance(self, spline, x, y, distance):
        """Get x y coordinates after moving wayponts in the direction normal to self.path_middle"""
        # tck is already the spline representation (t, c, k)
    
        # 1. Find the closest point on the spline to (x, y)
        def distance_to_point(t):
            xt, yt = splev(t, spline)
            return np.sqrt((xt - x) ** 2 + (yt - y) ** 2)
        
        res = minimize_scalar(distance_to_point, bounds=(0, 1), method='bounded')
        t_closest = res.x
        
        # 2. Calculate the normal to the spline at this point
        dx, dy = splev(t_closest, spline, der=1)
        normal = np.array([-dy, dx])
        normal /= np.linalg.norm(normal)
        
        # 3. Move the point (x, y) by the given distance in the normal direction
        new_x = x + distance * normal[0]
        new_y = y + distance * normal[1]
        
        return new_x, new_y

    def Bayesian(self):
        """Racing line using Bayesian optimization."""
        # Initialization
        waypoints_list = []
        lap_times = []
        
        x = self.mid_controls_decongested[0] #self.mid_waipoints[0]
        y = self.mid_controls_decongested[1] #self.mid_waipoints[1]
        widths = self.widths_decongested #self.widths
        
        # print(self.widths)
        # print(len(self.widths), len(self.mid_waipoints[0]))
        
        
        for j in range(10):
            """Randomly sample a new trajectory parametrized by waypoints"""
            new_x_list = []
            new_y_list = []
            #the bigger s, the more smooth spline is, 0 means spline goes through every point
            tck, u = splprep([x, y], s=0)
            # print(len((self.mid_waipoints[0])))
            # print(len(self.widths))
            for i in range (len(x)):
                # random distance to move track in the normal direction
                distance = np.random.uniform(-0.99,0.99) * widths[i] / 2
                
                x_new, y_new = self.move_xy_by_distance(tck, x[i], y[i], distance)
                new_x_list.append(x_new)
                new_y_list.append(y_new)
                
            plt.scatter(x, y, label='Dane oryginalne')
            plt.scatter(new_x_list, new_y_list, label='po przesunieciu')
            plt.legend()
            # plt.show()
            plt.savefig(f'img/mid_after_decongested_{j}.png')
            
            # Step 4: Compute min time to traverse using Algorithm 1
            lap_time = self.calcMinTime((new_x_list, new_y_list))
            lap_times.append(lap_time)
            waypoints_list.append([new_x_list, new_y_list])
           
            
        # Step 6: Initialize training data
        D = list(zip(waypoints_list, lap_times))
        
        # Step 7: Learn a GP model τ ∼ GP(w)
        X_train = np.array([w.flatten() for w in waypoints_list])
        y_train = np.array(lap_times)
        kernel = Matern(nu=2.5)
        gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)
        gp.fit(X_train, y_train)
        
        # Bayesian Optimization loop
        converged = False
        while not converged:
            # Step 11: Determine candidate trajectory w?
            w_candidates = np.random.rand(10, 2, self.track.size)
            X_candidates = np.array([w.flatten() for w in w_candidates])
            y_candidates, _ = gp.predict(X_candidates, return_std=True)
            w_best = w_candidates[np.argmin(y_candidates)]
            
            # Step 12: Compute min time to traverse τ? using Algorithm 1
            tau_best = self.calcMinTime(w_best)
            
            # Step 13: Add new sample to training data
            D.append((w_best, tau_best))
            X_train = np.array([w.flatten() for w, _ in D])
            y_train = np.array([t for _, t in D])
            
            # Step 14: Update the GP model using D
            gp.fit(X_train, y_train)
            
            # Check for convergence
            if len(lap_times) > 10 and np.std(lap_times[-10:]) < 1e-3:
                converged = True

        # Step 16: Return w* and corresponding waypoints (xi, yi)
        return w_best, w_best[0], w_best[1]
    ##################################################
    
    def minimise_curvature(self):
        """Generate a path minimising curvature."""

        def objfun(alphas):
            self.update(alphas)
            return self.path.gamma2(self.s)

        t0 = time.time()
        res = minimize(
            fun=objfun,
            x0=np.full(self.track.size, 0.5),
            method='L-BFGS-B',
            bounds=Bounds(0.0, 1.0)
        )
        self.update(res.x)
        return time.time() - t0

    def minimise_compromise(self, eps):
        """
        Generate a path minimising a compromise between path curvature and path
        length. eps gives the weight for path length.
        """

        def objfun(alphas):
            self.update(alphas)
            k = self.path.gamma2(self.s)
            d = self.path.length
            return (1-eps)*k + eps*d

        t0 = time.time()
        res = minimize(
            fun=objfun,
            x0=np.full(self.track.size, 0.5),
            method='L-BFGS-B',
            bounds=Bounds(0.0, 1.0)
        )
        self.update(res.x)
        return time.time() - t0

    def minimise_optimal_compromise(self, eps_min=0, eps_max=0.2):
        """
        Determine the optimal compromise weight when using optimise_compromise to
        produce a path.
        """

        def objfun(eps):
            self.minimise_compromise(eps)
            self.update_velocity()
            t = self.lap_time()
            if self.epsilon_history.size > 0:
                self.epsilon_history = np.vstack(
                    (self.epsilon_history, [eps, t]))
            else:
                self.epsilon_history = np.array([eps, t])
            return t

        self.epsilon_history = np.array([])
        t0 = time.time()
        res = minimize_scalar(
            fun=objfun,
            method='bounded',
            bounds=(eps_min, eps_max)
        )
        self.epsilon = res.x
        self.minimise_compromise(self.epsilon)
        end = time.time()
        return end - t0

    def minimise_lap_time(self):
        """
        Generate a path that directly minimises lap time.
        """

        def objfun(alphas):
            self.update(alphas)
            self.update_velocity()
            return self.lap_time()

        t0 = time.time()
        res = minimize(
            fun=objfun,
            x0=np.full(self.track.size, 0.5),
            method='L-BFGS-B',
            bounds=Bounds(0.0, 1.0)
        )
        self.update(res.x)
        return time.time() - t0

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

###############################################################################


def optimise_sector_compromise(i, corners, traj):
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
    sector = Trajectory(
        Track(left=traj.track.left[:, idxs], right=traj.track.right[:, idxs]),
        traj.vehicle
    )

    # Optimise path through sector
    sector.minimise_optimal_compromise()

    # Weight alphas for merging across straights
    weights = np.ones((d-a) % n)
    weights[:(b-a) % n] = np.linspace(0, 1, (b-a) % n)
    weights[(c-a) % n:] = np.linspace(1, 0, (d-c) % n)
    alphas = np.zeros(n)
    alphas[idxs] = sector.alphas * weights

    # Report and plot sector results
    print("  Sector {:d}: eps={:.4f}, run time={:.2f}s".format(
        i, sector.epsilon, rt
    ))
    # plot_path(
    #   "./plots/" + traj.track.name + "_sector" + str(i) + ".png",
    #   sector.track.left, sector.track.right, sector.path.position(sector.s)
    # )

    return alphas
