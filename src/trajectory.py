import math
import numpy as np
import os
import time
from multiprocessing import Pool
from functools import partial
from multiprocessing import Pool
from scipy.optimize import Bounds, minimize, minimize_scalar
from scipy.interpolate import splev, splprep
from scipy.stats import norm

from track import Track
from utils import define_corners, idx_modulo
from velocity import VelocityProfile
from matplotlib import pyplot as plt

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern
from sklearn.preprocessing import StandardScaler

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
        self.direction_vector = track.diffs
        self.length = track.length
        self.mid_waipoints = track.mid_controls
        self.widths = track.widths
        self.mid_controls_decongested = track.mid_controls_decongested
        self.widths_decongested = track.widths_decongested
        self.best = []
        self.sigma = []
        # end for Bayesian

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
        return time_sum
    
    ##################################################
    # for Bayesian optimisation
    def updateAlphas(self, alphas):
        path = Path(self.track.control_points_bayesian(alphas), self.track.closed)
        
        controls = path.controls
        length = path.length
        spline = path.spline
        return controls
   
    
    def calcMinTime(self, controls):
        """Minimum time to traverse on a fixed trajectory for Bayesian optimisation."""
        """Update control points and the resulting path."""
        
        self.path = Path(controls, self.track.closed)
        # Sample every metre
        self.s = np.linspace(0, self.path.length, self.ns)
        
        """Generate a new velocity profile for the current path."""
        s = self.s[:-1]
        s_max = self.path.length if self.track.closed else None
        k = self.path.curvature(s)
        self.velocity = VelocityProfile(self.vehicle, s, k, s_max)

        # Step 4: Return minimum time to traverse on (x_fine, y_fine)
        
        return self.lap_time()
    
    def move_xy_by_distance(self, x, y, distances):
        """Get x y coordinates after moving wayponts in the direction normal to self.path_middle"""
        
        new_x_a = []; new_y_a = []
        
        for i, d in enumerate(distances):
            new_x = x[i] + distances[i] * self.direction_vector[0][i]
            new_y = y[i] + distances[i] * self.direction_vector[1][i]
            
            new_x_a.append(new_x); new_y_a.append(new_y)
            
            
        # print(new_x_a, new_y_a)
        return new_x_a, new_y_a


    def expected_improvement(self, x, gp : GaussianProcessRegressor, tau_best, n_params):
        true_tau = self.calcMinTime(self.updateAlphas(x))
        x = x.reshape(-1, n_params)
        tau_mean, sigma = gp.predict(x, return_std=True)
        with np.errstate(divide='warn'):
            imp_est = tau_best - tau_mean[0]
            imp = tau_best - true_tau
            # print(f"Estimated tau {tau_mean[0]} \t true tau {true_tau}")
            # print(f"Estimated improvement {imp_est} \t true improvement {imp} \t improvement delta {imp_est - imp}")
            sigma = sigma[0]
            #Z = imp / sigma
            #ei = imp * norm.cdf(Z) + sigma * norm.pdf(Z)
            ei = max(0, imp)
            if sigma == 0.0: ei = 0.0
            #print(f"Expected improvement {ei} type {type(ei)}")
            self.sigma.append(sigma)
        return ei

    def find_best_alpha(self, gp, tau_best, x0, bounds, n_params):
        
        # scaler = StandardScaler()
        # x0 = np.array(x0)
        # x0_scaled = scaler.fit_transform(x0.reshape(-1, 1)).flatten()
        # # bounds_scaled = scaler.transform(bounds)
        # bounds_scaled = np.zeros_like(bounds)
        # for i in range(bounds.shape[0]):
        #     bounds_scaled[i] = scaler.fit_transform(bounds[i].reshape(-1, 1)).flatten()
        
        def min_obj(X):
            return -(self.expected_improvement(X, gp, tau_best, n_params))

        # x0 = [np.random.uniform(low=0, high=0.99) for _ in x0]
        res = minimize(min_obj, x0=x0, bounds=bounds, method='COBYLA',options={'maxiter': 10000, 'disp': False}) # methods: 'COBYLA' 'SLSQP' 'L-BFGS-B'
        return res.x #scaler.inverse_transform(res.x.reshape(-1, 1)).flatten() #

    def Bayesian(self):
        """Racing line using Bayesian optimization."""
        # Initialization
        alphas_list = [] # distances used for learning gp model
        lap_times = []
        
        x_mid = self.mid_controls_decongested[0] #self.mid_waipoints[0]
        y_mid = self.mid_controls_decongested[1] #self.mid_waipoints[1]
        widths = [w/2 for w in self.widths_decongested] #self.widths
        # print(f"ilosc punktow do spline: {len(widths)} szerokości na trasie: {widths}")
        n = len(x_mid)-1
        
        # print(self.widths)
        # print(len(self.widths), len(self.mid_waipoints[0]))
        
        t0 = time.time()
        for j in range(10):
            """Randomly sample a new trajectory parametrized by waypoints"""
            new_x_list = []; new_y_list = []
            distance_w_list = []
                           
            # random distance list to move track in the normal direction
            alphas = [np.random.uniform(0,0.99) for _ in range(len(x_mid)-1)]
            
            print("zew petla iteracja nr: ", j)    
            
            (new_x_list, new_y_list) = self.updateAlphas(alphas)
                
            # plt.figure(); plt.scatter(x_mid, y_mid, label='Dane oryginalne')
            # plt.scatter(new_x_list, new_y_list, label='po przesunieciu')
            # plt.legend(); plt.savefig(f'img/mid_after_decongested_{j}.png')
            
            # Step 4: Compute min time to traverse using Algorithm 1
            lap_time = self.calcMinTime((new_x_list, new_y_list))
            lap_times.append(lap_time)
            alphas_list.append(alphas)
           
            
        # Step 6: Initialize training data
        D = list(zip(alphas_list, lap_times))
        
        # Step 7: Learn a GP model τ ∼ GP(w)
        X_train = np.array([np.ravel(w) for w in alphas_list])
        y_train = np.array(lap_times)
        kernel = Matern(nu=2.5) #TODO co to???
        gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)
        gp.fit(X_train, y_train)
        
        # Bayesian Optimization loop
        converged = False
        while not converged:
            # Step 11: Determine candidate trajectory w*
            tau_best = np.min(y_train)
            alpha_best = alphas_list[np.argmin(y_train)]
            
            bounds = np.array([[0.0, 0.99] for _ in  alphas])
            
            w_star = self.find_best_alpha(gp, tau_best, alpha_best, bounds, n)
            alphas_list.append(w_star)
            
            
            # Step 12: Compute min time to traverse τ* using Algorithm 1
            # Also make sure we are in global x,y coordinates, not in widths coordinates
            x_star_list = []; y_star_list = []
            x_star_list, y_star_list = self.updateAlphas(w_star)
            tau_star = self.calcMinTime((x_star_list, y_star_list))
            
            print(f"tau best: {tau_best} tau star: {tau_star}")
            
            # Step 13: Add new sample to training data
            D.append((w_star, tau_star))
            X_train = np.array([np.ravel(w) for w, _ in D])
            y_train = np.array([t for _, t in D])
            
            
            # Step 14: Update the GP model using D
            gp.fit(X_train, y_train)
            
            # Check for convergence
            if len(y_train) > 20 and np.std(self.sigma[-10:]) < 1e-3:
                
                alpha_best = alphas_list[np.argmin(y_train)]
                print(f"czasy trajektorii: {y_train}")
                
                converged = True
                
        x_star_list = []; y_star_list = []
        x_star_list, y_star_list = self.updateAlphas(alpha_best)
            
            
        self.best = (x_star_list, y_star_list)

        return time.time() - t0
    
    def optimize_COBYLA(self, args):
        tau0, alpha0 = args
        bounds = np.array([[0.0, 0.99] for _ in  alpha0])
        
        def expected_improvement(x):
            tau = self.calcMinTime(self.updateAlphas(x))
            with np.errstate(divide='warn'):
                imp = tau0 - tau
                ei = max(0, imp)
            return -ei
        
        # methods: 'COBYLA' 'SLSQP' 'L-BFGS-B'
        res = minimize(expected_improvement, x0=alpha0, bounds=bounds, method='COBYLA',options={'maxiter': 2000, 'disp': False})
        
        w_star = res.x
        x_star_list = []; y_star_list = []
        x_star_list, y_star_list = self.updateAlphas(w_star)
        tau_star = self.calcMinTime((x_star_list, y_star_list))

        print(f"Initial: {tau0} Optimized: {tau_star}")
        return (tau_star, w_star)
    
    
    def Nonlinear(self):
        """Racing line using Bayesian optimization."""
        # Initialization
        alphas_list = [] # distances used for learning gp model
        lap_times = []
        
        x_mid = self.mid_controls_decongested[0] #self.mid_waipoints[0]
        
        t0 = time.time()
        for j in range(100):
            """Randomly sample a new trajectory parametrized by waypoints"""
            new_x_list = []; new_y_list = []
                           
            # random distance list to move track in the normal direction
            alphas = [np.random.uniform(0,0.99) for _ in range(len(x_mid)-1)]
                  
            (new_x_list, new_y_list) = self.updateAlphas(alphas)
                
            lap_time = self.calcMinTime((new_x_list, new_y_list))
            lap_times.append(lap_time)
            alphas_list.append(alphas)
           

        taus = np.array(lap_times)
        results = list(zip(taus, alphas_list))
        
        with Pool(processes=1) as p:
            args = sorted(results, key=lambda el: el[0])[0:10]
            rs = p.map(self.optimize_COBYLA, args)
            for res in rs:
                results.append(res)
        
        tau_best, alpha_best = sorted(results, key=lambda el: el[0])[0]
            
            
        x_star_list = []; y_star_list = []
        x_star_list, y_star_list = self.updateAlphas(alpha_best)
            
        print(f"Best is {tau_best}")
        self.best = (x_star_list, y_star_list)

        return time.time() - t0
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
