import math
import numpy as np
import time
from multiprocessing import Pool
from scipy.optimize import minimize
from scipy.interpolate import splev, splprep
from scipy.stats import norm

from track import Track
from utils import define_corners, idx_modulo

from matplotlib import pyplot as plt

from sklearn.gaussian_process import GaussianProcessRegressor
from sklearn.gaussian_process.kernels import Matern

from track import Track
from models.simple_gasoline_vehicle import SimpleGasolineVehicle
from path import Path

class TrajectoryBayesianNonlinear:
    """
    Stores the geometry and dynamics of a path, handling optimisation of the
    racing line. Samples are taken every metre. It's implemented for two optimisation methods, Bayesian and Nonlinear.
    """

    def __init__(self, track: 'Track', vehicle: SimpleGasolineVehicle):
        """Store track and vehicle and initialise a centerline path."""
        self.track = track
        self.ns = math.ceil(track.length)
        self.update(np.full(track.size, 0.5))
        self.vehicle = vehicle
        self.velocity = None
        
        self.direction_vector = track.diffs
        self.length = track.length
        self.mid_waipoints = track.mid_controls
        self.mid_controls_decongested = track.mid_controls_decongested
        self.widths_decongested = track.widths_decongested
        self.best = []
        self.sigma = []

    def update(self, alphas):
        """Update control points and the resulting path."""
        self.alphas = alphas
        self.path = Path(self.track.control_points(alphas), self.track.closed)
        # Sample every metre
        self.s = np.linspace(0, self.path.length, self.ns)

    def lap_time(self):
        """Calculate lap time from the velocity profile."""
        time_sum = np.sum(np.diff(self.s) / self.velocity.v)
        return time_sum
    
    ##################################################
    # for Bayesian optimisation
    def updateAlphas(self, alphas):
        path = Path(self.track.control_points_bayesian(alphas), self.track.closed)
        
        controls = path.controls
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

        # Return minimum time to traverse on (x_fine, y_fine)
        return self.lap_time()
    
    def move_xy_by_distance(self, x, y, distances):
        """Get x y coordinates after moving wayponts in the direction normal to self.path_middle"""
        new_x_a = []; new_y_a = []
        
        for i, d in enumerate(distances):
            new_x = x[i] + distances[i] * self.direction_vector[0][i]
            new_y = y[i] + distances[i] * self.direction_vector[1][i]
            new_x_a.append(new_x); new_y_a.append(new_y)
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
        n = len(x_mid)-1 # TODO: check if I use that anywhere
        
        # print(self.widths)
        # print(len(self.widths), len(self.mid_waipoints[0]))
        
        t0 = time.time()
        for j in range(10):
            """Randomly sample a new trajectory parametrized by waypoints"""
            new_x_list = []; new_y_list = []
            distance_w_list = []
                           
            # random distance list to move track in the normal direction
            alphas = [np.random.uniform(0,0.99) for _ in range(len(x_mid)-1)]
            (new_x_list, new_y_list) = self.updateAlphas(alphas)
                
            # plt.figure(); plt.scatter(x_mid, y_mid, label='Dane oryginalne')
            # plt.scatter(new_x_list, new_y_list, label='po przesunieciu')
            # plt.legend(); plt.savefig(f'img/mid_after_decongested_{j}.png')
            
            # Compute min time to traverse using Algorithm 1
            lap_time = self.calcMinTime((new_x_list, new_y_list))
            lap_times.append(lap_time)
            alphas_list.append(alphas)
           
            
        # Initialize training data
        D = list(zip(alphas_list, lap_times))
        
        # Learn a GP model τ ∼ GP(w)
        X_train = np.array([np.ravel(w) for w in alphas_list])
        y_train = np.array(lap_times)
        kernel = Matern(nu=2.5) #TODO co to???
        gp = GaussianProcessRegressor(kernel=kernel, n_restarts_optimizer=10)
        gp.fit(X_train, y_train)
        
        # Bayesian Optimization loop
        converged = False
        while not converged:
            # Determine candidate trajectory w*
            tau_best = np.min(y_train)
            alpha_best = alphas_list[np.argmin(y_train)]
            
            bounds = np.array([[0.0, 0.99] for _ in  alphas])
            
            w_star = self.find_best_alpha(gp, tau_best, alpha_best, bounds, n)
            alphas_list.append(w_star)
            
            
            # Compute min time to traverse τ* using Algorithm 1
            # Also make sure we are in global x,y coordinates, not in widths coordinates
            x_star_list = []; y_star_list = []
            x_star_list, y_star_list = self.updateAlphas(w_star)
            tau_star = self.calcMinTime((x_star_list, y_star_list))
            
            print(f"tau best: {tau_best} tau star: {tau_star}")
            
            # Add new sample to training data
            D.append((w_star, tau_star))
            X_train = np.array([np.ravel(w) for w, _ in D])
            y_train = np.array([t for _, t in D])
            
            # Update the GP model using D
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
            
        self.best = (x_star_list, y_star_list)

        return time.time() - t0