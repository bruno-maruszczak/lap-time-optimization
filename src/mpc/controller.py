import numpy as np
import do_mpc
import casadi as ca

from numpy.typing import ArrayLike
from mpc.model import VehicleModel

class Controller:
    def __init__(self, model : VehicleModel, control_costs : ArrayLike, n_horizon : int = 20, t_step : float = 0.1, n_robust : int = 1):
        self.model = model
        self.mpc_model = self.model.model
        self.t_step = t_step
        self.mpc = do_mpc.controller.MPC(self.mpc_model)

        setup_mpc = {
            'n_horizon' : n_horizon,
            't_step' : t_step,
            'n_robust' : n_robust,
            'store_full_solution' : True,
        }
        self.mpc.set_param(**setup_mpc)    

        alpha, rho = 1.0, 1.0
        q_n, q_mu = 1.0, 1.0
        self.set_constraints(rho, alpha)
        self.set_objective(control_costs, q_n, q_mu)
        self.mpc.setup()

    def set_objective(self, control_costs, q_n = 1.0, q_mu = 1.0):
        u_dim = len(self.mpc_model.u.labels())
        assert (control_costs.shape == (u_dim, 1))
        
        r_term = {key : cost for key, cost in zip(self.mpc_model.u.keys(), control_costs)}
        self.mpc.set_rterm(**r_term) 

        # Get variables from model
        sdot = self.model.sdot()
        s = self.mpc_model.x['s']
        n = self.mpc_model.x['n']
        mu = self.mpc_model.x['mu']

        # mterm - Add algebraic constraints here
        # TODO B(x)
        # 
        lterm = -self.t_step*(sdot) + q_n*(n**2) + q_mu*(mu**2) #+ B(self.mpc_model.x)
        mterm = ca.SX(0)
        self.mpc.set_objective(lterm=lterm, mterm=mterm)

    def set_constraints(self, rho, alpha):
        s = self.mpc_model.x['s']
        n = self.mpc_model.x['n']
        mu = self.mpc_model.x['mu']
        vx = self.mpc_model.x['vx']
        vy = self.mpc_model.x['vy']
        r = self.mpc_model.x['r']
        steering_angle = self.mpc_model.x['steering_angle']
        throttle = self.mpc_model.x['throttle']

        # non-linear constraints (for the vehicle to stay in track)
        left, right = self.model.get_lateral_constraint(s, n, mu)
        self.mpc.set_nl_cons('left_dist_cons', left, 0.)
        self.mpc.set_nl_cons('right_dist_cons', right, 0.)

        front, back = self.model.get_traction_ellipse_constraint(throttle, vx, vy, r, steering_angle, rho, alpha)
        self.mpc.set_nl_cons('front_traction_ellipse_cons', front, 0.)
        self.mpc.set_nl_cons('back_traction_ellipse_cons', back, 0.)

        # TODO poitentially add additional constraints, ex. for sterring angle, throttle or inputs
        not_set = 0.
        # Lower state bounds
        self.mpc.bounds['lower', '_x', 's'] = 0.
        self.mpc.bounds['lower', '_x', 'n'] = 0.
        self.mpc.bounds['lower', '_x', 'mu'] = 0.
        # self.mpc.bounds['lower', '_x', 'vx'] = not_set
        # self.mpc.bounds['lower', '_x', 'vy'] = not_set
        # self.mpc.bounds['lower', '_x', 'r'] = not_set
        # self.mpc.bounds['lower', '_x', 'steering_angle'] = not_set
        # self.mpc.bounds['lower', '_x', 'throttle'] = not_set

        # # Upper state bounds
        # self.mpc.bounds['upper', '_x', 's'] = not_set
        # self.mpc.bounds['upper', '_x', 'n'] = not_set 
        # self.mpc.bounds['upper', '_x', 'mu'] = not_set
        # self.mpc.bounds['upper', '_x', 'vx'] = not_set
        # self.mpc.bounds['upper', '_x', 'vy'] = not_set
        # self.mpc.bounds['upper', '_x', 'r'] = not_set 
        # self.mpc.bounds['upper', '_x', 'steering_angle'] = not_set 
        # self.mpc.bounds['upper', '_x', 'throttle'] = not_set 

        # # Lower input bounds
        # self.mpc.bounds['lower', '_u', 'steering_angle_change'] = not_set 
        # self.mpc.bounds['lower', '_u', 'throttle_change'] = not_set 

        # # Upper input bounds
        # self.mpc.bounds['upper', '_u', 'steering_angle_change'] = not_set 
        # self.mpc.bounds['upper', '_u', 'throttle_change'] = not_set 

