import numpy as np
import do_mpc
import casadi as ca

from numpy.typing import ArrayLike
from mpc.model import VehicleModel

class Controller:
    def __init__(self, model : VehicleModel, control_costs : ArrayLike, n_horizon : int = 20, t_step : float = 0.8, n_robust : int = 0):
        self.model = model
        self.mpc_model = self.model.model
        self.t_step = t_step
        self.mpc = do_mpc.controller.MPC(self.mpc_model)

        setup_mpc = {
            'n_horizon' : n_horizon,
            't_step' : t_step,
            'n_robust' : n_robust,
            'store_full_solution' : False,
            'nlpsol_opts': {'ipopt.max_iter': 1000 }
        }
        self.mpc.set_param(**setup_mpc)    

        # rho determines the shape of the friction ellipse constraint
        # alpha determines the maximum combined force. (the size of the ellipse)
        alpha, rho = 1.0, 1.0

        # q_n is a cost for deviating from the optimal path (in perpendicular to path direction)
        # q_mu is a cost for a heading that deviates from the optimal path
        # q_B penalizes the difference between the kinematic and dynamic side slip angle.
        q_n, q_mu, q_B = 0.5, 3.0, 1e-2
        self.set_constraints(rho, alpha)
        self.set_objective(control_costs, q_n, q_mu, q_B)

        # options for mpc log
        self.mpc.settings.nlpsol_opts['ipopt.print_level'] = 0
        # self.mpc.settings.nlpsol_opts['print_time'] = 0
        # self.mpc.settings.nlpsol_opts['ipopt.sb'] = 'yes'  
        # self.mpc.settings.supress_ipopt_output()
        self.mpc.setup()

    def set_objective(self, control_costs, q_n = 1.0, q_mu = 1.0, q_B = 1.0):
        u_dim = len(self.mpc_model.u.labels())
        assert (control_costs.shape == (u_dim, 1))
        
        r_term = {key : cost for key, cost in zip(self.mpc_model.u.keys(), control_costs)}
        self.mpc.set_rterm(**r_term) 

        # Get variables from model
        sdot = self.model.sdot()
        s = self.mpc_model.x['s']
        n = self.mpc_model.x['n']
        mu = self.mpc_model.x['mu']


        mterm = q_n*(n**2) + q_mu*(mu**2) + self.model.B(q_B) # Use to recreate plot results
        lterm = mterm - sdot

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

        # front, back = self.model.get_traction_ellipse_constraint(throttle, vx, vy, r, steering_angle, rho, alpha)
        # self.mpc.set_nl_cons('front_traction_ellipse_cons', front, 0., soft_constraint=True)
        # self.mpc.set_nl_cons('back_traction_ellipse_cons', back, 0., soft_constraint=True)

        # TODO Set constraints for velocities, from calculated optimal max_velocities. 
        not_set = 0.
        # Lower state bounds
        self.mpc.bounds['lower', '_x', 's'] = 0.
        self.mpc.bounds['lower', '_x', 'mu'] = -np.pi*0.5
        self.mpc.bounds['lower', '_x', 'vx'] = 0. 
        # self.mpc.bounds['lower', '_x', 'vy'] = not_set
        # self.mpc.bounds['lower', '_x', 'r'] = not_set
        self.mpc.bounds['lower', '_x', 'steering_angle'] = -np.pi/4
        self.mpc.bounds['lower', '_x', 'throttle'] = -1

        # # Upper state bounds
        # self.mpc.bounds['upper', '_x', 's'] = not_set
        # self.mpc.bounds['upper', '_x', 'n'] = not_set 
        self.mpc.bounds['upper', '_x', 'mu'] = np.pi*0.5 
        # self.mpc.bounds['upper', '_x', 'vx'] = not_set
        # self.mpc.bounds['upper', '_x', 'vy'] = not_set
        # self.mpc.bounds['upper', '_x', 'r'] = not_set 
        self.mpc.bounds['upper', '_x', 'steering_angle'] = np.pi/4
        self.mpc.bounds['upper', '_x', 'throttle'] = 1

        # Lower input bounds
        self.mpc.bounds['lower', '_u', 'steering_angle_change'] = -2*np.pi/4
        self.mpc.bounds['lower', '_u', 'throttle_change'] = -1 

        # Upper input bounds
        self.mpc.bounds['upper', '_u', 'steering_angle_change'] = 2*np.pi/4
        self.mpc.bounds['upper', '_u', 'throttle_change'] = 1

