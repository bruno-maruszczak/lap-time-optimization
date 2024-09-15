from numpy.typing import ArrayLike
from mpc.model import VehicleModel
import do_mpc

class Controller:
    def __init__(self, model : VehicleModel, control_costs : ArrayLike, n_horizon : int = 20, t_step : float = 0.1, n_robust : int = 1):
        self.model = model
        self.mpc_model = self.model.model
        self.t_step = t_step
        self.mpc = do_mpc.controller.MPC(self.mpc_model)

        setup_mpc = {
            'n_horizon' : n_horizon,
            't-step' : t_step,
            'n_robust' : n_robust,
            'store_full_solution' : True,
        }
        self.mpc.set_param(**setup_mpc)    

        self.set_objective(control_costs)

    def set_objective(self, control_costs):
        u_dim = len(self.mpc_model.u.keys())
        assert (control_costs.shape == (u_dim, 1))
        

        r_term = {key : cost for key, cost in zip(self.mpc_model.u.keys(), control_costs)}
        self.mpc.set_rterm(**r_term) 

        # mterm - Add algebraic constraints here
        return #TODO
        mterm = None
        lterm = None
        self.mpc.set_objective(mterm=mterm, lterm=lterm)

    def set_constraints(self):
        # Lower state bounds
        self.mpc.bounds['lower', '_x', 's'] = -1.
        self.mpc.bounds['lower', '_x', 'n'] = -1.
        self.mpc.bounds['lower', '_x', 'mu'] = -1.
        self.mpc.bounds['lower', '_x', 'vx'] = -1.
        self.mpc.bounds['lower', '_x', 'vy'] = -1.
        self.mpc.bounds['lower', '_x', 'r'] = -1.
        self.mpc.bounds['lower', '_x', 'steering_angle'] = -1.
        self.mpc.bounds['lower', '_x', 'throttle'] = -1.

        # Upper state bounds
        self.mpc.bounds['upper', '_x', 's'] = -1.
        self.mpc.bounds['upper', '_x', 'n'] = -1.
        self.mpc.bounds['upper', '_x', 'mu'] = -1.
        self.mpc.bounds['upper', '_x', 'vx'] = -1.
        self.mpc.bounds['upper', '_x', 'vy'] = -1.
        self.mpc.bounds['upper', '_x', 'r'] = -1.
        self.mpc.bounds['upper', '_x', 'steering_angle'] = -1.
        self.mpc.bounds['upper', '_x', 'throttle'] = -1.

        # Lower input bounds
        self.mpc.bounds['lower', '_u', 'steering_angle_change'] = None
        self.mpc.bounds['lower', '_u', 'throttle'] = None

        # Upper input bounds
        self.mpc.bounds['upper', '_u', 'steering_angle'] = None
        self.mpc.bounds['upper', '_u', 'throttle'] = None

