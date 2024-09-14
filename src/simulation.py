import numpy as np
import do_mpc

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from casadi import vertcat
import casadi as ca
import matplotlib.pyplot as plt
import matplotlib as mpl

from model import VehicleModel
from track import Track
from path import Path

from numpy.typing import ArrayLike
class Controller:
    def __init__(self, model : VehicleModel, control_costs : ArrayLike, n_horizon : int = 20, t_step : float = 0.1, n_robust : int = 1):
        self.model = model
        self.mpc_model = self.model.model

        self.mpc = do_mpc.controller.MPC(self.mpc_model)

        setup_mpc = {
            'n_horizon' : n_horizon,
            't-step' : 0.1,
            'n_robust' : 2,
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
        self.mpc.bounds['lower', '_x', 's'] = 0.
        self.mpc.bounds['lower', '_x', 'n'] = 0.
        self.mpc.bounds['lower', '_x', 'mu'] = 0.
        self.mpc.bounds['lower', '_x', 'vx'] = 0.
        self.mpc.bounds['lower', '_x', 'vy'] = 0.
        self.mpc.bounds['lower', '_x', 'r'] = 0.
        self.mpc.bounds['lower', '_x', 'steering_angle'] = 0.
        self.mpc.bounds['lower', '_x', 'throttle'] = 0.

        # Upper state bounds
        self.mpc.bounds['upper', '_x', 's'] = 0.
        self.mpc.bounds['upper', '_x', 'n'] = 0.
        self.mpc.bounds['upper', '_x', 'mu'] = 0.
        self.mpc.bounds['upper', '_x', 'vx'] = 0.
        self.mpc.bounds['upper', '_x', 'vy'] = 0.
        self.mpc.bounds['upper', '_x', 'r'] = 0.
        self.mpc.bounds['upper', '_x', 'steering_angle'] = 0.
        self.mpc.bounds['upper', '_x', 'throttle'] = 0.

        # Lower input bounds
        self.mpc.bounds['lower', '_u', 'steering_angle_change'] = None
        self.mpc.bounds['lower', '_u', 'throttle'] = None

        # Upper input bounds
        self.mpc.bounds['upper', '_u', 'steering_angle'] = None
        self.mpc.bounds['upper', '_u', 'throttle'] = None


        
def create_simulator(model : do_mpc.model.Model) -> do_mpc.simulator.Simulator:
    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step = 0.1)
    simulator.setup()

    return simulator

def plot_results(model : do_mpc.model.Model, sim : do_mpc.simulator.Simulator):
    # Customizing Matplotlib:
    mpl.rcParams['font.size'] = 18
    mpl.rcParams['lines.linewidth'] = 3
    mpl.rcParams['axes.grid'] = True

    sim_graphics = do_mpc.graphics.Graphics(sim.data)

    fig, ax = plt.subplots(3, sharex=True, figsize=(16,9))
    fig.align_ylabels()


    # Plot the position on curve s, n, mu
    sim_graphics.add_line(var_type='_x', var_name='s', axis=ax[0])
    sim_graphics.add_line(var_type='_x', var_name='n', axis=ax[0])
    sim_graphics.add_line(var_type='_x', var_name='mu', axis=ax[0])

    # Plot velocities
    sim_graphics.add_line(var_type='_x', var_name='vx', axis=ax[1])
    sim_graphics.add_line(var_type='_x', var_name='vy', axis=ax[1])


    ax[0].set_ylabel('position on a curve [s, n, mu]')
    ax[1].set_ylabel('speed in x and y [m/s]')
    ax[2].set_ylabel('inputs [angle, T]')
    ax[2].set_xlabel('time [s]')
    return fig, ax, sim_graphics
 

def plot_curvatures(path : Path):
    """
    Given Path object plots the path's curvature, comparing numerical values with symbolical interpolation (casadi),
    which is needed for do_mpc model.
    """
    spline = path.spline

    s_sym = ca.SX.sym('s')
    lookup = np.array(path.curvature_lookup_table)
    expr = path.find_curvature_at_s(s_sym)
    
    k = ca.Function('curvature', [s_sym], [expr])
    curv_sym = np.array([k(s).full().flatten() for s in path.arc_lengths_sampled])

    x, y = splev(path.u_sampled , spline)
    
    plt.plot(x, y)
    plt.figure()
    plt.plot(lookup[:, 0].tolist(), lookup[:, 1].tolist())
    plt.plot(path.arc_lengths_sampled, curv_sym, "r--")
    plt.show()

def main():
    # load sample track
    track = Track('./data/tracks/buckmore.json', track_width=1.0)
    path = track.mid
    
    #plot_curvatures(path)

    # create_model
    model = VehicleModel('./data/vehicles/MX5.json', path)

    print(model.model.x.labels())
    # create controller
    controller = Controller(model, np.reshape([1e-2, 1e-2], (-1, 1)))

    # simulation
    n = len(model.model.x.labels())
    s0, n0, mu0 = 0., 0., 0.
    vx0, vy0, r0 = 0.1, 0.1, 0.
    steer_angle0, throttle0 = 0., 1.
    x0 = np.reshape([s0, n0, mu0, vx0, vy0, r0, steer_angle0, throttle0], (-1, 1))


    sim = create_simulator(model.model)
    sim.x0 = x0

    fig, ax, sim_graphics = plot_results(model, sim)
    print(model.model.u)
    u0 = np.zeros((2,1))
    for i in range(10):
        sim.make_step(u0)

    sim_graphics.plot_results()
    # Reset the limits on all axes in graphic to show the data.
    sim_graphics.reset_axes()
    # Show the figure:
    plt.show()

if __name__ == "__main__":
    main()


