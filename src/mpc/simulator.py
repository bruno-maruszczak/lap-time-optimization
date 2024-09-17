import do_mpc
import casadi as ca
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt

from scipy.interpolate import splev

from path import Path
from mpc.model import VehicleModel   


class Simulator:
    def __init__(self, model : VehicleModel):
        self.model = model
        self.mpc_model = model.model

        self.simulator = do_mpc.simulator.Simulator(self.mpc_model)
        self.simulator.set_param(t_step = 0.1)
        self.simulator.setup()

    def plot_results(self):
        # Customizing Matplotlib:
        mpl.rcParams['font.size'] = 18
        mpl.rcParams['lines.linewidth'] = 3
        mpl.rcParams['axes.grid'] = True

        sim_graphics = do_mpc.graphics.Graphics(self.simulator.data)

        fig, ax = plt.subplots(9, sharex=True, figsize=(16,9))
        fig.align_ylabels()

        # Plot the position on curve s, n, mu
        sim_graphics.add_line(var_type='_x', var_name='s', axis=ax[0])
        sim_graphics.add_line(var_type='_x', var_name='n', axis=ax[1])
        sim_graphics.add_line(var_type='_x', var_name='mu', axis=ax[2])

        # Plot velocities
        sim_graphics.add_line(var_type='_x', var_name='vx', axis=ax[3])
        sim_graphics.add_line(var_type='_x', var_name='vy', axis=ax[4])

        sim_graphics.add_line(var_type='_x', var_name='steering_angle', axis=ax[5])
        sim_graphics.add_line(var_type='_x', var_name='throttle', axis=ax[6])
        sim_graphics.add_line(var_type='_u', var_name='steering_angle_change', axis=ax[7])
        sim_graphics.add_line(var_type='_u', var_name='throttle_change', axis=ax[8])

        ax[0].set_ylabel('s')
        ax[1].set_ylabel('n')
        ax[2].set_ylabel('mu')
        ax[3].set_ylabel('vx')
        ax[4].set_ylabel('vy')
        ax[5].set_ylabel('delta')
        ax[6].set_xlabel('T')
        ax[7].set_ylabel('ddelta')
        ax[8].set_xlabel('dT')

        return fig, ax, sim_graphics
    
    # TODO move to track.py probably
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