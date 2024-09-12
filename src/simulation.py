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

def create_simulator(model : do_mpc.model.Model) -> do_mpc.simulator.Simulator:
    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step = 0.1)
    simulator.setup()

    return simulator

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

    curv1 = path.find_curvature_at_s(path.arc_lengths_sampled)
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
    model = VehicleModel(path)

    # simulation
    sim = create_simulator(model.model)
    print(model.x.labels())

if __name__ == "__main__":
    main()

