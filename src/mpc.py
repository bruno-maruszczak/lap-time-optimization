import numpy as np
import matplotlib.pyplot as plt
import pickle
import os
import json
import argparse
from enum import IntEnum, unique

from mpc.track import Track as Track
from mpc.model import VehicleModel
from mpc.controller import Controller
from mpc.simulator import Simulator
from visualiser import Visualiser

import do_mpc

import casadi as ca
DEBUG = True

@unique
class Method(IntEnum):
    CURVATURE = 0
    COMPROMISE = 1
    DIRECT = 2
    BAYES = 3

parser = argparse.ArgumentParser(description='Choosing race path to follow')
methods = parser.add_argument_group(
    'generation methods').add_mutually_exclusive_group(required=True)
methods.add_argument('--curvature',
                     action='store_const', dest='method', const=Method.CURVATURE,
                     help='curvature path'
                     )
methods.add_argument('--compromise',
                     action='store_const', dest='method', const=Method.COMPROMISE,
                     help='an optimal length-curvature compromise path'
                     )
methods.add_argument('--laptime',
                     action='store_const', dest='method', const=Method.DIRECT,
                     help='path computed with direct lap time'
                     )
methods.add_argument('--bayes',
                     action='store_const', dest='method', const=Method.BAYES,
                     help='optimal path via bayesian optimisation'
                     )

args = parser.parse_args()

if args.method is Method.CURVATURE:
    print("[ Path method: curvature ]")
    method = "curvature"
elif args.method is Method.COMPROMISE:
    print("[ Path method: compromise ]")
    method = "compromise"
elif args.method is Method.DIRECT:
    print("[ Path method: compromise ]")
    method = "compromise"
elif args.method is Method.BAYES:
    print("[ Path method: BAYES ]")
    method = "bayesian"
else:
    raise ValueError("Did not recognise args.method {}".format(args.method))

def plot_dist(track : Track, side):
    """
    Given Path object plots the path's curvature, comparing numerical values with symbolical interpolation (casadi),
    which is needed for do_mpc model.
    """

    s_sym = ca.SX.sym('s')
    expr = track.find_dist_to_band(s_sym, side)
    
    d = ca.Function('curvature', [s_sym], [expr])
    
    d_sym = np.array([d(s).full().flatten() for s in track.optimal_path.arc_lengths_sampled])

    s = track.optimal_path.arc_lengths_sampled 
    fig, ax = plt.subplots()
    ax.plot(s, track.bound_dist_table['left'])
    ax.plot(s, d_sym, 'r--')

def log(msg, **kwargs):
    if DEBUG:
        print(msg, **kwargs)

def main(): 
    log("Loading Track")
    n_samples = 1000
    track = Track("MX-5", "buckmore", method, n_samples)
    path = track.optimal_path


    # plot_dist(track, "left")
    # plt.show()

    # create_model
    
    log("Loading vehicle model...")
    model = VehicleModel('./data/vehicles/MX5.json', track)

    # create controller
    log("Creating MPC controller...")
    # Pickle doesnt work with do_mpc mpc controller, there's a function that saves the solution object, works only on linux TODO
    controller = Controller(model, np.reshape([1e-2, 1e-2], (-1, 1)))

    # Prepare x0
    s0, n0, mu0 = 0., 0., 0.
    vx0, vy0, r0 = 0.1, 0.0, 0.
    steer_angle0, throttle0 = 0., 0.1
    x0 = np.reshape([s0, n0, mu0, vx0, vy0, r0, steer_angle0, throttle0], (-1, 1))

    # simualtion
    log("Creating simulator...")
    simulator = Simulator(model)
    sim = simulator.simulator
    sim.x0 = x0
    controller.mpc.x0 = x0 
    controller.mpc.set_initial_guess()
    estimator = do_mpc.estimator.StateFeedback(model.model)
    estimator.x0 = x0

    fig, ax, sim_graphics = simulator.plot_results()
    u0 = np.zeros((2,1))
    
    steps = 20
    # Prepare variables for saving states, contorl to json
    X = np.zeros((steps + 1, *x0.shape))
    X[0] = x0
    
    Y = np.zeros((steps + 1, *x0.shape))
    Y[0] = x0

    U = np.zeros((steps + 1, *u0.shape))
    U[0] = u0

    for i in range(1, steps + 1):
        log(f"\n---------------------------\nsimulation step: {i}\n---------------------------\n")
        u0 = controller.mpc.make_step(x0)
        y = sim.make_step(u0)
        x0 = estimator.make_step(y)
        X[i] = x0
        Y[i] = y
        U[i] = u0

    # Save to json
    with open('sim_results.json', 'w') as f:
        data = {'x': X.tolist(), 'y': Y.tolist(), 'u': U.tolist()}
        json.dump(data, f)
    
    visualiser = Visualiser(track, 'sim_results.json')
    visualiser.plot_optimal_path()
    plt.show()

    # Read json
    with open('sim_results.json', 'r') as f:
        data = json.load(f)
        print(np.array(data['x']), np.array(data['y']), np.array(data['u']), sep='\n')

    log("Plotting results...")
    sim_graphics.plot_results()
    # Reset the limits on all axes in graphic to show the data.
    sim_graphics.reset_axes()
    # Show the figure:
    plt.show()

if __name__ == "__main__":
    main()

