import numpy as np
import matplotlib.pyplot as plt
import pickle
import os
import json

from mpc.track import Track as Track
from mpc.model import VehicleModel
from mpc.controller import Controller
from mpc.simulator import Simulator
import do_mpc

import casadi as ca
DEBUG = True

def load_or_create(pickle_file, cls, *constructor_args, **constructor_kwargs):
    """
    Load an object from a pickle file or create a new one if the file does not exist.

    Args:
        pickle_file (str): Path to the pickle file.
        cls (type): The class of the object to create.
        *constructor_args: Positional arguments to pass to the class constructor.
        **constructor_kwargs: Keyword arguments to pass to the class constructor.

    Returns:
        object: The loaded or newly created object.
    """
    if os.path.exists(pickle_file):
        # Load the object from the pickle file
        with open(pickle_file, 'rb') as f:
            obj = pickle.load(f)
            print("Loaded object from pickle.")
    else:
        # Create a new object
        obj = cls(*constructor_args, **constructor_kwargs)
        # Save the object to the pickle file
        with open(pickle_file, 'wb') as f:
            pickle.dump(obj, f)
            print("Saved object to pickle.")
    
    return obj

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
    track = load_or_create("track.pkl", Track, "Mazda MX-5", "buckmore", "curvature", n_samples)
    path = track.optimal_path

    # plot_dist(track, "left")
    # plt.show()

    # create_model
    
    log("Loading vehicle model...")
    model = load_or_create("model.pkl", VehicleModel, './data/vehicles/MX5.json', track)

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

