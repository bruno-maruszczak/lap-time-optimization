import numpy as np
import matplotlib.pyplot as plt

from track import Track
from mpc.model import VehicleModel
from mpc.controller import Controller
from mpc.simulator import Simulator

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

    simulator = Simulator(model)
    sim = simulator.simulator
    sim.x0 = x0

    fig, ax, sim_graphics = simulator.plot_results()
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

