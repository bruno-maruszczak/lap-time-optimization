import numpy as np
import do_mpc

import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import splprep, splev
from casadi import vertcat

import matplotlib.pyplot as plt
import matplotlib as mpl

from model import VehicleModel
from track import Path as Spline

def create_simulator(model : do_mpc.model.Model) -> do_mpc.simulator.Simulator:
    simulator = do_mpc.simulator.Simulator(model)
    simulator.set_param(t_step = 0.1)
    simulator.setup()

    return simulator

def generate_circle_points(radius, center_x, center_y, num_points):
    points = []
    angle_step = 2 * np.pi / num_points  # Angle between points in radians

    for i in range(num_points):
        angle = i * angle_step
        x = center_x + radius * np.cos(angle)
        y = center_y + radius * np.sin(angle)
        points.append((x, y))

    return np.array(points)

def main():
    # Example track (circle)
    points = generate_circle_points(10, 0, 0, 16)
    x_points, y_points = points[:, 0], points[:, 1]
    track = Spline([x_points, y_points], closed=True)

    # create_model
    model = VehicleModel(track)

    # simulation
    sim = create_simulator(model.model)
    print(model.x.labels())

if __name__ == "__main__":
    main()
