import matplotlib.pyplot as plt
import matplotlib as mpl
import json
import numpy as np
from scipy.interpolate import splev, splprep

from mpc.track import Track

class Visualiser:
    def __init__(self, track : Track, json_path):
        self.track = track
        
        self.fig, self.ax = plt.subplots(figsize=(16, 9))
        self.fig.suptitle("Visualiser")
        self.sim_data = self.read_sim_data(json_path)

        self.plot_optimal_path()
        self.plot_track()
        self.get_vehicle_positions(self.sim_data)


    def plot_optimal_path(self):
        self.ax.plot(*self.track.optimal_path.get_sample_points(), 'g')

    def plot_track(self):
        self.ax.plot(*self.track.left_bound.get_sample_points(), 'black')
        self.ax.plot(*self.track.right_bound.get_sample_points(), 'black')

    def read_sim_data(self, file_path):
        sim_data = {}
        with open(file_path, 'r') as f:
            data = json.load(f)
            for key, value in data.items():
                sim_data[key] = np.array(value)
        return sim_data

    def get_vehicle_positions(self, data):
        n = len(data['x'])
        positions = np.zeros((n, 2))
        velocities = np.zeros((n, 2))

        states = data['x']
        for i, state in enumerate(states):
            s, n, mu, vx, vy, r, _, _ = state
            u = self.track.optimal_path.find_u_given_s(s)
            spline = self.track.optimal_path.spline
            x, y = splev(u, spline)
            dx, dy = splev(u, spline, der=1)

            tangent_length = np.hypot(dx, dy)
            tangent_vector = np.array([dx, dy]) / tangent_length

            # Compute normal vector (rotate tangent by 90 degrees)
            normal_vector = np.array([-tangent_vector[1], tangent_vector[0]])

            # Compute the new point (x, y) based on deviation n along the normal vector
            x += n * normal_vector[0]
            y += n * normal_vector[1]

            # calculate the speed vector
            vx = vx * tangent_vector
            vy = vy * normal_vector

            velocity = vx + vy

            positions[i, :] = np.array([x, y]).ravel()
            velocities[i, :] = velocity.ravel()

        for i in range(0, len(positions), 10): 
            position = positions[i]
            velocity = velocities[i]
            self.ax.quiver(position[0], position[1], velocity[0], velocity[1], angles='xy', scale_units='xy', scale=1, color="blue")

        self.ax.scatter(positions[:, 0], positions[:, 1])
