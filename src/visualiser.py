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
        #self.sim_data = self.read_sim_data(json_path)


    def plot_optimal_path(self):
        self.ax.plot(self.track.optimal_path.get_sample_points(), 'g')

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
        positions = np.zeros(2, n)

        # setup first position
        positions[0] = self.track.optimal_path.position(s = 0.)

        X = data['x']
        for i,x in enumerate(X):
            pass

