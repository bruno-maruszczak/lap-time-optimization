import matplotlib.pyplot as plt
import matplotlib as mpl
import json

from mpc.track import Track

class Visualiser:
    def __init__(self, track : Track, json_path):
        self.track = track
        
        self.fig, self.ax = plt.subplots(figsize=(16, 9))

    def plot_optimal_path(self):
        path = self.track.optimal_path
        x, y = path.get_sample_points()
        self.ax.plot(x, y)

