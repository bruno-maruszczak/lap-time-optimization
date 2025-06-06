import matplotlib.pyplot as plt
from matplotlib import animation, patches, transforms

class Plot:
    def __init__(self, parking_lot, results, plot_bounds, colours, car):
        self.parking_lot = parking_lot
        self.results = results
        self.plot_bounds = plot_bounds
        self.colours = colours
        self.car = car

    def static_plot(self, filename='parking_paths.png'):
        for m, (_, L) in self.results.items():
            print(f"{m} path length ≈ {L:.2f} m")

        plt.figure(figsize=(9, 12))
        ax = plt.gca(); ax.set_aspect('equal', 'box')
        ax.set_xlim(self.plot_bounds[0], self.plot_bounds[1])
        ax.set_ylim(self.plot_bounds[2], self.plot_bounds[3])
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(False)

        for slot in self.parking_lot.slot_lines:
            x, y = slot.exterior.xy
            ax.plot(x, y, color="#aaaaaa", linewidth=1)

        for pc in self.parking_lot.parked_cars:
            x, y = pc.exterior.xy
            ax.fill(x, y, color="#444444")

        for m, (coords, L) in self.results.items():
            xs, ys = zip(*[(x, y) for (x, y, _) in coords])
            ax.plot(xs, ys, color=self.colours[m], linewidth=2.5, label=f"{m} ({L:.1f} m)")
            ax.scatter(xs[0], ys[0], color=self.colours[m], marker='o')
            ax.scatter(xs[-1], ys[-1], color=self.colours[m], marker='x')

        ax.legend(loc='upper right')
        plt.tight_layout()
        plt.savefig(filename, dpi=200)

    def animate(self, filename='parking_animation.gif', order=None):
        if order is None:
            order = ["bicycle", "4ws"]

        fig, axA = plt.subplots(figsize=(9, 12))
        axA.set_aspect('equal', 'box')
        axA.set_xlim(self.plot_bounds[0], self.plot_bounds[1])
        axA.set_ylim(self.plot_bounds[2], self.plot_bounds[3])
        axA.set_xlabel('x [m]')
        axA.set_ylabel('y [m]')

        for slot in self.parking_lot.slot_lines:
            x, y = slot.exterior.xy
            axA.plot(x, y, color="#bbbbbb", linewidth=1)
        for pc in self.parking_lot.parked_cars:
            x, y = pc.exterior.xy
            axA.fill(x, y, color="#555555")

        car_patch = patches.Rectangle(
            (-self.car.CAR_LEN/2, -self.car.CAR_WID/2),
            self.car.CAR_LEN, self.car.CAR_WID,
            linewidth=1, edgecolor='k', facecolor='lime'
        )
        axA.add_patch(car_patch)

        sequence = []
        for name in order:
            pts = self.results[name][0]
            sequence.extend([(name, i) for i in range(len(pts))])
            sequence.extend([(name, len(pts)-1)] * 15)

        def update(frame):
            name, idx = frame
            x, y, yaw = self.results[name][0][idx]
            tr = transforms.Affine2D().rotate_around(0, 0, yaw).translate(x, y) + axA.transData
            car_patch.set_transform(tr)
            return [car_patch]

        ani = animation.FuncAnimation(fig, update, frames=sequence, interval=60, blit=True, repeat=True)
        try:
            ani.save(filename, writer='pillow', fps=18)
            print(f'Saved animation to {filename}')
        except Exception as e:
            print('Could not save GIF:', e)
        try:
            plt.show()
        except Exception:
            print('Headless backend: static PNG and GIF saved.')