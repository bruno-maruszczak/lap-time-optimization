import math
import random
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.affinity import rotate, translate
from ompl import base as ob
from ompl import control as oc

# local files
from parking.parking import ParkingLot, Car
def main():
    parking_lot = ParkingLot()
    car = Car()

    plot_bounds = parking_lot.PLOT_BOUNDS

    # ------------------ OMPL boiler‑plate (state space, dynamics) --------------

    WHEELBASE = 2.7   # fairly typical midsize‑car wheelbase
    def make_space():
        space = ob.SE2StateSpace()
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, plot_bounds[0]); bounds.setHigh(0, plot_bounds[1])
        bounds.setLow(1, plot_bounds[2]); bounds.setHigh(1, plot_bounds[3])
        space.setBounds(bounds)
        return space
    
    def bicycle_ode(q, u, qdot):
        theta = q[2]; v = u[0]; steer = u[1]
        qdot[0] = v * math.cos(theta)
        qdot[1] = v * math.sin(theta)
        qdot[2] = v * math.tan(steer) / WHEELBASE


    def fourws_ode(q, u, qdot):
        theta = q[2]; v = u[0]; sf = u[1]; sr = u[2]
        qdot[0] = v * math.cos(theta)
        qdot[1] = v * math.sin(theta)
        qdot[2] = v * (math.tan(sf) - math.tan(sr)) / WHEELBASE


    def is_state_valid(state):
        car_polygon = car.polygon(state.getX(), state.getY(), state.getYaw())
        # Keep inside global bounds
        if not parking_lot.lot_boundary.contains(car_polygon):
            return False
        # Collision with any parked car
        return all(not car_polygon.intersects(obstacle) for obstacle in parking_lot.obstacles)

    # ------------------ Planner wrapper ----------------------------------------

    def create_planner(model: str, runtime: float = 60.0):
        space = make_space()
        dim   = 2 if model == "bicycle" else 3
        cspace = oc.RealVectorControlSpace(space, dim)

        cb = ob.RealVectorBounds(dim)
        cb.setLow(0, -1.5); cb.setHigh(0, 1.5)             # ±1.5 m/s
        cb.setLow(1, -0.523); cb.setHigh(1, 0.523)          # ±30° steering
        if dim == 3:
            cb.setLow(2, -0.523); cb.setHigh(2, 0.523)
        cspace.setBounds(cb)

        ss = oc.SimpleSetup(cspace)
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(is_state_valid))

        ode_fn = bicycle_ode if model == "bicycle" else fourws_ode
        solver = oc.ODEBasicSolver(ss.getSpaceInformation(), oc.ODE(ode_fn))
        ss.setStatePropagator(oc.ODESolver.getStatePropagator(solver))

        si = ss.getSpaceInformation(); si.setMinMaxControlDuration(1, 20); si.setPropagationStepSize(0.1)

        start = ob.State(space)
        start_x = 0.0
        start_y = parking_lot.Y_MIN + car.CAR_LEN/2 + 0.5  # keep entire car inside bounds
        start().setX(start_x); start().setY(start_y)  # just below the first row
        start().setYaw(math.pi*0.95)              # facing "up"

        goal_cx = - (parking_lot.N_SPOTS/2 - 0.5 - parking_lot.GOAL_COL) * parking_lot.SLOT_W
        goal_cy = parking_lot.ROWS_Y[parking_lot.GOAL_ROW]
        goal = ob.State(space)
        goal().setX(goal_cx); goal().setY(goal_cy); goal().setYaw(math.pi/2)

        ss.setStartAndGoalStates(start, goal, 0.25)
        ss.getProblemDefinition().setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
        ss.setPlanner(oc.SST(si))

        if not ss.solve(runtime):
            raise RuntimeError(f"{model}: planning failed")

        path = ss.getSolutionPath(); path.interpolate()
        coords = [(st.getX(), st.getY(), st.getYaw()) for st in path.getStates()]
        length = sum(math.hypot(coords[i][0] - coords[i-1][0], coords[i][1] - coords[i-1][1]) for i in range(1, len(coords)))
        return coords, length

    # ------------------ Main (plot & animate) -----------------------------------

    if __name__ == "__main__":
        random.seed(7)

        colours = {"bicycle": "deepskyblue", "4ws": "orange"}
        results = {m: create_planner(m) for m in ("bicycle", "4ws")}

        for m, (_, L) in results.items():
            print(f"{m} path length ≈ {L:.2f} m")

        # ---------- STATIC PLOT (larger) ----------
        plt.figure(figsize=(9, 12))
        ax = plt.gca(); ax.set_aspect('equal', 'box')
        ax.set_xlim(plot_bounds[0], plot_bounds[1])
        ax.set_ylim(plot_bounds[2], plot_bounds[3])
        ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(False)

        # Draw parking slot outlines (thin grey)
        for slot in parking_lot.slot_lines:
            x, y = slot.exterior.xy
            ax.plot(x, y, color="#aaaaaa", linewidth=1)

        # Draw parked cars (dark grey)
        for pc in parking_lot.parked_cars:
            x, y = pc.exterior.xy
            ax.fill(x, y, color="#444444")

        # Plot each planned path
        for m, (coords, L) in results.items():
            xs, ys = zip(*[(x, y) for (x, y, _) in coords])
            ax.plot(xs, ys, color=colours[m], linewidth=2.5, label=f"{m} ({L:.1f} m)")
            ax.scatter(xs[0], ys[0], color=colours[m], marker='o')
            ax.scatter(xs[-1], ys[-1], color=colours[m], marker='x')

        ax.legend(loc='upper right'); plt.tight_layout(); plt.savefig('parking_paths.png', dpi=200)

        # ---------- ANIMATION ----------
        from matplotlib import animation, patches, transforms
        fig, axA = plt.subplots(figsize=(9, 12)); axA.set_aspect('equal', 'box')
        axA.set_xlim(plot_bounds[0], plot_bounds[1]); axA.set_ylim(plot_bounds[2], plot_bounds[3])
        axA.set_xlabel('x [m]'); axA.set_ylabel('y [m]')

        # Draw static scenery
        for slot in parking_lot.slot_lines:
            x, y = slot.exterior.xy; axA.plot(x, y, color="#bbbbbb", linewidth=1)
        for pc in parking_lot.parked_cars:
            x, y = pc.exterior.xy; axA.fill(x, y, color="#555555")

        # Animated robot patch
        car_patch = patches.Rectangle((-car.CAR_LEN/2, -car.CAR_WID/2), car.CAR_LEN, car.CAR_WID,
                                    linewidth=1, edgecolor='k', facecolor='lime')
        axA.add_patch(car_patch)

        sequence = []
        order    = ["bicycle", "4ws"]
        for name in order:
            pts = results[name][0]
            sequence.extend([(name, i) for i in range(len(pts))])
            sequence.extend([(name, len(pts)-1)] * 15)  # pause

        def update(frame):
            name, idx = frame
            x, y, yaw = results[name][0][idx]
            tr = transforms.Affine2D().rotate_around(0, 0, yaw).translate(x, y) + axA.transData
            car_patch.set_transform(tr)
            return [car_patch]

        ani = animation.FuncAnimation(fig, update, frames=sequence, interval=60, blit=True, repeat=True)
        try:
            ani.save('parking_animation.gif', writer='pillow', fps=18)
            print('Saved animation to parking_animation.gif')
        except Exception as e:
            print('Could not save GIF:', e)
        try:
            plt.show()
        except Exception:
            print('Headless backend: static PNG and GIF saved.')




if __name__ == "__main__":
    main()