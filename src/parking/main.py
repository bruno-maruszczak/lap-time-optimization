import math
import random
import matplotlib.pyplot as plt
from shapely.geometry import Polygon
from shapely.affinity import rotate, translate
from ompl import base as ob
from ompl import control as oc

# ----------------- Parking‑lot geometry ------------------------------------
# Roughly life‑sized, metres.
SLOT_W   = 2.6          # parking‑slot width  (x‑direction)
SLOT_L   = 5.5          # parking‑slot length (y‑direction)
AISLE_W  = 6.0          # drive aisle between two facing rows
N_SPOTS  = 8            # columns per row
ROWS_Y   = [SLOT_L / 2, SLOT_L + AISLE_W + SLOT_L / 2]  # centre‑lines of the two rows

# Choose the *goal* slot (row index 0 = front row, 1 = back row)
GOAL_ROW = 0
GOAL_COL = 3  # zero‑based index in the row (so fourth slot from left)

# Bounds of the lot (pad a little for margins)
X_MIN = - (N_SPOTS/2) * SLOT_W - 9
X_MAX =   (N_SPOTS/2) * SLOT_W + 9
# Extend downward so the whole car fits at the starting pose
Y_MIN = -10.0  # ample space below the rows                            # room to start outside the lot
Y_MAX = ROWS_Y[1] + SLOT_L/2 + 10.0
PLOT_BOUNDS = [X_MIN, X_MAX, Y_MIN, Y_MAX]

# Car that will move (our robot)
CAR_LEN = 4.5   # m
CAR_WID = 2.0   # m  (larger relative to slot for realism)

# Parked‑car rectangles (obstacles) and slot outlines  ----------------------
parked_cars = []   # Polygons for collision checking
slot_lines  = []   # Each slot as a Polygon outline for plotting only

for row_idx, cy in enumerate(ROWS_Y):
    # x‑coordinate of the centre of the *leftmost* slot
    first_cx = - (N_SPOTS/2 - 0.5) * SLOT_W
    for col in range(N_SPOTS):
        cx = first_cx + col * SLOT_W
        # Slot outline (thin rectangle the size of the slot)
        slot = Polygon([  # rectangle corners (counter‑clockwise)
            (cx - SLOT_W/2, cy - SLOT_L/2),
            (cx + SLOT_W/2, cy - SLOT_L/2),
            (cx + SLOT_W/2, cy + SLOT_L/2),
            (cx - SLOT_W/2, cy + SLOT_L/2)
        ])
        slot_lines.append(slot)

        # Skip the goal slot so it remains empty
        if row_idx == GOAL_ROW and col == GOAL_COL:
            continue

        # Create a parked car inside this slot (slightly inset)
        inset = 0.15
        pcar = Polygon([
            (cx - (CAR_WID/2), cy - (CAR_LEN/2)),
            (cx + (CAR_WID/2), cy - (CAR_LEN/2)),
            (cx + (CAR_WID/2), cy + (CAR_LEN/2)),
            (cx - (CAR_WID/2), cy + (CAR_LEN/2))
        ])
        parked_cars.append(pcar)

# Lot boundary (outline only; we allow driving over the white lines)
lot_boundary = Polygon([
    (PLOT_BOUNDS[0], PLOT_BOUNDS[2]), (PLOT_BOUNDS[1], PLOT_BOUNDS[2]),
    (PLOT_BOUNDS[1], PLOT_BOUNDS[3]), (PLOT_BOUNDS[0], PLOT_BOUNDS[3])
])

# ------------------ Helper to make robot car polygon -----------------------

def car_polygon(x: float, y: float, yaw: float) -> Polygon:
    rect = Polygon([
        (-CAR_LEN/2, -CAR_WID/2), (CAR_LEN/2, -CAR_WID/2),
        (CAR_LEN/2,  CAR_WID/2), (-CAR_LEN/2,  CAR_WID/2)
    ])
    rect = rotate(rect, math.degrees(yaw), origin=(0, 0), use_radians=False)
    return translate(rect, x, y)

# ------------------ OMPL boiler‑plate (state space, dynamics) --------------

WHEELBASE = 2.7   # fairly typical midsize‑car wheelbase


def make_space():
    space = ob.SE2StateSpace()
    bounds = ob.RealVectorBounds(2)
    bounds.setLow(0, PLOT_BOUNDS[0]); bounds.setHigh(0, PLOT_BOUNDS[1])
    bounds.setLow(1, PLOT_BOUNDS[2]); bounds.setHigh(1, PLOT_BOUNDS[3])
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
    car = car_polygon(state.getX(), state.getY(), state.getYaw())
    # Keep inside global bounds
    if not lot_boundary.contains(car):
        return False
    # Collision with any parked car
    return all(not car.intersects(pc) for pc in parked_cars)

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
    start_y = Y_MIN + CAR_LEN/2 + 0.5  # keep entire car inside bounds
    start().setX(start_x); start().setY(start_y)  # just below the first row
    start().setYaw(math.pi*0.95)              # facing "up"

    goal_cx = - (N_SPOTS/2 - 0.5 - GOAL_COL) * SLOT_W
    goal_cy = ROWS_Y[GOAL_ROW]
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
    ax.set_xlim(PLOT_BOUNDS[0], PLOT_BOUNDS[1])
    ax.set_ylim(PLOT_BOUNDS[2], PLOT_BOUNDS[3])
    ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]'); ax.grid(False)

    # Draw parking slot outlines (thin grey)
    for slot in slot_lines:
        x, y = slot.exterior.xy
        ax.plot(x, y, color="#aaaaaa", linewidth=1)

    # Draw parked cars (dark grey)
    for pc in parked_cars:
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
    axA.set_xlim(PLOT_BOUNDS[0], PLOT_BOUNDS[1]); axA.set_ylim(PLOT_BOUNDS[2], PLOT_BOUNDS[3])
    axA.set_xlabel('x [m]'); axA.set_ylabel('y [m]')

    # Draw static scenery
    for slot in slot_lines:
        x, y = slot.exterior.xy; axA.plot(x, y, color="#bbbbbb", linewidth=1)
    for pc in parked_cars:
        x, y = pc.exterior.xy; axA.fill(x, y, color="#555555")

    # Animated robot patch
    car_patch = patches.Rectangle((-CAR_LEN/2, -CAR_WID/2), CAR_LEN, CAR_WID,
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
