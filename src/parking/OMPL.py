from ompl import base as ob
from ompl import control as oc
import matplotlib.pyplot as plt
import math

# ------------------ OMPL boiler‑plate (state space, dynamics) --------------
class OMPL:
    WHEELBASE = 2.7   # fairly typical midsize‑car wheelbase

    def __init__(self, plot_bounds, car, parking_lot):
        self.plot_bounds = plot_bounds
        self.car = car
        self.parking_lot = parking_lot

    def make_space(self):
        space = ob.SE2StateSpace()
        bounds = ob.RealVectorBounds(2)
        bounds.setLow(0, self.plot_bounds[0])
        bounds.setHigh(0, self.plot_bounds[1])
        bounds.setLow(1, self.plot_bounds[2])
        bounds.setHigh(1, self.plot_bounds[3])
        space.setBounds(bounds)
        width = bounds.high[0] - bounds.low[0]
        height = bounds.high[1] - bounds.low[1]
        # print("Bounds object:", bounds)
        # print("Space bounds:", space.getBounds())
        # print(f"Width: {width}, Height: {height}")
        return space

    def bicycle_ode(self,q, u, qdot):
        theta = q[2]
        v = u[0]
        steer = u[1]
        qdot[0] = v * math.cos(theta)
        qdot[1] = v * math.sin(theta)
        qdot[2] = v * math.tan(steer) / OMPL.WHEELBASE

    def fourws_ode(self, q, u, qdot):
        theta = q[2]
        v = u[0]
        sf = u[1]
        sr = u[2]
        qdot[0] = v * math.cos(theta)
        qdot[1] = v * math.sin(theta)
        qdot[2] = v * (math.tan(sf) - math.tan(sr)) / OMPL.WHEELBASE

    def is_state_valid(self, state):
        car_polygon = self.car.polygon(state.getX(), state.getY(), state.getYaw())
       
        # For debug
        # for i, boundary in enumerate(self.parking_lot.obstacles):
        #     if boundary.contains(car_polygon):
        #         print(f"State collides with boundary {i}")
        #         try:
        #             x, y = car_polygon.exterior.xy
        #             plt.plot(x, y, label='Car')
        #             bx, by = boundary.exterior.xy
        #             plt.plot(bx, by, label=f'Boundary {i}')
        #             plt.legend()
        #             plt.title(f"Collision with boundary {i}")
        #             plt.show()
        #         except Exception as e:
        #             print(f"Plotting failed: {e}")
        #     else: 
        #         print("state does not collide")
        
        if not self.parking_lot.lot_boundary.contains(car_polygon):
            return False
        # Collision with any parked car
        return all(not car_polygon.intersects(obstacle) for obstacle in self.parking_lot.obstacles)
    
    def create_planner(self, model: str, runtime: float = 60.0):
        space = self.make_space()
        dim = 2 if model == "bicycle" else 3
        cspace = oc.RealVectorControlSpace(space, dim)

        cb = ob.RealVectorBounds(dim)
        cb.setLow(0, -1.5)
        cb.setHigh(0, 1.5)             # ±1.5 m/s
        cb.setLow(1, -0.523)
        cb.setHigh(1, 0.523)           # ±30° steering
        if dim == 3:
            cb.setLow(2, -0.523)
            cb.setHigh(2, 0.523)
        cspace.setBounds(cb)

        ss = oc.SimpleSetup(cspace)
        ss.setStateValidityChecker(ob.StateValidityCheckerFn(self.is_state_valid))

        ode_fn = self.bicycle_ode if model == "bicycle" else self.fourws_ode
        solver = oc.ODEBasicSolver(ss.getSpaceInformation(), oc.ODE(ode_fn))
        ss.setStatePropagator(oc.ODESolver.getStatePropagator(solver))

        si = ss.getSpaceInformation()
        si.setMinMaxControlDuration(1, 20)
        si.setPropagationStepSize(0.1)

        # ------------------ Planner wrapper ----------------------------------------
        start = ob.State(space)
        start_x = 5.0
        # start_y = self.parking_lot.Y_MIN + self.car.CAR_LEN / 2 + 0.5  # keep entire car inside bounds
        start_y = 6.0
        start().setX(start_x)
        start().setY(start_y)  # just below the first row
        start().setYaw(math.pi * 0.95)  # facing "up"

        # goal_cx = - (self.parking_lot.N_SPOTS / 2 - 0.5 - self.parking_lot.GOAL_COL) * self.parking_lot.SLOT_W
        # goal_cy = self.parking_lot.ROWS_Y[self.parking_lot.GOAL_ROW]
        goal_cx = 37
        goal_cy = 2.5
        goal = ob.State(space)
        goal().setX(goal_cx)
        goal().setY(goal_cy)
        goal().setYaw(math.pi)

        ss.setStartAndGoalStates(start, goal, 0.25)
        ss.getProblemDefinition().setOptimizationObjective(ob.PathLengthOptimizationObjective(si))
        ss.setPlanner(oc.SST(si))

        if not ss.solve(runtime):
            raise RuntimeError(f"{model}: planning failed")

        path = ss.getSolutionPath()
        path.interpolate()
        coords = [(st.getX(), st.getY(), st.getYaw()) for st in path.getStates()]
        length = sum(
            math.hypot(coords[i][0] - coords[i - 1][0], coords[i][1] - coords[i - 1][1])
            for i in range(1, len(coords))
        )
        return coords, length