from shapely.geometry import Polygon
import numpy as np
from shapely.affinity import rotate, translate
import math

class ParkingLot:
    def __init__(self):
        # Roughly life‑sized, metres.
        self.SLOT_W   = 2.5          # parking‑slot width  (x‑direction)
        self.SLOT_L   = 5            # parking‑slot length (y‑direction)
        self.AISLE_W  = 6.0          # drive aisle between two facing rows
        self.N_SPOTS  = 8            # columns per row
        self.ROWS_Y   = [self.SLOT_L / 2, self.SLOT_L + self.AISLE_W + self.SLOT_L / 2]  # centre‑lines of the two rows

        # Choose the *goal* slot (row index 0 = front row, 1 = back row)
        self.GOAL_ROW = 0
        self.GOAL_COL = 3  # zero‑based index in the row (so fourth slot from left)

        # Bounds of the lot (pad a little for margins)
        self.X_MIN = - (self.N_SPOTS/2) * self.SLOT_W - 9
        self.X_MAX =   (self.N_SPOTS/2) * self.SLOT_W + 9
        # Extend downward so the whole car fits at the starting pose
        self.Y_MIN = -10.0  # ample space below the rows
        self.Y_MAX = self.ROWS_Y[1] + self.SLOT_L/2 + 10.0
        self.PLOT_BOUNDS = [self.X_MIN, self.X_MAX, self.Y_MIN, self.Y_MAX]

        

        # Parked‑car rectangles (obstacles) and slot outlines
        self.parked_cars = []   # Polygons for collision checking
        self.slot_lines  = []   # Each slot as a Polygon outline for plotting only
        self.obstacles = []

        # Bounding Lines - real dimensions of lines on a parking lot
        self.line_width = 0.1

        # Lot boundary (outline only; we allow driving over the white lines)
        self.lot_boundary = Polygon([
            (self.PLOT_BOUNDS[0], self.PLOT_BOUNDS[2]), (self.PLOT_BOUNDS[1], self.PLOT_BOUNDS[2]),
            (self.PLOT_BOUNDS[1], self.PLOT_BOUNDS[3]), (self.PLOT_BOUNDS[0], self.PLOT_BOUNDS[3])
        ])

    def init_parking_slots(self, CAR_LEN, CAR_WID):
        for row_idx, cy in enumerate(self.ROWS_Y):
         # x‑coordinate of the centre of the *leftmost* slot
            first_cx = - (self.N_SPOTS/2 - 0.5) * self.SLOT_W
            for col in range(self.N_SPOTS):
                cx = first_cx + col * self.SLOT_W
                # Slot outline (thin rectangle the size of the slot)
                slot = Polygon([  # rectangle corners (counter‑clockwise)
                    (cx - self.SLOT_W/2, cy - self.SLOT_L/2),
                    (cx + self.SLOT_W/2, cy - self.SLOT_L/2),
                    (cx + self.SLOT_W/2, cy + self.SLOT_L/2),
                    (cx - self.SLOT_W/2, cy + self.SLOT_L/2)
                ])
                self.slot_lines.append(slot)
                # Skip the goal slot so it remains empty
                if row_idx == self.GOAL_ROW and col == self.GOAL_COL:
                    continue
        
                # Create a parked car inside this slot (slightly inset)
                inset = 0.15
                pcar = Polygon([
                    (cx - (CAR_WID/2), cy - (CAR_LEN/2)),
                    (cx + (CAR_WID/2), cy - (CAR_LEN/2)),
                    (cx + (CAR_WID/2), cy + (CAR_LEN/2)),
                    (cx - (CAR_WID/2), cy + (CAR_LEN/2))
                ])
                self.parked_cars.append(pcar)
                self.obstacles = self.parked_cars


class ParkingLotBitMap:
    def __init__():
        


class Car:
    def __init__(self):
        # Car that will move (our robot)
        self.CAR_LEN = 4.5   # m
        self.CAR_WID = 2.0   # m  (larger relative to slot for realism)

    # ------------------ Helper to make robot car polygon -----------------------
    def polygon(self, x: float, y: float, yaw: float) -> Polygon:
        rect = Polygon([
            (-self.CAR_LEN/2, -self.CAR_WID/2), (self.CAR_LEN/2, -self.CAR_WID/2),
            (self.CAR_LEN/2,  self.CAR_WID/2), (-self.CAR_LEN/2,  self.CAR_WID/2)
        ])
        rect = rotate(rect, math.degrees(yaw), origin=(0, 0), use_radians=False)
        return translate(rect, x, y)