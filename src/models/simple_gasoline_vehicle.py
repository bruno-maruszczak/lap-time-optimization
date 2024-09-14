from models.vehicle_base import VehicleBase

import json
import numpy as np
from math import sqrt

GRAV = 9.81  # ms^-2

###############################################################################


class SimpleGasolineVehicle(VehicleBase):
    """Vehicle parameters and behaviour."""

    def __init__(self, vehicle_filepath):
        """Load vehicle data from JSON file."""
        self.load_params(vehicle_filepath)
        
        print("[ Imported {} ]".format(self.name))

    def load_params(self, vehicle_filepath):
        vehicle_data = json.load(open(vehicle_filepath))
        self.name = vehicle_data["name"]
        self.mass = vehicle_data["mass"]
        self.friction_coef = vehicle_data["frictionCoefficient"]
        self.engine_profile = [
            vehicle_data["engineMap"]["v"],
            vehicle_data["engineMap"]["f"]
        ]

    def engine_force(self, velocity, gear=None):
        """Map current velocity to force output by the engine."""
        return np.interp(velocity, self.engine_profile[0], self.engine_profile[1])

    def traction(self, velocity, curvature):
        """Determine remaining traction when negotiating a corner."""
        f = self.cof * self.mass * GRAV
        f_lat = self.mass * velocity**2 * curvature
        if f <= f_lat:
            return 0
        return sqrt(f**2 - f_lat**2)
