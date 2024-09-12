from math import sqrt
import json
import numpy as np
import re

GRAV = 9.81  # m/s^2

###############################################################################


class Vehicle2:
    """Vehicle parameters and behaviour."""

    def __init__(self, filepath):
        """Load vehicle data from JSON file."""
        with open(filepath, 'r') as f:
            json_str = f.read()
            json_str_clean = self.remove_comments(json_str)
            vehicle_data = json.loads(json_str_clean)
            vehicle_data = json.load(open(filepath))
            self.rotational_inertia = vehicle_data["rotational_inertia"]
            self.name = vehicle_data["name"]
            self.mass = vehicle_data["mass"]
            self.length_f = vehicle_data["length_f"]
            self.length_r = vehicle_data["length_r"]
            self.B_f = vehicle_data["frontTire"]["B_f"]
            self.C_f = vehicle_data["frontTire"]["C_f"]
            self.D_f = vehicle_data["frontTire"]["D_f"]
            self.B_r = vehicle_data["rearTire"]["B_r"]
            self.C_r = vehicle_data["rearTire"]["C_r"]
            self.D_r = vehicle_data["rearTire"]["D_r"]
            self.Cr_0 = vehicle_data["Cr_0"]
            self.Cr_2 = vehicle_data["Cr_2"]
            self.ptv = vehicle_data["ptv"]
            # // Engine control inputs and parameters:
            self.T = vehicle_data["control"]["T"]
            self.C_m = vehicle_data["control"]["C_m"]
            self.friction_coef = vehicle_data["control"]["lambda"]
            self.ro_long = vehicle_data["control"]["ro_long"]
        print("[ Imported {} ]".format(self.name))

    def engine_force(self, velocity, gear=None):
        """maximum engine force in longitudinal direction"""
        F_M = self.C_m * self.T
        return F_M

    def traction(self, velocity):
        """Determine remaining traction when negotiating a corner."""
        # vx is velocity in longitudinal direction, we just have v wypadkowe
        f_long = (self.T * self.C_m) - self.Cr_0 - (self.Cr_2 * (velocity**2))
        # print(f"traction: {f_long}")
        return f_long
    
    def remove_comments(self, json_str):
        # Remove single-line comments
        json_str = re.sub(r'//.*', '', json_str)
        # Remove multi-line comments
        json_str = re.sub(r'/\*.*?\*/', '', json_str, flags=re.DOTALL)
        return json_str
