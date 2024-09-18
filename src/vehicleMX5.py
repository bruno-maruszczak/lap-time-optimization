from math import sqrt
import json
import numpy as np
import re

GRAV = 9.81  # m/s^2

###############################################################################


class VehicleMX5:
    """Vehicle parameters and behaviour."""

    def __init__(self, vehicle_filepath):
        """Load vehicle data from JSON file."""
        self.load_params(vehicle_filepath)
        print("[ Imported {} ]".format(self.name))

    def engine_force(self, velocity, gear=None):
        """maximum accelerating force in longitudinal direction"""
        return (self.T * self.C_m) - self.Cr_0 - (self.Cr_2 * (velocity**2))

    def traction(self, v, k, lam=1.0):
        """Determine remaining traction when negotiating a corner."""
        # vx is velocity in longitudinal direction, it is our v_resultant in path coordinate system
        #B = (self.B_f + self.B_r)*0.5
        #C = (self.C_f + self.C_r)*0.5
        D = (self.D_f + self.D_r)*0.5
        
        #alpha = np.arctan(0.0)
        Fn = self.mass * GRAV
        #F_lat = Fn * D * np.sin(C * np.arctan(B * alpha))
        F_max = lam*D*Fn
        F_lat = self.mass * v*v * k
        if F_max <= F_lat:
            return 0
        return sqrt(F_max**2 - F_lat**2)
    
    def remove_comments(self, json_str):
        # Remove single-line comments
        json_str = re.sub(r'//.*', '', json_str)
        # Remove multi-line comments
        json_str = re.sub(r'/\*.*?\*/', '', json_str, flags=re.DOTALL)
        return json_str
    
    def load_params(self, vehicle_filepath):
        """Load vehicle data from JSON file."""
        with open(vehicle_filepath, 'r') as f:
            json_str = f.read()
            json_str_clean = self.remove_comments(json_str)
            data = json.loads(json_str_clean)
            self.rotational_inertia = data["rotational_inertia"]
            self.name = data["name"]
            self.mass = data["mass"]
            self.length_f = data["length_f"]
            self.length_r = data["length_r"]
            # Pacejka tire parameters:
            # front tire:
            self.B_f = data["frontTire"]["B_f"]
            self.C_f = data["frontTire"]["C_f"]
            self.D_f = data["frontTire"]["D_f"]
            # rear tire:
            self.B_r = data["rearTire"]["B_r"]
            self.C_r = data["rearTire"]["C_r"]
            self.D_r = data["rearTire"]["D_r"]
            # mechanical transmision
            self.C_m = data["control"]["C_m"]
            # rolling resistance
            self.Cr_0 = data["Cr_0"]
            # resistance proportional to square of velocity
            self.Cr_2 = data["Cr_2"]
            self.ptv = data["ptv"]

            # Engine control inputs and parameters:
            # forward/backward direction
            self.T = data["control"]["T"]
            self.friction_coef = data["control"]["lambda"]
            # eliptical parameter for friction ellipse
            self.ro_long = data["control"]["ro_long"]
