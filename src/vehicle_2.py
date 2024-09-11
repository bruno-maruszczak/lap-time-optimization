from math import sqrt
import json
import numpy as np

GRAV = 9.81  # m/s^2

###############################################################################


class Vehicle2:
    """Vehicle parameters and behaviour."""

    def __init__(self, path):
        """Load vehicle data from JSON file."""
        vehicle_data = json.load(open(path))
        self.rotational_inertia = vehicle_data["rotational_inertia"]
        self.name = vehicle_data["name"]
        self.mass = vehicle_data["mass"]
        self.length_f = vehicle_data["length_f"]
        self.A_f = vehicle_data["frontTire"]["A_f"]
        self.B_f = vehicle_data["frontTire"]["B_f"]
        self.C_f = vehicle_data["frontTire"]["C_f"]
        self.length_r = vehicle_data["length_r"]
        self.A_r = vehicle_data["rearTire"]["A_r"]
        self.B_r = vehicle_data["rearTire"]["B_r"]
        self.C_r = vehicle_data["rearTire"]["C_r"]
        self.Cr_0 = vehicle_data["Cr_0"]
        self.Cr_2 = vehicle_data["Cr_2"]
        self.ptv = vehicle_data["ptv"]
        self.T = vehicle_data["T"]
        self.C_m = vehicle_data["C_m"]
        self.friction_coef = self.friction_coefficient()
        print("[ Imported {} ]".format(self.name))

    def friction_coefficient(self):
        # TODO:
        # function to calculate friction coefficient
        # need that for limit_local_velocities method in VelocityProfile2 class in velocity2.py
        # there is a comment there explaining more.
        pass

    def engine_force(self, velocity, gear=None):
        # we need function to calculate engine force for a given velocity
        # our force is constant, because Torque is constatnt. In order to be prune we would need to calculate F_total  = F_long + F_lat
        # F_long = Fx = CmT - Cr0 - Cr2 * vx^2
        # where CmT is engine force, Cr0 is rolling resistance, Cr2 is air resistance, vx is velocity in longitudinal direction
        # F_lat = FyF + FyR = FNF * DF * sin(CF * arctan(BF * alphaF)) + FNR * DR * sin(CR * arctan(BR * alphaR))
        # TODO: as for me, engine_force is constant for every velocity, there i no need to calculate it
        F_long = self.ptv - self.Cr_0 - self.Cr_2 * velocity**2
        """Map current velocity to force output by the engine."""
        return np.interp(velocity, self.engine_profile[0], self.engine_profile[1])

    def traction(self, velocity):
        """Determine remaining traction when negotiating a corner."""
        # f_long = Fx = CmT - Cr0 - Cr2 * vx^2
        # where CmT is engine force (Torque[-1;1]*parameter), Cr0 is rolling resistance, Cr2 is air resistance, vx is velocity in longitudinal direction
        # and I just need f_long to return correct value in this function. 
        # TODO: need velocity in longitudinal direction!!! how to get it?
        f_long = self.T * self.C_m - self.Cr_0 - self.Cr_2 * velocity**2
        if f <= f_lat:
            return 0
        # ramaining traction in longitudinal direction
        return f_long
