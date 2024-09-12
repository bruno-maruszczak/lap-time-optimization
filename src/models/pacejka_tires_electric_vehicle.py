import numpy as np
import do_mpc
from path import Path
import json

class PacejkaTiresElectricVehicle:
    def __init__(self, track_line: Path):
        self.track_line = track_line
        self.mass = 1.0
        self.rotational_inertia = 1.0
        self.length_f = 1.0
        self.B_f = 1.0
        self.C_f = 1.0
        self.D_f = 1.0
        self.length_r = 1.0
        self.B_r = 1.0
        self.C_r = 1.0
        self.D_r = 1.0
        self.Cr_0 = 0.0
        self.Cr_2 = 0.0
        self.ptv = 0.0
        self.model = self.create_model()

    def load_params(self, filepath):
        """Load vehicle data from JSON file."""
        data = json.load(open(filepath))
        self.rotational_inertia = data["rotational_inertia"]
        self.name = data["name"]
        self.mass = data["mass"]
        self.length_f = data["length_f"]
        self.A_f = data["frontTire"]["A_f"]
        self.B_f = data["frontTire"]["B_f"]
        self.C_f = data["frontTire"]["C_f"]
        self.length_r = data["length_r"]
        self.A_r = data["rearTire"]["A_r"]
        self.B_r = data["rearTire"]["B_r"]
        self.C_r = data["rearTire"]["C_r"]
        self.Cr_0 = data["Cr_0"]
        self.Cr_2 = data["Cr_2"]
        self.ptv = data["ptv"]


    def k(self, s):
        return self.track_line.find_curvature_at_s(s)

    def create_model(self) -> do_mpc.model.Model:
        """
        Setups all variables, inputs, parameters of an MPC model.
        """

        model_type = 'continuous'
        model = do_mpc.model.Model(model_type)

        s = model.set_variable("_x", 's', shape=(1,1))
        n = model.set_variable("_x", 'n', shape=(1,1))
        mu = model.set_variable("_x", 'mu', shape=(1,1))

        vx = model.set_variable('_x', 'vx', shape=(3,1))
        vy = model.set_variable('_x', 'vy', shape=(3,1))
        r = model.set_variable('_x', 'r', shape=(1,1))

        steering_angle = model.set_variable('_x', 'steering_angle', shape=(1,1))
        throttle = model.set_variable('_x', 'throttle', shape=(1,1))

        steering_angle_change = model.set_variable('_u', 'steering_angle_change', shape=(1,1))
        throttle_change = model.set_variable('_u', 'throttle_change', shape=(1,1))


        sdot = (vx*np.cos(mu) - vy*np.sin(mu)) / (1 - n * self.k(s))
        
        # Tires
        alpha_f = np.arctan((vy + self.length_f*r)/vx) - steering_angle
        alpha_r = np.arctan((vy - self.length_r*r)/vx)
        g = 9.81
        Fn_f = self.length_r * self.mass * g / (self.length_f + self.length_r)
        Fn_r = self.length_f * self.mass * g / (self.length_f + self.length_r)
        Fy_f = Fn_f * self.D_f * np.sin(self.C_f * np.arctan(self.B_f * alpha_f))
        Fy_r = Fn_r * self.D_r * np.sin(self.C_r * np.arctan(self.B_r * alpha_r))
        
        Fx = self.C_m * throttle - self.Cr_0 - self.Cr_2 * vx * vx

        rt = (np.tan(steering_angle)*vx)/(self.length_f + self.length_r)
        Mtv = self.ptv * (rt - r)


        model.set_rhs('s', sdot)
        model.set_rhs('n', 
            vx*np.sin(mu) + vy*np.cos(mu)
        )
        model.set_rhs('mu',
            r - self.k(s)*sdot                    
        )
        model.set_rhs('vx',
            (1.0 / self.mass) * (Fx - Fy_f * np.sin(steering_angle) + self.mass * vy * r)
        )
        model.set_rhs('vy', 
            (1.0 / self.mass) * (Fy_r + Fy_f * np.cos(steering_angle) - self.mass * vx * r)
        )
        model.set_rhs('r',
            (1.0 / self.rotational_inertia) * (Fy_f * self.length_f * np.cos(steering_angle) - Fy_r * self.length_r + Mtv)
        )
        model.set_rhs('throttle', throttle_change)
        model.set_rhs('steering_angle', steering_angle_change)


        return model