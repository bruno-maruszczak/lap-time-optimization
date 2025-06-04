import numpy as np
import do_mpc
from path import Path
import re

import casadi as ca
import json

from mpc.track import Track

class VehicleModel:
    def __init__(self, params_file_path, track: Track):
        self.track = track
        self.optimal_path = self.track.optimal_path

        self.g = 9.81
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
        self.width = 1.0
        self.load_params(params_file_path)
        self.model = self.create_model()
        self.model.setup()

    def remove_comments(self, json_str):
        # Remove single-line comments
        json_str = re.sub(r'//.*', '', json_str)
        # Remove multi-line comments
        json_str = re.sub(r'/\*.*?\*/', '', json_str, flags=re.DOTALL)
        return json_str
    
    def load_params(self, path):
        """Load vehicle data from JSON file."""
        with open(path, 'r') as f:
            json_str = f.read()
            json_str_clean = self.remove_comments(json_str)
            data = json.loads(json_str_clean)
            self.rotational_inertia = data["rotational_inertia"]
            self.name = data["name"]
            self.mass = data["mass"]
            self.length_f = data["length_f"]
            self.length_r = data["length_r"]
            self.width = data["width"]

            self.B_f = data["frontTire"]["B_f"]
            self.C_f = data["frontTire"]["C_f"]
            
            self.B_r = data["rearTire"]["B_r"]
            self.C_r = data["rearTire"]["C_r"]

            self.C_m = data["control"]["C_m"]
            self.Cr_0 = data["Cr_0"]
            self.Cr_2 = data["Cr_2"]
            self.ptv = data["ptv"]

    def k(self, s):
        return self.optimal_path.find_curvature_at_s(s)

    
    def get_lateral_constraint(self, s, n, mu):
        length = self.length_f + self.length_r
        width = self.width

        NL = self.track.find_dist_to_band(s, "left")
        NR = self.track.find_dist_to_band(s, "right")
        
        left_constraint = n - length * 0.5 * ca.sin(ca.sign(mu) * mu) + width * 0.5 * ca.cos(mu) - NL
        right_constraint = - n + length * 0.5 * ca.sin(ca.sign(mu) * mu) + width * 0.5 * ca.cos(mu) - NR

        # TODO what are these for?
        # left_constraint_trunc = ca.if_else(left_constraint > NL, left_constraint - NL, 0.0)
        # right_constraint_trunc = ca.if_else(right_constraint > NR, right_constraint - NR, 0.0)

        return left_constraint, right_constraint

    def get_traction_ellipse_constraint(self, throttle, vx, vy, r, steering_angle, rho=1.0, alpha=1.0):
        long = rho * 0.5 * self.get_motor_force(throttle)
        af, ar = self.get_slip_angles(vx, vy, r, steering_angle)
        Fy_f, Fy_r = self.get_lateral_forces(af, ar)
        Df = alpha * self.D_f
        Dr = alpha * self.D_r
        ellipse_f = long*long + Fy_f*Fy_f - Df*Df
        ellipse_r = long*long + Fy_r*Fy_r - Dr*Dr

        # TODO what are these for?
        # ellipse_f_trunc = ca.if_else(ellipse_f > 0.0, ellipse_f, 0.0)
        # ellipse_r_trunc = ca.if_else(ellipse_r > 0.0, ellipse_r, 0.0)

        return ellipse_f, ellipse_r

    def get_slip_angles(self, vx, vy, r, steering_angle):
        alpha_f = ca.atan2((vy + self.length_f*r),vx) - steering_angle
        alpha_r = ca.atan2((vy - self.length_r*r),vx)
        return alpha_f, alpha_r

    def get_lateral_forces(self, alpha_f, alpha_r):
        Fn_f = self.length_r * self.mass * self.g / (self.length_f + self.length_r)
        Fn_r = self.length_f * self.mass * self.g / (self.length_f + self.length_r)

        # Fy_f = Fn_f * self.D_f * ca.sin(self.C_f * ca.atan(self.B_f * alpha_f))
        # Fy_r = Fn_r * self.D_r * ca.sin(self.C_r * ca.atan(self.B_r * alpha_r))
        Fy_f = -Fn_f * self.D_f * ca.sin(self.C_f * ca.atan(self.B_f * alpha_f))
        Fy_r = -Fn_r * self.D_r * ca.sin(self.C_r * ca.atan(self.B_r * alpha_r))
        return Fy_f, Fy_r
    
    def get_motor_force(self, throttle):
        return self.C_m * throttle

    def sdot(self):
        x = self.model.x
        sdot = (x['vx']*ca.cos(x['mu']) - x['vy']*ca.sin(x['mu'])) / (1 - x['n'] * self.k(x['s']))
        return sdot

    def B(self, q_B):
        x = self.model.x
        b_dyn = ca.atan(x['vy']/x['vx'])
        b_kin = ca.atan(x['steering_angle']*self.length_r / (self.length_f + self.length_r))
        return q_B * (b_dyn - b_kin)**2

    def create_model(self) -> do_mpc.model.Model:
        """
        Setups all variables, inputs, parameters of an MPC model.
        """

        model_type = 'continuous'
        model = do_mpc.model.Model(model_type, 'MX')

        s = model.set_variable("_x", 's', shape=(1,1))
        n = model.set_variable("_x", 'n', shape=(1,1))
        mu = model.set_variable("_x", 'mu', shape=(1,1))

        vx = model.set_variable('_x', 'vx', shape=(1,1))
        vy = model.set_variable('_x', 'vy', shape=(1,1))
        r = model.set_variable('_x', 'r', shape=(1,1))

        steering_angle = model.set_variable('_x', 'steering_angle', shape=(1,1))
        throttle = model.set_variable('_x', 'throttle', shape=(1,1))

        steering_angle_change = model.set_variable('_u', 'steering_angle_change', shape=(1,1))
        throttle_change = model.set_variable('_u', 'throttle_change', shape=(1,1))

        sdot = (vx*ca.cos(mu) - vy*ca.sin(mu)) / (1 - n * self.k(s))
        
        # Tires

        alpha_f, alpha_r = self.get_slip_angles(vx, vy, r, steering_angle)

        Fy_f, Fy_r = self.get_lateral_forces(alpha_f, alpha_r)
        
        Fx =  self.get_motor_force(throttle) - self.Cr_0 - self.Cr_2 * vx * vx

        rt = (ca.tan(steering_angle)*vx)/(self.length_f + self.length_r)
        #Mtv = self.ptv * (rt - r)
        Mtv = 0.0

        model.set_rhs('s', sdot)
        model.set_rhs('n', 
            vx*ca.sin(mu) + vy*ca.cos(mu)
        )
        model.set_rhs('mu',
            r - self.k(s)*sdot                    
        )
        model.set_rhs('vx',
            (1.0 / self.mass) * (Fx - Fy_f * ca.sin(steering_angle) + self.mass * vy * r)
        )
        model.set_rhs('vy', 
            (1.0 / self.mass) * (Fy_r + Fy_f * ca.cos(steering_angle) - self.mass * vx * r)
        )
        model.set_rhs('r',
            (1.0 / self.rotational_inertia) * (Fy_f * self.length_f * ca.cos(steering_angle) - Fy_r * self.length_r + Mtv)
        )
        model.set_rhs('throttle', throttle_change)
        model.set_rhs('steering_angle', steering_angle_change)
        
        return model