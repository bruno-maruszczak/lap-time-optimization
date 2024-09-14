from path import Path
import numpy as np
from math import sqrt


GRAV = 9.81  # ms^-2

class VehicleBase:
    """Vehicle parameters and behaviour."""

    def __init__(self):
        pass

    def load_params(self, filepath):
        pass

    def traction(self, velocity, curvature):
        raise NotImplementedError("Subclasses should implement this method")

    def engine_force(self, velocity, gear=None):
        raise NotImplementedError("Subclasses should implement this method")

    def get_velocity_profile(self, path: Path, samples):
        s_in = samples
        s_max = path.length if path.closed else None
        k_in = path.find_curvature_at_s(s_in)

        # self.limit_local_velocities(k)
        v_local = np.sqrt(self.vehicle.cof * GRAV / k_in)

        
        # self.limit_acceleration(k)
        # Start at slowest point
        shift = -np.argmin(v_local)
        s = np.roll(s_in, shift)
        v = np.roll(v_local, shift)
        k = np.roll(k_in, shift)

        # Limit according to acceleration
        for i in range(s.size):
            wrap = i == (shift % s.size)
            if wrap and s_max is None:
                continue
            if v[i] > v[i-1]:
                traction = self.traction(v[i-1], k[i-1])
                force = min(self.engine_force(v[i-1]), traction)
                accel = force / self.mass
                ds = self.s_max - s[i-1] if wrap else s[i] - s[i-1]
                vlim = sqrt(v[i-1]**2 + 2*accel*ds)
                v[i] = min(v[i], vlim)

        # Reset shift and return
        v_acclim = np.roll(v, -shift)
        
        # self.limit_deceleration(k)
        
        # Start at slowest point, move backwards
        shift = -np.argmin(v_local)
        s = np.flip(np.roll(s_in, shift), 0)
        k = np.flip(np.roll(k_in, shift), 0)
        v = np.flip(np.roll(v_local, shift), 0)

        # Limit according to deceleration
        for i in range(s.size):
            wrap = i == (-shift)
            if wrap and s_max is None:
                continue
            if v[i] > v[i-1]:
                traction = self.traction(v[i-1], k[i-1])
                decel = traction / self.mass
                ds = s_max - s[i] if wrap else s[i-1] - s[i]
                vlim = sqrt(v[i-1]**2 + 2*decel*ds)
                v[i] = min(v[i], vlim)

        # Reset shift/flip and return
        v_declim = np.roll(np.flip(v, 0), -shift)
        
        return np.minimum(v_acclim, v_declim)
