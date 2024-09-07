import numpy as np
from scipy.interpolate import splev, splprep
from scipy.integrate import cumulative_trapezoid
import matplotlib.pyplot as plt

class Path:
    """Wrapper for scipy.interpolate.BSpline."""

    def __init__(self, controls, closed, n_samples=1000):
        """Construct a spline through the given control points."""
        self.controls = controls
        self.closed = closed
        self.dists = cumulative_distances(controls)
        self.spline, _ = splprep(controls, u=self.dists, k=3, s=0, per=self.closed)
        self.length = self.dists[-1]

        # sample u, to calculate transformation u -> arc_length (return u given arc length)
        self.u_sampled = np.linspace(0, 1, n_samples)
        self.arc_lengths_sampled = self.calculate_arc_length()

    def calculate_arc_length(self):
        """
        Calculate the cumulative arc length for each parameter value u.

        Returns:
        - arc_lengths: array of arc length values corresponding to u_fine
        """
        # Evaluate first derivatives dx/du and dy/du
        dx_du, dy_du = splev(self.u_sampled, self.spline, der=1)
        
        # Calculate differential arc length: ds = sqrt((dx/du)^2 + (dy/du)^2)
        ds_du = np.sqrt(dx_du**2 + dy_du**2)
        
        # Compute cumulative arc length by integrating ds/du
        arc_lengths = cumulative_trapezoid(ds_du, self.u_sampled, initial=0)
        
        return arc_lengths

    def find_u_given_s(self, s):
        """
        Find the paramter u of curve that represents given travelled arc lengths (from beginning)

        Parameters:
        - s : desired arc lengths (distance from beginning of the curve)

        Returns:
        - u : parametrs u that represent given arc lengths on the curve
        """
        u = np.interp(s, self.arc_lengths_sampled, self.u_sampled)
        return u

    def find_curvature_at_s(self, s):
        """
        Find the curvature at a given arc length s using the precomputed arc lengths and spline representation.
        
        Parameters:
        - s: desired arc length (distance from beginning of the curve)
        
        Returns:
        - curvature: curvature at the given arc length s
        """
        # Interpolate to find the corresponding u for the given arc length s
        u = self.find_u_given_s(self, s)
        
        # Calculate curvature at the interpolated u value
        curvature = self.curvature(u)
        
        return curvature
    
    def position(self, s=None):
        """Returns x-y coordinates of sample points."""
        if s is None:
            return self.controls
        x, y = splev(s, self.spline)
        return np.array([x, y])

    def curvature(self, u=None):
        """
        Calculate curvature of spline at given parameter u value
        
        Parameters:
        - u: parameter values at which to compute the curvature
        
        Returns:
        - curvature: the curvature values at each parameter value u
        """

        if u is None:
            u = self.dists

        # First derivatives dx/du and dy/du
        dx_du, dy_du = splev(u, self.spline, der=1)
        
        # Second derivatives d2x/du2 and d2y/du2
        d2x_du2, d2y_du2 = splev(u, self.spline, der=2)
        
        # Curvature formula: kappa(u) = |dx/du * d2y/du2 - dy/du * d2x/du2| / (dx/du^2 + dy/du^2)^(3/2)
        curvature = np.abs(dx_du * d2y_du2 - dy_du * d2x_du2) / (dx_du**2 + dy_du**2)**(3/2)
        
        return curvature

    def gamma2(self, u=None):
        """Returns the sum of the squares of sample curvatures, Gamma^2."""
        if u is None:
            u = self.dists

        # First derivatives dx/du and dy/du
        dx_du, dy_du = splev(u, self.spline, der=1)
        
        # Second derivatives d2x/du2 and d2y/du2
        d2x_du2, d2y_du2 = splev(u, self.spline, der=2)
        
        # Curvature formula: kappa(u) = |dx/du * d2y/du2 - dy/du * d2x/du2| / (dx/du^2 + dy/du^2)^(3/2)
        curvature = (dx_du * d2y_du2 - dy_du * d2x_du2) / (dx_du**2 + dy_du**2)**(3/2)
        curvature = curvature**2
        return np.sum(curvature)

###############################################################################

def cumulative_distances(points):
    """Returns the cumulative linear distance at each point."""
    d = np.cumsum(np.linalg.norm(np.diff(points, axis=1), axis=0))
    return np.append(0, d)
