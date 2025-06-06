import argparse
import os
import numpy as np
from enum import IntEnum, unique
import numpy as np
from plot import plot_corners, plot_path, plot_trajectory
from track import Track
from trajectory import Trajectory
from trajectory_bayesian_nonlinear import TrajectoryBayesianNonlinear
from vehicle import Vehicle
from vehicleMX5 import VehicleMX5
from utils import save_path_to_json, save_widths_to_json, save_velocities_to_json

###############################################################################
# Argument parsing


@unique
class Method(IntEnum):
    CURVATURE = 0
    COMPROMISE = 1
    DIRECT = 2
    COMPROMISE_SECTORS = 3
    COMPROMISE_ESTIMATED = 4
    BAYES = 5
    NONLINEAR = 6


parser = argparse.ArgumentParser(description='Racing line optimisation')
parser.add_argument('track',
                    nargs=1, type=str,
                    help='path to JSON containing track data'
                    )
parser.add_argument('vehicle',
                    nargs=1, type=str,
                    help='path to JSON containing vehicle data'
                    )
parser.add_argument('track_width',
                    nargs=1, type=float, 
                    help='float value from 0.01 to 1.0 representing percentage of the track that can be used, \
                        0.99 means that the whole track can be used, 0.0 means that the track is very narrow'
                    )
methods = parser.add_argument_group(
    'generation methods').add_mutually_exclusive_group(required=True)
methods.add_argument('--curvature',
                     action='store_const', dest='method', const=Method.CURVATURE,
                     help='minimise curvature'
                     )
methods.add_argument('--compromise',
                     action='store_const', dest='method', const=Method.COMPROMISE,
                     help='minimise an optimal length-curvature compromise'
                     )
methods.add_argument('--bayes',
                     action='store_const', dest='method', const=Method.BAYES,
                     help='minimise time via bayesian optimisation'
                     )
methods.add_argument('--nonlinear',
                     action='store_const', dest='method', const=Method.NONLINEAR,
                     help='minimise time via nonlinear optimisation'
                     )
methods.add_argument('--laptime',
                     action='store_const', dest='method', const=Method.DIRECT,
                     help='directly minimise lap time'
                     )
methods.add_argument('--sectors',
                     action='store_const', dest='method', const=Method.COMPROMISE_SECTORS,
                     help='optimise and merge sector paths'
                     )
methods.add_argument('--estimated',
                     action='store_const', dest='method', const=Method.COMPROMISE_ESTIMATED,
                     help='minimise a pre-computed length-curvature compromise'
                     )
parser.add_argument('--plot-corners',
                    action='store_true', dest='plot_corners',
                    help='plot detected corners'
                    )
parser.add_argument('--plot-path',
                    action='store_true', dest='plot_path',
                    help='plot the generated path'
                    )
parser.add_argument('--plot-trajectory',
                    action='store_true', dest='plot_trajectory',
                    help='plot the generated path with velocity gradient'
                    )
parser.add_argument('--plot-all',
                    action='store_true', dest='plot_all',
                    help='plot all relevant graphs'
                    )
parser.add_argument('--plot-format',
                    type=str, dest='ext', default='png',
                    help='file format used to save plots'
                    )
args = parser.parse_args()

###############################################################################
# Generation

track_width = args.track_width[0]
track = Track(args.track[0], track_width=track_width)
if args.vehicle[0] == "./data/vehicles/MX5.json":
    vehicle = VehicleMX5(args.vehicle[0])
else:
    vehicle = Vehicle(args.vehicle[0])
if args.method is Method.BAYES or args.method is Method.NONLINEAR:
    trajectory = TrajectoryBayesianNonlinear(track, vehicle)
else:
    trajectory = Trajectory(track, vehicle)

# Corner detection parameters
K_MIN = 0.03
PROXIMITY = 40
LENGTH = 10

if args.method is Method.CURVATURE:
    print("[ Minimising curvature ]")
    run_time = trajectory.minimise_curvature()
    print("[ Computing lap time ]")
    trajectory.update_velocity()
    lap_time = trajectory.lap_time()
elif args.method is Method.COMPROMISE:
    print("[ Minimising optimal compromise ]")
    run_time = trajectory.minimise_optimal_compromise()
    print("  epsilon = {:.4f}".format(trajectory.epsilon))
    print("[ Computing lap time ]")
    trajectory.update_velocity()
    lap_time = trajectory.lap_time()
elif args.method is Method.DIRECT:
    print("[ Minimising lap time ]")
    run_time = trajectory.minimise_lap_time()
    print("[ Computing lap time ]")
    trajectory.update_velocity()
    lap_time = trajectory.lap_time()
elif args.method is Method.COMPROMISE_SECTORS:
    print("[ Optimising sectors ]")
    run_time = trajectory.optimise_sectors(K_MIN, PROXIMITY, LENGTH)
    print("[ Computing lap time ]")
    trajectory.update_velocity()
    lap_time = trajectory.lap_time()
elif args.method is Method.COMPROMISE_ESTIMATED:
    print("[ Minimising pre-computed compromise ]")
    mask = track.corners(trajectory.s, K_MIN, PROXIMITY, LENGTH)[1]
    epsilon = 0.406 * track.avg_curvature(trajectory.s[mask])
    print("  epsilon = {:.4f}".format(epsilon))
    run_time = trajectory.minimise_compromise(epsilon)
    print("[ Computing lap time ]")
    trajectory.update_velocity()
    lap_time = trajectory.lap_time()
elif args.method is Method.BAYES:
    print("[ BAYES  ]")
    run_time = trajectory.Bayesian()
    print("[ Computing lap time ]")
    lap_time = trajectory.calcMinTime(trajectory.best)
elif args.method is Method.NONLINEAR:
    print("[ NONLINEAR ]")
    run_time = trajectory.Nonlinear()
    print("[ Computing lap time ]")
    lap_time = trajectory.calcMinTime(trajectory.best)
else:
    raise ValueError("Did not recognise args.method {}".format(args.method))

length = trajectory.path.length # dlugosc sciezki
mean_velocity = np.mean(trajectory.velocity.v)
max_velocity = np.max(trajectory.velocity.v)

print()
print("=== Results ==========================================================")
print("Lap time = {:.3f}".format(lap_time))
print("Run time = {:.3f}".format(run_time))
print("Path Length = {:.3f}".format(length))
print("Max velocity = {:.3f}".format(max_velocity))
print("Mean velocity = {:.3f}".format(mean_velocity))
print("======================================================================")
print()

###############################################################################
# Plotting

method_dirs = ['curvature', 'compromise', 'laptime', 'sectors', 'estimated', 'bayesian', 'nonlinear']
plot_dir = os.path.join(
    os.path.dirname(__file__), '..', 'data', 'plots', vehicle.name, track.name,
    method_dirs[args.method]
)
if not os.path.exists(plot_dir):
    os.makedirs(plot_dir)

if args.plot_corners or args.plot_all:
    plot_corners(
        os.path.join(plot_dir, "corners." + args.ext),
        track.old_left, track.old_right, track.mid.position(trajectory.s),
        track.corners(trajectory.s, K_MIN, PROXIMITY, LENGTH)[1]
    )

if args.plot_path or args.plot_all:
    plot_path(
        os.path.join(plot_dir, "path." + args.ext),
        track.old_left, track.old_right, trajectory.path.position(trajectory.s),
        trajectory.path.controls
    )
    save_path_to_json(plot_dir, trajectory.path.position(trajectory.s)[0], trajectory.path.position(trajectory.s)[1], f"path")

if args.plot_trajectory or args.plot_all:

    plot_trajectory(
        os.path.join(plot_dir, "trajectory." + args.ext),
        track.old_left, track.old_right, trajectory.path.position(trajectory.s),
        trajectory.velocity.v
    )

# saving left and right boundries of track to json file
save_path_to_json(plot_dir, track.old_left[0], track.old_left[1], f"left")
save_path_to_json(plot_dir, track.old_right[0], track.old_right[1], f"right")  
save_widths_to_json(plot_dir, track.widths, f"widths")
save_velocities_to_json(plot_dir, trajectory.velocity.v, f"velocities")

print(f"track size: {track.size}")
print(f"widths: {len(track.widths)}")
