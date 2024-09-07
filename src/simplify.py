import argparse
import os
from enum import IntEnum, unique
from plot import plot_corners, plot_path, plot_trajectory
from track import Track
from trajectory import Trajectory
from vehicle import Vehicle

###############################################################################
## Argument parsing

@unique
class SplineType(IntEnum):
  POLY = 0
  TRIG = 1
  HYP = 2

parser = argparse.ArgumentParser(description='Track spline simplification method comparison')
parser.add_argument('track',
  nargs=1, type=str,
  help='path to JSON containing track data'
)
parser.add_argument('vehicle',
  nargs=1, type=str,
  help='path to JSON containing vehicle data'
)

methods = parser.add_argument_group('spline').add_mutually_exclusive_group(required=True)
methods.add_argument('--poly',
  action='store_const', dest='spline', const=SplineType.POLY,
  help='Polynomial B-Spline interpolation'
)
methods.add_argument('--trig',
  action='store_const', dest='spline', const=SplineType.TRIG,
  help='Trigonometric spline interpolation'
)
methods.add_argument('--hyp',
  action='store_const', dest='spline', const=SplineType.HYP,
  help='Hyperbolic spline interpolation'
)

#parser.add_argument('--plot-corners',
#  action='store_true', dest='plot_corners',
#  help='plot detected corners'
#)


args = parser.parse_args()

###############################################################################
## Generation

track = Track(args.track[0])
vehicle = Vehicle(args.vehicle[0])
trajectory = Trajectory(track, vehicle)


#if args.method is Method.CURVATURE:
#  print("[ Minimising curvature ]")
#  run_time = trajectory.minimise_curvature()
#elif args.method is Method.COMPROMISE:
#  print("[ Minimising optimal compromise ]")
#  run_time = trajectory.minimise_optimal_compromise()
#  print("  epsilon = {:.4f}".format(trajectory.epsilon))
#else:
#  raise ValueError("Did not recognise args.method {}".format(args.method))

#print("[ Computing lap time ]")
#trajectory.update_velocity()
#lap_time = trajectory.lap_time()
#
#print()
#print("=== Results ==========================================================")
#print("Lap time = {:.3f}".format(lap_time))
#print("Run time = {:.3f}".format(run_time))
#print("======================================================================")
#print()
#
################################################################################
### Plotting
#
#method_dirs = ['curvature', 'compromise', 'laptime', 'sectors', 'estimated']
#plot_dir = os.path.join(
#  os.path.dirname(__file__), '..', 'data', 'plots', track.name,
#  method_dirs[args.method]
#)
#if not os.path.exists(plot_dir): os.makedirs(plot_dir)
#
#if args.plot_corners or args.plot_all:
#  plot_corners(
#    os.path.join(plot_dir, "corners." + args.ext),
#    track.left, track.right, track.mid.position(trajectory.s),
#    track.corners(trajectory.s, K_MIN, PROXIMITY, LENGTH)[1]
#  )
#
#if args.plot_path or args.plot_all:
#  plot_path(
#    os.path.join(plot_dir, "path." + args.ext),
#    track.left, track.right, trajectory.path.position(trajectory.s),
#    trajectory.path.controls
#  )
#
#if args.plot_trajectory or args.plot_all:
#  plot_trajectory(
#    os.path.join(plot_dir, "trajectory." + args.ext),
#    track.left, track.right, trajectory.path.position(trajectory.s),
#    trajectory.velocity.v
#  )
#print(track.size)
