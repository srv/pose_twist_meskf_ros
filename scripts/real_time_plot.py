#!/usr/bin/env python
import roslib;
import sys
import pylab
import math
import numpy as np
import weakref
import string
import random
import time
import ntpath
import os
from matplotlib import pyplot
from mpl_toolkits.mplot3d import Axes3D
import tf.transformations as tf

# Global variables
legend_edited = False
colors = ['g','r','b']
ax_gt = None
ax_odom = None
ax_meskf = None
gt_data = []

class Error(Exception):
  """ Base class for exceptions in this module. """
  pass

def check_file_len(file):
  """ Check if the file length is > 0 """
  f = open(file)
  lines = f.readlines()
  f.close()
  return len(lines) > 0

def rm_ax(ax_id):
  """ Remove the axes lines """
  if (ax_id is not None and ax_id):
    l = ax_id.pop(0)
    wl = weakref.ref(l)
    l.remove()
    del l

def real_time_plot(gt_file, odom_file, meskf_file):
  """ Function to plot the data saved into the files in real time """

  global blocking_file, legend_edited, colors, ax_gt, ax_odom, ax_meskf, gt_data

  # Remove the main axes
  rm_ax(ax_gt)
  rm_ax(ax_odom)

  # Ground truth
  if (gt_file != "" and os.path.exists(gt_file) and check_file_len(gt_file) and gt_file != "none"):
    # Open the file for some sanity checks
    f = open(gt_file)
    lines = f.readlines()
    f.close()

    # Delimiter detection
    size = lines[1].split(",")
    if (len(size) == 1):
      delimiter = ' '
    else:
      delimiter = ','

    # Check gt file type

    size = lines[1].split(delimiter)
    if (len(size) > 12):
      data = pylab.loadtxt(gt_file, delimiter=delimiter, skiprows=1, usecols=(0,5,6,7,8,9,10,11))
    else:
      data = pylab.loadtxt(gt_file, delimiter=delimiter, usecols=(0,1,2,3,4,5,6,7))

    # Plot
    if (len(data.shape) == 1):
      data = [data]
      data = np.array(data)
    ax_gt = ax.plot(data[:,1], data[:,2], data[:,3], colors[0], label='Ground Truth')

  # Load visual odometry data (saved with rostopic echo -p /stereo_odometer/odometry > file.txt)
  if (odom_file != "" and os.path.exists(odom_file) and check_file_len(odom_file)):

    # Read the data
    data = pylab.loadtxt(odom_file, delimiter=',', skiprows=1, usecols=(5,6,7,8,9,10,11))

    # Plot
    if (len(data.shape) == 1):
      data = np.array([data])
    ax_odom = ax.plot(data[:,0], data[:,1], data[:,2], colors[1], label='Visual Odometry')

  # Load meskf data (saved with rostopic echo -p /pose_twist_meskf/odometry > file.txt)
  if (meskf_file != "" and os.path.exists(meskf_file) and check_file_len(meskf_file)):

    # Read the data
    data = pylab.loadtxt(meskf_file, delimiter=',', skiprows=1, usecols=(5,6,7,8,9,10,11))

    # Plot
    if (len(data.shape) == 1):
      data = np.array([data])
    ax_meskf = ax.plot(data[:,0], data[:,1], data[:,2], colors[2], label='Meskf Output')


  # Update the plot
  pyplot.draw()

  # Show legend only once
  if (ax_odom is not None and ax_meskf is not None and legend_edited is False):
    ax.legend()
    legend_edited = True


if __name__ == "__main__":
  import argparse
  parser = argparse.ArgumentParser(
          description='Plot 3D graphics of odometry data files in real time.',
          formatter_class=argparse.ArgumentDefaultsHelpFormatter)
  parser.add_argument('ground_truth_file',
          help='file with ground truth')
  parser.add_argument('visual_odometry_file',
          help='file with visual odometry')
  parser.add_argument('meskf_file',
          help='file with meskf output')
  args = parser.parse_args()

  # Some hardcode parameters
  font = {'family' : 'Sans',
          'weight' : 'normal',
          'size'   : 14}
  pylab.rc('font', **font)

  # Init figure
  fig = pylab.figure(1)
  ax = Axes3D(fig)
  ax.grid(True)
  ax.set_title("Real-time plot")
  ax.set_xlabel("X")
  ax.set_ylabel("Y")
  ax.set_zlabel("Z")

  # Start timer for real time plot
  timer = fig.canvas.new_timer(2500)
  real_time_plot(args.ground_truth_file, args.visual_odometry_file, args.meskf_file)
  timer.add_callback( real_time_plot,
                      args.ground_truth_file,
                      args.visual_odometry_file,
                      args.meskf_file)
  timer.start()
  pylab.show()