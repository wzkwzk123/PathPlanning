import numpy as np
import time
import math
from numpy import arccos, array, dot, pi, cross
from numpy.linalg import det, norm
from shapely.geometry import Point
from shapely.geometry import LineString
from scipy.interpolate import CubicSpline
from scipy.spatial import distance

def generate_spline(dx=1.25, range=100, wleft=-2.7, wright=2.7):
	# return possible future paths relativ to the vehicle
	y = [0, 10, 11, 12, 13, 60, range]
	cs = {}
	for deltaX in np.arange(wleft, wright + 0.9, 0.9):
		# Round a number to only two decimals
		cs[round(deltaX, 2)] = CubicSpline(y, [0, deltaX, deltaX, deltaX, deltaX, deltaX, deltaX], bc_type='natural')
	x = np.arange(0, range, dx)
	path = {}
	for deltaX in np.arange(wleft, wright + 0.9, 0.9):
		path[round(deltaX, 2)] = list(cs[round(deltaX, 2)](x))
	import matplotlib.pyplot as plt
	for deltaX in np.arange(wleft, wright + 0.9, 0.9):
		plt.plot(x, cs[round(deltaX, 2)](x))
	plt.show()
	return path
generate_spline()

