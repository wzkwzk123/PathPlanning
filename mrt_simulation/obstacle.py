import numpy as np
from calculate_function import *
from shapely.geometry import Polygon, LineString

class Obstacle(object):
	"""docstring for Obstacle"""
	def __init__(self, hull_position):
		super(Obstacle, self).__init__()
		self.hull = hull_position
		self.obs_type = "static"
		self.visibility = True

	def set_obs_type(self, type):
		self.obs_type = type

	def get_obs_type(self):
		return self.obs_type

	def get_hull(self):
		return self.hull

	def set_visibility(self, bool):
		self.visibility = bool

	def get_visibility(self):
		return self.visibility


    def get_intersects_point(self, view_point, edge_point):
        view_line = LineString([view_point, edge_point])
        if Polygon(self.hull).intersects(view_line):
            dis_tmp = 1000
            p_final = []
            for p in Polygon(self.hull).intersection(view_line).coords:
                dis = dis_square(p, view_point)
                if dis < dis_tmp:
                    dis_tmp = dis
                    p_final = p
            return p_final
        else:
            return False