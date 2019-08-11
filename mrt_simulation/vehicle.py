import numpy as np
import time
from calculate_function import *
from obstacle import Obstacle

class Vehicle(Obstacle):
	"""docstring for Vehicle"""
	def __init__(self, id, state=None, simulation_rate=50):
		super(Vehicle, self).__init__()
		self.state = state
		self.last_state = None
		self.last_segment_id = None
		self.length = 5
		self.width = 2.5
		super(Vehicle, self).__init__(hull_points=vehicle_pose_to_rectangle(self.state[0], 
			self.state[1],self.state[3], self.length, self.width))
		self.set_obs_type("dynamic")
		self.path = None
		self.left_bound = None
		self.follow_path = False
		self.delta_t = 1 / simulation_rate
		self.id = id
		self.segment_dis_list = []
		self.yaw_list = []
		self.acc_history = []
		self.v_history = []
		self.drive_acc = False

		def set_simulation_rate(self, rate):
			self.delta_t = 1 / rate

		def set_state(self, state):
			self.state = state

		def set_id(self, id):
			self.id = id

		def get_state(self):
			return self.state

		def get_position(simulation_rate):
			return [self.state[0], self.state[1]]

    def set_shape(self, length, width):
        self.length = length
        self.width = width

        
    def set_bound(self, path):
        self.left_bound = path["left"]
        self.right_bound = path["right"]

    def set_follow_path(self, path):
        self.path = path
        self.follow_path = True
        segment, segment_id, dis = self.get_closest_segment_and_distance(self.path)
        if dis > 1:
            print("position: ", self.get_position(), "path: ", self.path)
            print("initial position of vehicle not on the path!")
            return
        start_point = project_point_to_segment(self.get_position(), segment)
        self.state[0], self.state[1] = start_point
        self.state[3] = np.arctan2(segment[1][1] - segment[0][1], segment[1][0] - segment[0][0])

        # save segments distance to seg_dis_list and yaw list
        self.segment_dis_list = []
        self.yaw_list = []
        for i in range(len(self.path) - 1):
            self.segment_dis_list.append(dis_square(self.path[i], self.path[i+1]))
            self.yaw_list.append(np.arctan2(self.path[i+1][1] - self.path[i][1], self.path[i+1][0] - self.path[i][0]))
        self.yaw_list.append(self.yaw_list[-1])




    def get_closest_segment_and_distance(self, path, dt=None, last_state=None, last_segment_id=None):
        # path: [[x1, y1], [x2, y2], ...]
        # last_state: state of the vehicle at the last time step
        # last_segment_id: id of the segment which the vehicle was on
        # return: segment: [[x1, y1], [x2, y2]], dis_min: x
        dis_min = 1000
        segment = []
        segment_id = None
        if last_segment_id is None or last_state is None:
            for i in range(len(path) - 1):
                dis = point_to_line_dist(np.array(self.get_position()), np.array(path[i]), np.array(path[i + 1]))
                if dis < dis_min:
                    segment = [path[i], path[i + 1]]
                    dis_min = dis
                    segment_id = i
        else:
            p1 = path[last_segment_id]
            p2 = path[last_segment_id+1]
            angle1 = np.arctan2(p1[1] - last_state[1], p1[0] - last_state[0])
            if abs(angle1 - last_state[3]) < np.pi/2:
                if dis_square([last_state[0], last_state[1]], p1) > last_state[2] * dt:
                    segment_id = last_segment_id
                else:
                    segment_id = max(0, last_segment_id-1)
            else:
                if dis_square([last_state[0], last_state[1]], p2) > last_state[2] * dt:
                    segment_id = last_segment_id
                else:
                    segment_id = min(len(path)-2, last_segment_id+1)
            dis_min = point_to_line_dist(np.array(self.get_position()),
                                         np.array(path[segment_id]),
                                         np.array(path[segment_id + 1]))
            segment = [path[segment_id], path[segment_id + 1]]
        return segment, segment_id, dis_min


