class CostFunction(object):
	"""docstring for CostFunction"""
	def __init__(self):
		self.a_last = 0
		self.t_fail_safe_last = 1
		self.path_last = 0
		self.w1 = 5000  # velocity
		self.w2 = 20    # acceleration
		self.w3 = 200  # acceleration variance
		self.w4 = 10   # collision risk
		self.w5 = 50    # stay in the middle of the path
		self.w6 = 10   # path variance#
		self.w7 = 0    # path visibility risk 0 or 8000
		self.w8 = 100    # t_fail_safe value
		self.w9 = 10    # t_fail_safe variance
		self.v_desired = 15
		self.a_max = 4

		self.scene = None

	def set_vis_weight(self, w):
		self.w7 = w

	def set_scene(self, scene):
		self.scene = scene





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
	# import matplotlib.pyplot as plt
	# for deltaX in np.arange(wleft, wright + 0.9, 0.9):
	#     plt.plot(x, cs[round(deltaX, 2)](x))
	# plt.show()
	return path

def vehicle_pose_to_rectangle(pos_x, pos_y, yaw, length, width):
	# get the hull point of vehicle given position and yaw angle
	head = [pos_x + length / 2 * math.cos(yaw), pos_y + length / 2 * math.sin(yaw)]
	tair = [pos_x - length / 2 * math.cos(yaw), pos_y - length / 2 * math.sin(yaw)]
	head_left = [head[0] - width / 2 * math.sin(yaw), head[1] + width / 2 * math.cos(yaw)]
	head_right= [head[0] + width / 2 * math.sin(yaw), head[1] - width / 2 * math.cos(yaw)]
	tair_left = [tair[0] - width / 2 * math.sin(yaw), tair[1] + width / 2 * math.cos(yaw)]
	tair_right= [tair[0] + width / 2 * math.sin(yaw), tair[1] - width / 2 * math.cos(yaw)]
	return [head_left, head_right, tair_right, tair_left]

def point_to_line_dist(P, A, B):
	""" segment line AB, point P, where each one is an array([x, y]) """
	if all(A == P) or all(B == P):
		return0
	if arccos(dot((P - A) / norm(P - A), (B - A) / norm(B - A))) > pi / 2:
		return norm(P - A)
	if arccos(dot((P - B) / norm(P - B), (A - B) / norm(A - B))) > pi / 2:
		return norm(P - B)
	return norm(cross(A-B, A-P))/norm(B-A)

def project_point_to_segment(point, line):
    # point: [x, y]
    # line: [[x1, y1], [x2, y2]]
    x = np.array(point)
    u = np.array(line[0])
    v = np.array(line[1])
    n = v - u
    n = n / np.linalg.norm(n, 2)
    return list(u + n * np.dot(x - u, n))

def dis_square(p1, p2):
    # get the distance of 2 points
    # point: [x, y]
    # distance.euclidean(p1, p2)
    # np.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)
    # math.sqrt(((p1[0]-p2[0])**2)+((p1[1]-p2[1])**2))
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)

		