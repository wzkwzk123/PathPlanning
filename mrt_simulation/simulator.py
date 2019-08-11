import pygame, time
import numpy as np
form calcuate import CostFunction
from shapely.geometry import Polygon, Point, LineString


# define the width of the pygame window
WINDOW_WIDTH = 800
WINDOW_HEIGHT = 800


class Simulation_Road(object):
	"""docstring for Simulation_road
	simulation_rate : simulation_rate will cover the simulation rate of vhicle
	"""
	def __init__(self, w, h, simulation_rate=20):
		super(Simulation_road, self).__init__()
		self.map_width = w
		self.map_height = h
		self.ratio = WINDOW_WIDTH / self.map_width
		self.simulation_rate = simulation_rate

		self.cost_function = CostFunction()

		self.initial_circle_points = self.generate_points_on_circle()
		self._display = pygame.display.set_mode((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.HWSURFACE | pygame.DOUBLEBUF)
		self.display = pygame.Surface((WINDOW_WIDTH, WINDOW_HEIGHT), pygame.SRCALPHA, 32)
		self.display.fill(WHITE)

		self.surf_view = pygame.Surface((2 * self.view_range * self.ratio, 2 * self.view_range * self.ratio))
		self.surf_view.set_alpha(60)

	def set_record_status(self, with_vis_risk, with_object):
		self.record["vis_risk"] = "with_vis" if with_vis_risk else "no_vis"
		if with_vis_risk:
			self.cost_function.set_vis_weight(8000)
		else:
			self.cost_function.set_vis_weight(0)
		self.record["with_obj"] = "with_obj" if with_object else "no_obj"
		self.cost_function.set_scene(self.scene)



	def generate_points_on_circle(self):
		delta = np.arange(0, 360, 0.5) * np.pi / 180
		return [[self.view_range * math.cos(x), self.view_range * math.sin(x)] for x in delta]

    def set_ego_vehicle(self, vehicle):
        vehicle.set_simulation_rate(self.simulation_rate)
        self.ego_vehicle = vehicle

    def add_object_vehicle(self, vehicle):
        vehicle.set_simulation_rate(self.simulation_rate)
        self.obj_vehicles.append(vehicle)

    def add_obstacle(self, obstacle):
        self.obstacles.append(obstacle)

    def add_obj_path(self, path):
        self.path.append(path)

    def set_roi(self, roi):
        self.roi = roi


    def execute(self):
    	"""Launch the PyGame"""
    	pygame.init()
    	self.

    def world_coord_to_map(self, p):
        # transform the coordinate in the world system to the pygame window coordinate
        return [int(p[0] * self.ratio + WINDOW_WIDTH / 2),
                int(-p[1] * self.ratio + WINDOW_HEIGHT / 2)]

    def get_view_polygon(self):
        # return polygon in map coordinate
        view_polygon = []
        view_polygon_static = []
        # self.ego_vechicle is a private variable of Vehicle class
        circle_points = [[x[0] + self.ego_vehicle.get_position()[0],
                          x[1] + self.ego_vehicle.get_position()[1]] for x in self.initial_circle_points]  # m
        for p in circle_points:
            close_p = []
            close_p_static = []
            dis_tmp = 1000
            for obs in self.obstacles:
                inter_p = obs.get_intersects_point(self.ego_vehicle.get_position(), p)
                if inter_p:
                    dis = dis_square(self.ego_vehicle.get_position(), inter_p)
                    if dis < dis_tmp:
                        dis_tmp = dis
                        close_p = inter_p
            dis_tmp = 1000
            for obs in self.obstacles:
                if obs.get_obs_type() == "static":
                    inter_p = obs.get_intersects_point(self.ego_vehicle.get_position(), p)
                    if inter_p:
                        dis = dis_square(self.ego_vehicle.get_position(), inter_p)
                        if dis < dis_tmp:
                            dis_tmp = dis
                            close_p_static = inter_p
            if not close_p:
                view_polygon.append(self.world_coord_to_map(p))
            else:
                view_polygon.append(self.world_coord_to_map(close_p))
            if not close_p_static:
                view_polygon_static.append(self.world_coord_to_map(p))
            else:
                view_polygon_static.append(self.world_coord_to_map(close_p_static))
        self.view_polygon = Polygon(view_polygon)
        self.view_polygon_static = Polygon(view_polygon_static)

    def prediction(self):
        # sets_of_obj_at_t: {id1: {t0: sets, t1: sets}, id2: {t0: sets, t1: sets}, ...}
        # sets_of_obj_at_t = [{}] * len(self.obj_vehicles)
        sets_of_obj_at_t = {}

        # get sets of predicted objects because of occlusion
        self.sensing_edge = []
        # in ego path
        for path in self.path:
            path_line = LineString([self.world_coord_to_map(p) for p in path])
            occluded_part_map_list = [path_line.intersection(self.view_polygon)]
                                      # path_line.intersection(self.view_polygon_static)]
            for occluded_part_map in occluded_part_map_list:
                if occluded_part_map.type == "LineString":
                    occluded_part = []
                    for i in range(len(occluded_part_map.coords)):
                        occluded_part.append(self.map_coord_to_world(occluded_part_map.coords[i]))
                    if len(occluded_part) > 1:
                        real_edge = True
                        for obj in self.obj_vehicles:
                            bounds = []
                            for i in range(len(obj.get_hull()) - 1):
                                bounds.append(LineString([obj.get_hull()[i], obj.get_hull()[i + 1]]))
                            bounds.append(LineString([obj.get_hull()[-1], obj.get_hull()[0]]))
                            for bound in bounds:
                                if bound.distance(Point(occluded_part[0])) < 0.5:
                                    real_edge = False
                                    break
                            # obj_polygon = Polygon(obj.get_hull())
                            # if Point(occluded_part[0]).within(obj_polygon):
                            #     real_edge = True
                            # elif obj_polygon.distance(Point(occluded_part[0])) < 0.5:
                            #     real_edge = False
                        if real_edge:
                            edge = SensingEdge(occluded_part[0][0], occluded_part[0][1])
                            edge.set_path(path)
                            self.sensing_edge.append(edge)

                if occluded_part_map.type == "MultiLineString":
                    for part in occluded_part_map.geoms:
                        occluded_part = []
                        for i in range(len(part.coords)):
                            occluded_part.append(self.map_coord_to_world(part.coords[i]))
                        if len(occluded_part) > 1:
                            real_edge = True
                            for obj in self.obj_vehicles:
                                bounds = []
                                for i in range(len(obj.get_hull()) - 1):
                                    bounds.append(LineString([obj.get_hull()[i], obj.get_hull()[i + 1]]))
                                bounds.append(LineString([obj.get_hull()[-1], obj.get_hull()[0]]))
                                for bound in bounds:
                                    if Point(occluded_part[0]).distance(bound) < 0.5:
                                        real_edge = False
                                        break
                                # obj_polygon = Polygon(obj.get_hull())
                                # if Point(occluded_part[0]).within(obj_polygon):
                                #     real_edge = True
                                # elif obj_polygon.distance(Point(occluded_part[0])) < 0.5:
                                #     real_edge = False
                            if real_edge:
                                edge = SensingEdge(occluded_part[0][0], occluded_part[0][1])
                                edge.set_path(path)
                                self.sensing_edge.append(edge)

    def update(self, step):
        """do prediction, calculate step risks for different acceleration, choose best acceleration and send command"""

        # update roi
        if self.scene == "overtake":
            self.roi = [[x[0], x[1]] for x in self.roi if x[1] > self.ego_vehicle.get_position()[1]]
            if len(self.roi) == 0:
                self.roi = [self.ego_vehicle.get_path[0]]

        # prediction #

        # last = time.time()
        # get sets of visible objects
        failsafe_time = 1.5
        if self.scene == "merging":
            T = 3.2
            dt = 0.2
        elif self.scene == "follow_brake":
            T = 3.5
            dt = 0.1
        elif self.scene == "overtake":
            # dynamic prediction horizon
            T = self.ego_vehicle.get_velocity() / 4 + failsafe_time + 1
            dt = 0.2
        else:
            T = 4.5
            dt = 0.3
        self.t_range = [round(t, 2) for t in np.arange(0, T, dt)]
        # self.t_fail_safe_range = [int(1.0/dt), int(2.0/dt), int(3.0/dt)]
        self.t_fail_safe_range = [int(failsafe_time / dt)]
        sets_of_obj_at_t = self.prediction()
        # now = time.time()
        # print("prediction: ", now - last)

        # generate possible trajectorys
        real_path = {}
        for delta_x, path in self.trajs.items():
            real_path[delta_x] = [[path[i] + self.ego_vehicle.get_position()[0],
                                   2 * i + self.ego_vehicle.get_position()[1]] for i in np.arange(0, 50, 1)]

        # filter path which are not in the ego lane and not too big difference with the last one
        real_path_copy = deepcopy(real_path)
        for delta_x, path in real_path_copy.items():
            if path[-1][0] < self.ego_vehicle.get_bound()[0][0][0] + 0.8 or \
                    path[-1][0] > self.ego_vehicle.get_bound()[1][0][0] - 0.8 or \
                    abs(delta_x - self.last_path) > 1.9:
                real_path.pop(delta_x)
        del real_path_copy

        # try one step for each trajectory and calculate the visibility increase, then give reward
        # last = time.time()
        vis_risk_for_path = {}
        ego_copy = deepcopy(self.ego_vehicle)
        ego_copy.set_id("ego_copy")
        ego_copy.memorize_state()
        for dx, path in real_path.items():
            ego_copy.set_follow_path(path)
            vis_risk_for_path[dx] = self.get_vis_risk_of_path_after_t(ego_copy)
            ego_copy.recover_memory()

        # now = time.time()
        # print("risk for trajectories: ", now - last)

        # risk calculation #
        # last = time.time()
        risk_for_path_and_a = {}
        a_range = []
        if self.last_action:
            a_range = np.arange(max(-4, self.last_action - 0.8), min(3, self.last_action + 0.9), 0.2)
        else:
            a_range = np.arange(-4, 3, 0.3)
        for deltaX, path in real_path.items():
            ego_copy.set_follow_path(path)
            risk_for_a, a_range = self.risk_calculation(ego_copy, a_range, sets_of_obj_at_t)
            risk_for_path_and_a[deltaX] = risk_for_a
            # ego_copy.recover_memory()
        del ego_copy
        # now = time.time()
        # print("risk calculation for a: ", now - last)

        # choose a based on the cost #
        path_id, acc_control, t_fail_safe = self.cost_function.get_a_and_path_from_cost(a_range,
                                                                                        [self.t_range[i] for i in self.t_fail_safe_range],
                                                                                        risk_for_path_and_a,
                                                                                        vis_risk_for_path,
                                                                                        self.ego_vehicle)
        self.last_action = acc_control
        self.last_path = path_id
        # print(acc_control)


        for obj in self.obj_vehicles:
            polygons = []
            if obj.get_visibility():
                reachable_obj = obj.get_reachable_set(T)
                for i in range(len(reachable_obj)):
                    hull_map = [self.world_coord_to_map(b) for b in reachable_obj[i][4]]
                    polygons.append(Polygon(hull_map))
            if polygons:
                obj_set_polygon = cascaded_union(polygons).exterior.coords
                pygame.draw.polygon(self.display, RED, obj_set_polygon, 2)

        # control the vehicle #
        self.ego_vehicle.set_follow_path(real_path[round(path_id, 2)])
        if self.ego_vehicle.drive_acc:
            acc_control = self.ego_vehicle.get_ACC([obj for obj in self.obj_vehicles if obj.id == "obj1"])
        self.ego_vehicle.step(self.ego_vehicle.delta_t, acc=acc_control)
        for obj in self.obj_vehicles:
            if obj.id in self.obj_acc_profil:
                print("front acc: ", self.obj_acc_profil[obj.id][step])
                obj.step(obj.delta_t, acc=self.obj_acc_profil[obj.id][step])
            else:
                if obj.drive_acc:
                    acc = obj.get_ACC([obj for obj in self.obj_vehicles if obj.id == "obj0"])
                    print("ACC: ", acc)
                    obj.step(obj.delta_t, acc=acc)
                else:
                    obj.step(obj.delta_t, acc=0)

        # visualize the reachable sets
        reachable_ego, safe_set = self.ego_vehicle.predicted_and_safe_set(t_fail_safe, T - dt)
        polygons = []
        for i in range(len(reachable_ego)):
            hull_map = [self.world_coord_to_map(b) for b in reachable_ego[i][4]]
            pygame.draw.polygon(self.display, BLUE, hull_map, 2)
        #     polygons.append(Polygon(hull_map))
        # if polygons:
        #     ego_set_polygon = cascaded_union(polygons).exterior.coords
        #     pygame.draw.polygon(self.display, BLUE, ego_set_polygon, 2)
        polygons = []
        for i in range(len(safe_set)):
            hull_map = [self.world_coord_to_map(b) for b in safe_set[i][4]]
            pygame.draw.polygon(self.display, LOW_RED, hull_map, 2)
        #     polygons.append(Polygon(hull_map))
        # if polygons:
        #     safe_set_polygon = cascaded_union(polygons).exterior.coords
        #     pygame.draw.polygon(self.display, RED, safe_set_polygon, 2)

        print("v: {}".format(self.ego_vehicle.get_velocity()), "a: {}".format(acc_control))

