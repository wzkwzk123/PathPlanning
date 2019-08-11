import numpy as np

from simulator import Simulation_Road


def overtake():
	simulation = Simulation_Road("overtake", 160, 160)
	with_vis_risk = True
	with_obj = True
	simulation.set_record_status(with_vis_risk, with_obj)

	ego_path = [[0, i] for i in np.arange(-80, 80, 1.25)]
	ego_path_left = [[p[0] - 7.5, p[1]] for p in ego_path]
	ego_path_right = [[p[0] + 2.5, p[1]] for p in ego_path]
	ego_vehicle = Vehicle(id="ego", state=[0, -80, 6, 0.5*np.pi, 0])
	ego_vehicle.set_follow_path(ego_path)
	ego_vehicle.set_bound({"left": ego_path_left, "right": ego_path_right})
	simulation.set_ego_vehicle(ego_vehicle)

	# vehicle in the same lane
	obj = Vehicle(id="obj0", state=[0, -65, 6, 0.5*np.pi, 0])
	obj.set_follow_path(ego_path)
	obj.set_bound({"left": ego_path_left, "right": ego_path_right})
	# change the lang and width of vehicle
	obj.set_shape(length=6, width=3)
	simulation.add_object_vehicle(obj)
	simulation.add_obstacle(obj)

	# object in the opposite lane
	





def main():
	overtake()

if __name__ == "__main__":
	main()