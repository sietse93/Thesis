from class_ConvertRefFrame import ConvertRefFrame
from func_ConvertCrf2Json import crf2json
import json
import pdb


def main():
	"""Converts .txt files from ORB and ground truth to the same reference frame (right hand relative
	coordinate system) And exports them to json files"""

	base_dir = "/home/sietse/results_carla0.9/VansOppositeRoad/"

	Towns = (1, 2)
	scenario = "dynamic"
	dynamic_variable = 10
	mode = "SLAM"
	for Town in Towns:
		if Town == 1:
			starting_locations = (27, 0, 58)
		elif Town == 2:
			starting_locations = (18, 37, 78)
		elif Town == 3:
			starting_locations = (75, 97, 127, 132)
		else:
			print("Town does not exist")
			return
		for SL in starting_locations:
			# for scenario in scenarios:
			ConvertOneScenarioAtSL(Town, SL, scenario, base_dir, dynamic_variable, mode)


def ConvertOneScenarioAtSL(Town, SL, scenario, base_dir, DV, mode):
	# NOTE: STATIC DATA IS NOT WELL IMPLEMENTED FOR SCENARIO VANS IN OPPOSITE DIRECTION
	DS = DV
	if scenario == "dynamic":
		# for DS in DV:
		dir_name = "T{}_SL{}_{}{}/".format(Town, SL, scenario[0], DS)
		print("Converting {}".format(dir_name[:-1]))
		file_dir = base_dir + dir_name
		for i in range(5):
			if mode == "SLAM":
				file_name = "T{}_SL{}_{}{}_orb_{}.txt".format(Town, SL, scenario[0], DS, i)
			elif mode == "VO":
				file_name = "T{}_SL{}_{}{}_orb_vo_{}.txt".format(Town, SL, scenario[0], DS, i)
			elif mode == "MC":
				file_name = "T{}_SL{}_{}{}_orb_mc_off_{}.txt".format(Town, SL, scenario[0], DS, i)
			file_loc = base_dir + dir_name + file_name
			with ConvertRefFrame("orb", file_loc, "C{}--".format(i)) as dyn_orb:
				dyn_orb.process_data()

			crf2json(dyn_orb, file_dir, file_name)

		file_name = "T{}_SL{}_{}{}_gt.txt".format(Town, SL, scenario[0], DS)
		file_loc = base_dir + dir_name + file_name

		with ConvertRefFrame("gt", file_loc, "k-") as gt:
			gt.process_data()
		crf2json(gt, file_dir, file_name)

	elif scenario == "static":
		dir_name = "T{}_SL{}_{}/".format(Town, SL, scenario[0])
		file_dir = base_dir + dir_name
		print("Converting {}".format(dir_name[:-1]))
		for i in range(5):
			file_name = "T{}_SL{}_{}_orb_{}.txt".format(Town, SL, scenario[0], i)

			file_loc = base_dir + dir_name + file_name

			with ConvertRefFrame("orb", file_loc, "C{}-".format(i)) as stat_orb:
				stat_orb.process_data()

			crf2json(stat_orb, file_dir, file_name)




if __name__ == "__main__":
	main()
