import json 
from class_ConvertRefFrame import ConvertRefFrame
import numpy as np
import pdb


def crf2json(ConvertRefFrame, file_dir, file_name):
	"""Writes the class ConvertRefFrame to a json file"""

	if file_name[-4:] == ".txt":
		file_name = file_name[:-4]
	
	# convert class to a dictionary containing lists only. Arrays are not excepted
	positions_list = [position.tolist() for position in ConvertRefFrame.positions]
	orientations_list = [orientation.tolist() for orientation in ConvertRefFrame.orientations]
	Q_list = [q.tolist() for q in ConvertRefFrame.Q]
	crf_dict = {"time": ConvertRefFrame.time,
				"positions": positions_list,
				"orientations": orientations_list,
				"file_location": ConvertRefFrame.flocation,
				"plot_style": ConvertRefFrame.plotstyle,
				"method": ConvertRefFrame.method,
				"Q": Q_list,
				"label": ConvertRefFrame.label}
	json_data = json.dumps(crf_dict)
	json_file_name = file_dir + file_name + "_json.txt"
	json_file = open(json_file_name, 'w')
	json_file.write(json_data)
	json_file.close()


def json2crf(file_dir, file_name):
	"""Converts JSON file to ConvertRefFrame class"""
	json_file = open(file_dir+file_name, 'r')
	crf_dict = json.load(json_file)
	# get the minimal attributes from the dictionary to build a ConvertRefFrame
	method = crf_dict["method"]
	file_location = crf_dict["file_location"]
	plotstyle = crf_dict["plot_style"]

	crf = ConvertRefFrame(method, file_location, plotstyle)
	# add attributes to class
	crf.time = crf_dict["time"] 
	positions_list = crf_dict["positions"] 	 	
	crf.positions = [np.array(position) for position in positions_list]
	orientations_list = crf_dict["orientations"] 
	crf.orientations = [np.array(orientation) for orientation in orientations_list]
	Q_list = crf_dict["Q"]
	crf.Q = [np.array(q) for q in Q_list]
	crf.label = crf_dict["label"]

	crf.filelocation = file_dir+file_name

	return crf
