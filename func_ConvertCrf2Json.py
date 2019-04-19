import json 
from class_ConvertRefFrame import ConvertRefFrame
import numpy as np

def crf2json(ConvertRefFrame, file_dir, file_name):
	"""Writes the class ConvertRefFrame to a json file"""

	if file_name[-4:] == ".txt":
		file_name = file_name[:-4]
	
	# convert class to a dictionary containing lists only. Arrays are not excepted
	positions_list = [position.tolist() for position in ConvertRefFrame.positions]
	orientations_list = [orientation.tolist() for orientation in ConvertRefFrame.orientations]
	crf_dict = {	"time": ConvertRefFrame.time, 
			"positions": positions_list, 
			"orientations": orientations_list, 
			"file_location": ConvertRefFrame.flocation, 
			"plot_style": ConvertRefFrame.plotstyle,
			"method": ConvertRefFrame.method
			}
	json_data = json.dumps(crf_dict)
	json_file_name = file_dir + file_name + "_json.txt"
	json_file = open(json_file_name, 'w')
	json_file.write(json_data)
	json_file.close()

def json2crf(file_dir, file_name):
	json_file = open(file_dir+file_name, 'r')
	crf_dict = json.load(json_file)
	
	method = crf_dict["method"]
	file_location = crf_dict["file_location"]
	plotstyle = crf_dict["plot_style"]

	crf = ConvertRefFrame(method, file_location, plotstyle)
	crf.time = crf_dict["time"] 
	positions_list = crf_dict["positions"] 	 	
	crf.positions = [np.array(position) for position in positions_list]
	orientations_list = crf_dict["orientations"] 
	crf.orientations = [np.array(orientation) for orientation in orientations_list]
	crf.filelocation = file_dir+file_name
