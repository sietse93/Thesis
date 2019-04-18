from class_ConvertRefFrame import ConvertRefFrame
import json
import pdb

# Converts .txt files from ORB and CARLA to the same reference frame (right hand relative 
# coordinate system). 

def main(): 
	file_dir = "/home/svanschouwenburg/results_afstuderen/"
	file_loc = "/home/svanschouwenburg/results_afstuderen/T1_SL0_s_gt.txt"
	name = "T1_SL0_s_gt"	
	with ConvertRefFrame("gt", file_loc, 'k-') as gt: 
		gt.process_data()
	print("data processed")
	positions_list = [position.tolist() for position in gt.positions]
	orientations_list = [orientation.tolist() for orientation in gt.orientations]
	gt_dict = {	"time": gt.time, 
			"positions": positions_list, 
			"orientations": orientations_list, 
			"file_location": gt.flocation, 
			"plot_style": gt.plotstyle
			}  

	json_file_name = name + "_json.txt"
	json_data = json.dumps(gt_dict)
	json_txt_file = open(file_dir+json_file_name,'w')
	json_txt_file.write(json_data)
	json_txt_file.close()	

	json_txt_file_read = open(file_dir+json_file_name,'r')
	json_dict_read = json.load(json_txt_file_read)
	# print(json_dict_read["orientations"])
	

if __name__=="__main__": 
	main()
