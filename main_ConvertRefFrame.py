from class_ConvertRefFrame import ConvertRefFrame
from func_ConvertCrf2Json import crf2json
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
	pdb.set_trace()
	crf2json(gt, file_dir, name)
	print("converted to json")
	json_file_name = file_dir+name+"_json.txt"
	json_txt_file_read = open(json_file_name,'r')
	json_dict_read = json.load(json_txt_file_read)

	

if __name__=="__main__": 
	main()
