from CarlaSlamPerformanceV2 import CarlaSlamEvaluate

# lets first try to analyze stuckbehindvan d20
# scenario contains 3 distances
# each has 5 orb estimations
# 1 groundtruth 
# 10 starting locations

# each (scenario) distance: 10 starting locations (each a gt) with 5 orb estimations. 
# each starting location has a json file containing
# 1) ground truth poses 
# 2) average RPE dist static of complete trajectory
# 3) average RPE dist dynamic of complete trajectory 
# 4) average RPE dist static vs dynamic 
# 5) best RPE dist over time static 
# 6) best RPE dist over time dynamic
# 7) best static orb pose est
# 8) best dynamic orb pose est 
# 9)  

