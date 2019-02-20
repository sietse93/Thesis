from CarlaSlamPerformance import CarlaSlamEvaluate




def match_gt_slam(gt, slam):
    index_matches = [gt.time.index(t) for t in slam.time]
    matches_gt = [(gt.time[index], gt.positions[index]) for index in index_matches]
    matches_slam = [(t, slam.positions[index]) for index, t in enumerate(slam.time)]
    matches = (matches_gt, matches_slam)
    return matches


def main():

    """Test file to incorporate the ATE of the TUM benchmark"""

    floc_gt = "/home/sietse/official_experiment_data/SL_20_NV_0_SV_1_gt.txt"
    floc_slam = "/home/sietse/official_experiment_data/SL_20_NV_0_SV_1_orb.txt"

    with CarlaSlamEvaluate("gt", floc_gt,'k-') as gt:
        gt.process_data()

    with CarlaSlamEvaluate("orb", floc_slam, "g-") as slam:
        slam.process_data()

    # matches -- list of matched tuples ((stamp1,data1),(stamp2,data2))
    matches = match_gt_slam(gt, slam)
    print(matches)
    print(matches(0))

    # first_xyz =[[float(value) for value in first_list[a][0:3]] for a,b in matches]



if __name__ == "__main__":
    main()
