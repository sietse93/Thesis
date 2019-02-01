from CarlaSlamPerformance import CarlaSlamEvaluate


def load_experiment(flocation, SL):
    """Loads the dynamic and static data based on the starting location"""
    floc_gt_static = flocation + "SL_{}_NV_0_SV_1_gt.txt".format(SL)
    floc_gt_dynamic = flocation + "SL_{}_NV_40_SV_1_gt.txt".format(SL)

    method_gt = "gt"
    with CarlaSlamEvaluate(method_gt, floc_gt_static, 'k-') as gt_static:
        gt_static.process_data()

    with CarlaSlamEvaluate(method_gt, floc_gt_dynamic, 'k--') as gt_dynamic:
        gt_dynamic.process_data()

    floc_orb_static = flocation + "SL_{}_NV_0_SV_1_orb.txt".format(SL)
    floc_orb_dynamic = flocation + "SL_{}_NV_40_SV_1_orb.txt".format(SL)

    method_orb = "orb"

    with CarlaSlamEvaluate(method_orb, floc_orb_static, 'b-') as orb_static:
        orb_static.process_data()

    with CarlaSlamEvaluate(method_orb, floc_orb_dynamic, 'r--') as orb_dynamic:
        orb_dynamic.process_data()

    return gt_static, gt_dynamic, orb_static, orb_dynamic

def main():
    flocation = "/home/sietse/official_experiment_data/"
    SL = 144
    (sl144_gt_static, sl144_gt_dynamic, sl144_orb_static, sl144_orb_dynamic) = load_experiment(flocation, SL)

if __name__ == "__main__":
    main()