def get_detectors(configs):
    if configs.detect_online:
        from .detector2d import get_detector2d
        if configs.data_type == "KITTI":
            from .detector3d import get_detector3d
            return get_detector2d(configs), get_detector3d(configs)
        else:
            print("     get online 2D detectors ")
            return get_detector2d(configs)
    else:
        if configs.data_type == "KITTI":
            return None, None
        else:
            return None


def get_sequence(data_dir, configs):
    if configs.data_type == "KITTI":
        print("get_sequence model KITTI")
        from .kitti_sequence import KITIISequence
        return KITIISequence(data_dir, configs)
    # We use a single class for Redwood and Freiburg sequence
    else:
        if configs.data_type == "Freiburg":
            # print("get_sequence model Freiburg -1")
            from .mono_sequence import MonoSequence
            # print("get_sequence model Freiburg -2")
            return MonoSequence(data_dir, configs)
        else:
            print("get_sequence model else")
            from .mono_sequence import MonoSequence
            return MonoSequence(data_dir, configs)