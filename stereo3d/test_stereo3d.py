"""This module tests core functionality in i3drsgm module"""
import os
from stereo3d import Stereo3D
from stereo3d.stereocalibration import StereoCalibration
from stereo3d.stereocapture import StereoCapture


def test_init_stereo3d():
    """Test initalising Stereo3D class"""
    script_folder = os.path.dirname(os.path.realpath(__file__))
    sample_data_folder = os.path.join(script_folder, os.pardir, "sample_data")
    stcal = StereoCalibration()
    left_cal_file = os.path.join(sample_data_folder, "deimos_left.yaml")
    right_cal_file = os.path.join(sample_data_folder, "deimos_right.yaml")
    res, _ = stcal.get_cal_from_yaml(left_cal_file, right_cal_file)
    if not res:
        raise Exception("Failed to load calibration yamls")
    stcap = StereoCapture("Image", [
        os.path.join(sample_data_folder, "deimos_left.png"),
        os.path.join(sample_data_folder, "deimos_right.png")])
    Stereo3D(stcap, stcal, "BM", show_window=False)
