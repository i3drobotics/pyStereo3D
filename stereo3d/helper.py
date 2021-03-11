"""This module is used for manually testing functionality while developing stereo3d module"""
from stereo3d import Stereo3D
from stereo3d.stereocalibration import StereoCalibration
from stereo3d.stereocapture import StereoCapture

CAMERA_TYPE_PHOBOS = 0
CAMERA_TYPE_DEIMOS = 1
CAMERA_TYPE_PYLON = 2
CAMERA_TYPE_IMAGE = 3
CAMERA_TYPE_VREP = 4

camera_type = CAMERA_TYPE_IMAGE

stcap = None
camera_name = None
if (camera_type == CAMERA_TYPE_PHOBOS):
    camera_name = "phobos"
    stcap = StereoCapture("Phobos",["22864912","22864917"])
elif (camera_type == CAMERA_TYPE_PYLON):
    stcap = StereoCapture("Pylon",["22864912","22864917"])
elif (camera_type == CAMERA_TYPE_DEIMOS):
    camera_name = "deimos"
    stcap = StereoCapture("Deimos",0)
elif (camera_type == CAMERA_TYPE_IMAGE):
    camera_name = "deimos"
    stcap = StereoCapture("Image",["sample_data/deimos_left.png","sample_data/deimos_right.png"])
else:
    print("Invalid camera type.")
    exit()

# define inout folder
folder = "sample_data/"

CAL_MODE_FROM_IMAGES = 0
CAL_MODE_FROM_YAML = 1
CAL_MODE_FROM_XML = 2
CAL_MODE_IDEAL = 3

stcal = None
cal_mode = CAL_MODE_FROM_YAML
if (cal_mode == CAL_MODE_IDEAL):
    resolution=[2448,2048]
    pixel_pitch=0.00000345
    focal_length=0.016
    baseline=0.3
    output_folder = folder + "ideal_cal/"
    stcal = StereoCalibration()
    stcal.get_cal_from_ideal(resolution, pixel_pitch, focal_length, baseline, output_folder)
elif (cal_mode == CAL_MODE_FROM_IMAGES):
    # define calibration directories
    left_images_folder = folder + "deimos_cal/"
    right_images_folder = folder + "deimos_cal/"
    output_folder = folder + "deimos_cal/"
    left_wildcard = "*_l.png"
    right_wildcard = "*_r.png"
    grid_size = 39.0
    grid_rows = 6
    grid_cols = 8
    # generate calibration from images
    stcal = StereoCalibration()
    stcal.calibrate(
        left_images_folder,right_images_folder,
        output_folder,left_wildcard,right_wildcard,
        grid_size, grid_rows, grid_cols
    )
elif (cal_mode == CAL_MODE_FROM_YAML):
    # define calibration files for left and right image
    left_cal_file = folder + camera_name +"_left.yaml"
    right_cal_file = folder + camera_name +"_right.yaml"
    # get calibration from yaml files
    stcal = StereoCalibration()
    stcal.get_cal_from_yaml(left_cal_file,right_cal_file)
elif (cal_mode == CAL_MODE_FROM_XML):
    # define calibration files for left and right image
    left_cal_file = folder + camera_name +"_left_calibration.xml"
    right_cal_file = folder + camera_name +"_right_calibration.xml"
    stereo_cal_file = folder + camera_name +"_stereo_calibration.xml"
    left_rect_file = folder + camera_name +"_left_rectification.xml"
    right_rect_file = folder + camera_name +"_right_rectification.xml"
    # get calibration from yaml files
    stcal = StereoCalibration()
    stcal.get_cal_from_xml(left_cal_file,right_cal_file,stereo_cal_file,left_rect_file,right_rect_file)

# setup Stereo3D
s3D = Stereo3D(stcap,stcal,"I3DRSGM")
# run Stereo3D GUI for generating 3D
s3D.run(folder)