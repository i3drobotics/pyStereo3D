from Stereo3D import Stereo3D, StereoCalibration
from Stereo3D.StereoCapture import *

camera_name = "deimos"
marked_image_filepath = "SampleData/deimos_left_marked.png"
stcap = StereoCapture("Image",["SampleData/deimos_left.png","SampleData/deimos_right.png"])
# define inout folder
folder = "SampleData/"
outputfolder = "SampleData/output/"

CAL_MODE_FROM_IMAGES = 0
CAL_MODE_FROM_YAML = 1
CAL_MODE_FROM_XML = 2

stcal = None
cal_mode = CAL_MODE_FROM_YAML
if (cal_mode == CAL_MODE_FROM_IMAGES):
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

matcher = cv2.StereoBM_create()
default_min_disp = 1000
default_num_disparities = 3
default_block_size = 7
default_uniqueness_ratio = 12
default_texture_threshold = 5
default_speckle_size = 30
default_speckle_range = 16
calc_block = (2 * default_block_size + 5)
matcher.setBlockSize(calc_block)
matcher.setMinDisparity(int(default_min_disp - 1000))
matcher.setNumDisparities(16*(default_num_disparities+1))
matcher.setUniquenessRatio(default_uniqueness_ratio)
matcher.setTextureThreshold(default_texture_threshold)
matcher.setSpeckleWindowSize(default_speckle_size)
matcher.setSpeckleRange(default_speckle_range)
# setup Stereo3D
s3D = Stereo3D(stcap,stcal,matcher)
connected = s3D.connect()
marked_image = cv2.imread(marked_image_filepath)
# run Stereo3D GUI for generating 3D
res, _ = s3D.grab3D(False)
if res:
    rect_marked_image, _ = s3D.stereo_calibration.rectify_pair(marked_image,s3D.image_right)
    points_file_string = "points.ply"
    s3D.save_point_cloud(s3D.disparity,rect_marked_image,outputfolder,points_file_string,False)
else:
    print("Failed to grab 3D")