from stereo3d import Stereo3D, StereoCalibration
from stereo3d.stereocapture import StereoCapture
import cv2

CAL_MODE_FROM_IMAGES = 0
CAL_MODE_FROM_YAML = 1
CAL_MODE_FROM_XML = 2
MATCH_MODE_BM = "BM"
MATCH_MODE_SGBM = "SGBM"

folder_images = "D:/Users/monke/OneDrive/Desktop/ml/in/deimos/" # Set this to your image folder (Must end with '/')
out_folder = "D:/Users/monke/OneDrive/Desktop/ml/out/" # Set ths to your output folder (Must end with '/')
folder_cal = "D:/Users/monke/OneDrive/Desktop/Deimos/deimos_cal_001023/" # Set this to your calibration folder (Must end with '/')
left_image_name = "20200527_114055_686_l.png" # Set this to the left image name
right_image_name = "20200527_114055_686_r.png" # Set this to the right image name
marked_image_name = "20200527_114055_686_l_marked.png" # Set this to the left marked image name
cal_mode = CAL_MODE_FROM_XML # Choose calibration load type (can set this to CAL_MODE_FROM_IMAGES to calibrate from images however this isn't well tested yet so I wouldn't)
matcher = MATCH_MODE_BM # Choose matcher type

stcap = StereoCapture("Image",[folder_images+left_image_name,folder_images+right_image_name])
stcal = None
if (cal_mode == CAL_MODE_FROM_IMAGES):
    # define calibration directories
    left_images_folder = folder_cal
    right_images_folder = folder_cal
    output_folder = folder_cal
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
    left_cal_file = folder_cal + "left.yaml"
    right_cal_file = folder_cal + "right.yaml"
    # get calibration from yaml files
    stcal = StereoCalibration()
    stcal.get_cal_from_yaml(left_cal_file,right_cal_file)
elif (cal_mode == CAL_MODE_FROM_XML):
    # define calibration files for left and right image
    left_cal_file = folder_cal + "left_calibration.xml"
    right_cal_file = folder_cal + "right_calibration.xml"
    stereo_cal_file = folder_cal + "stereo_calibration.xml"
    left_rect_file = folder_cal + "left_rectification.xml"
    right_rect_file = folder_cal + "right_rectification.xml"
    # get calibration from yaml files
    stcal = StereoCalibration()
    stcal.get_cal_from_xml(left_cal_file,right_cal_file,stereo_cal_file,left_rect_file,right_rect_file)

matcher_bm_05_3 = cv2.StereoBM_create()
default_min_disp = 1000
default_num_disparities = 38
default_block_size = 6
default_uniqueness_ratio = 3
default_texture_threshold = 5
default_speckle_size = 500
default_speckle_range = 5
calc_block = (2 * default_block_size + 5)
matcher_bm_05_3.setBlockSize(calc_block)
matcher_bm_05_3.setMinDisparity(int(default_min_disp - 1000))
matcher_bm_05_3.setNumDisparities(16*(default_num_disparities+1))
matcher_bm_05_3.setUniquenessRatio(default_uniqueness_ratio)
matcher_bm_05_3.setTextureThreshold(default_texture_threshold)
matcher_bm_05_3.setSpeckleWindowSize(default_speckle_size)
matcher_bm_05_3.setSpeckleRange(default_speckle_range)

matcher_sgbm_05_3 = cv2.StereoSGBM_create()
default_min_disp = 1000
default_num_disparities = 38
default_block_size = 6
default_uniqueness_ratio = 3
default_speckle_size = 500
default_speckle_range = 5
calc_block = (2 * default_block_size + 5)
matcher_sgbm_05_3.setBlockSize(calc_block)
matcher_sgbm_05_3.setMinDisparity(int(default_min_disp - 1000))
matcher_sgbm_05_3.setNumDisparities(16*(default_num_disparities+1))
matcher_sgbm_05_3.setUniquenessRatio(default_uniqueness_ratio)
matcher_sgbm_05_3.setSpeckleWindowSize(default_speckle_size)
matcher_sgbm_05_3.setSpeckleRange(default_speckle_range)

matcher = matcher_bm_05_3

# setup Stereo3D
s3D = Stereo3D(stcap,stcal,matcher)
connected = s3D.connect()
marked_image = cv2.imread(folder_images+marked_image_name)
left_image = cv2.imread(folder_images+left_image_name)
# run Stereo3D GUI for generating 3D
res, _ = s3D.grab3D(False)
if res:
    rect_marked_image, _ = s3D.stereo_calibration.rectify_pair(marked_image,s3D.image_right)
    rect_left_image, _ = s3D.stereo_calibration.rectify_pair(left_image,s3D.image_right)
    points_file_string = "points.ply"
    s3D.save_point_cloud(s3D.disparity,rect_left_image,out_folder,points_file_string,False)
    points_file_string = "points_marked.ply"
    s3D.save_point_cloud(s3D.disparity,rect_marked_image,out_folder,points_file_string,False)
    left_rect_filename = "left_rect.png"
    right_rect_filename = "right_rect.png"
    s3D.stereo_camera.save_images(s3D.rect_image_left,s3D.rect_image_right,out_folder,left_rect_filename,right_rect_filename,False)
else:
    print("Failed to grab 3D")

# setup Stereo3D
s3D = Stereo3D(stcap,stcal,matcher)
# run Stereo3D GUI for generating 3D
s3D.run(out_folder)