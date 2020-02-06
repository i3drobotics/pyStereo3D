from Stereo3D import Stereo3D
from StereoCapture import *

# define camera capture types
CAMERA_TYPE_PYLON = 0
CAMERA_TYPE_IC = 1
CAMERA_TYPE_OPENCV_SPLIT = 2
CAMERA_TYPE_OPENCV_DUAL = 3
CAMERA_TYPE_OPENCV_IMAGE = 4
CAMERA_TYPE_OPENCV_IMAGE_RECT = 5
CAMERA_TYPE_OPENCV_FOLDER = 5

# select camera capture type
camera_type = CAMERA_TYPE_OPENCV_IMAGE_RECT

# define output folder
folder = "pyStereo3D/SampleData/"
# define calibration files for left and right image
left_cal_file = folder + "deimos_left.yaml"
right_cal_file = folder + "deimos_right.yaml"

# setup stereo camera
stcam = None
isRectified = False
if (camera_type == CAMERA_TYPE_PYLON): # Pylon camera capture
    left_camera_serial = "22864917"
    right_camera_serial = "22864912"
    camL = PylonCapture(left_camera_serial)
    camR = PylonCapture(right_camera_serial)
    stcam = StereoCapturePylon(camL,camR)
elif (camera_type == CAMERA_TYPE_IC):  # IC camera capture
    #TODO ICCapture module is linux only so class not included in the package untill windows support is added
    pass
elif (camera_type == CAMERA_TYPE_OPENCV_SPLIT): # OpenCV
    camL = CVCapture(0)
    camR = CVCapture(1)
    stcam = StereoCaptureCVSplit(camL,camR)
elif (camera_type == CAMERA_TYPE_OPENCV_DUAL): # OpenCV (One camera has left and right stored in red and green channels)
    cam = CVCapture(0)
    stcam = StereoCaptureCVDual(cam)
elif (camera_type == CAMERA_TYPE_OPENCV_IMAGE): # Read left and right from image
    camL = CVImageCapture("pyStereo3D/SampleData/deimos_left.png")
    camR = CVImageCapture("pyStereo3D/SampleData/deimos_right.png")
    stcam = StereoCaptureCVSplit(camL,camR)
elif (camera_type == CAMERA_TYPE_OPENCV_IMAGE_RECT): # Read left and right from image
    isRectified = True
    left_cal_file = folder + "phobos_left.yaml"
    right_cal_file = folder + "phobos_right.yaml"
    camL = CVImageCapture("pyStereo3D/SampleData/phobos_rect_left.png")
    camR = CVImageCapture("pyStereo3D/SampleData/phobos_rect_right.png")
    stcam = StereoCaptureCVSplit(camL,camR)
elif (camera_type == CAMERA_TYPE_OPENCV_FOLDER): # Read left and right from images in folders (MUST be in seperate folders)
    camL = CVImageFolderCapture("pyStereo3D/SampleData/deimos_left/")
    camR = CVImageFolderCapture("pyStereo3D/SampleData/deimos_right/")
    stcam = StereoCaptureCVSplit(camL,camR)
else:
    print("Invalid camera type.")
    exit()

# load stereo camera into generic stereo camera capture class
stcap = StereoCapture(stcam)

# setup Stereo3D
s3D = Stereo3D(left_cal_file,right_cal_file,stcap)
# run Stereo3D GUI for generating 3D
s3D.run(folder,isRectified=isRectified) #if input images are already rectified then set 'isRectified' to True