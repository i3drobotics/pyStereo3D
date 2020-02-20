from Stereo3D import Stereo3D
from StereoCapture import StereoCapture

CAMERA_TYPE_PHOBOS = 0
CAMERA_TYPE_DEIMOS = 1
CAMERA_TYPE_PYLON = 2
CAMERA_TYPE_IMAGE = 3

camera_type = CAMERA_TYPE_DEIMOS

stcap = None
if (camera_type == CAMERA_TYPE_PHOBOS):
    stcap = StereoCapture("Phobos",["22864917","22864912"])
elif (camera_type == CAMERA_TYPE_PYLON):
    stcap = StereoCapture("Pylon",["22864917","22864912"])
elif (camera_type == CAMERA_TYPE_DEIMOS):
    stcap = StereoCapture("Deimos",0)
elif (camera_type == CAMERA_TYPE_IMAGE):
    stcap = StereoCapture("Image",["pyStereo3D/SampleData/deimos_left.png","pyStereo3D/SampleData/deimos_right.png"])
else:
    print("Invalid camera type.")
    exit()

# define inout folder
folder = "pyStereo3D/SampleData/"
# define calibration files for left and right image
left_cal_file = folder + "deimos_left.yaml"
right_cal_file = folder + "deimos_right.yaml"

# setup Stereo3D
s3D = Stereo3D(left_cal_file,right_cal_file,stcap)
# run Stereo3D GUI for generating 3D
s3D.run(folder)