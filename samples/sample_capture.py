from stereo3d import Stereo3D, StereoCalibration
from stereo3d.stereocapture import StereoCapture, StereoCaptureVREP

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
    stcap = StereoCapture("Phobos",["22864917","22864912"])
elif (camera_type == CAMERA_TYPE_PYLON):
    stcap = StereoCapture("Pylon",["22864917","22864912"])
elif (camera_type == CAMERA_TYPE_DEIMOS):
    camera_name = "deimos"
    stcap = StereoCapture("Deimos",0)
elif (camera_type == CAMERA_TYPE_IMAGE):
    camera_name = "deimos"
    stcap = StereoCapture("Image",["sample_data/deimos_left.png","sample_data/deimos_right.png"])
elif (camera_type == CAMERA_TYPE_VREP):
    api_port = 20000
    left_vision_sensor_name = "StereoCameraLeft"
    right_vision_sensor_name = "StereoCameraRight"
    stcapVREP = StereoCaptureVREP(left_vision_sensor_name,right_vision_sensor_name,api_port)
    stcap = StereoCapture(stcapVREP)
else:
    print("Invalid camera type.")
    exit()

# define inout folder
folder = "sample_data/"

stcap.runGui(folder)