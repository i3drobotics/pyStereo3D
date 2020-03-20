from Stereo3D import Stereo3D, StereoCalibration
from Stereo3D.StereoCapture import *

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
    stcap = StereoCapture("Image",["../../SampleData/deimos_left.png","../../SampleData/deimos_right.png"])
elif (camera_type == CAMERA_TYPE_VREP):
    left_api_port = 20000
    left_vision_sensor_name = "StereoCameraLeft"
    right_api_port = 20001
    right_vision_sensor_name = "StereoCameraRight"
    camL = VREPCapture(left_api_port,left_vision_sensor_name)
    camR = VREPCapture(right_api_port,right_vision_sensor_name)
    stcapVREP = StereoCaptureVREP(camL,camR)
    stcap = StereoCapture(stcapVREP)
else:
    print("Invalid camera type.")
    exit()

# define inout folder
folder = "../../SampleData/"

stcap.runGui(folder)