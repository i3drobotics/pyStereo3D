from Stereo3D import Stereo3D, StereoCapture, StereoCalibration

CAMERA_TYPE_PHOBOS = 0
CAMERA_TYPE_DEIMOS = 1
CAMERA_TYPE_PYLON = 2
CAMERA_TYPE_IMAGE = 3

camera_type = CAMERA_TYPE_PHOBOS

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
    stcap = StereoCapture("Image",["../../SampleData/deimos_left.png","../../SampleData/deimos_right.png"])
else:
    print("Invalid camera type.")
    exit()

# define inout folder
folder = "../../SampleData/"

stcap.runGui(folder)