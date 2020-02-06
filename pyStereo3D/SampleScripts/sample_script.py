from Stereo3D import Stereo3D
from StereoCapture import StereoCapture, CVCapture, CVImageCapture, StereoCaptureCVDual, StereoCaptureCVSplit

'''
# create opencv camera (CVCapture object)
cvcam = CVCapture(0)
# create opencv stereo camera (StereoCaptureCVDual object)
stcvcam = StereoCaptureCVDual(cvcam)
# create generic stereo camera (StereoCapture object)
stcam = StereoCapture(stcvcam)
'''

# create opencv camera (CVCapture object)
cvcamL = CVImageCapture("pyStereo3D/SampleData/left.png")
cvcamR = CVImageCapture("pyStereo3D/SampleData/right.png")
# create opencv stereo camera (StereoCaptureCVDual object)
stcvcam = StereoCaptureCVSplit(cvcamL,cvcamR)
# create generic stereo camera (StereoCapture object)
stcam = StereoCapture(stcvcam)

# define inout folder
folder = "pyStereo3D/SampleData/"
# define calibration files for left and right image
left_cal_file = folder + "left.yaml"
right_cal_file = folder + "right.yaml"

s3D = Stereo3D(left_cal_file,right_cal_file,stcam)
s3D.run(folder)