__author__ = "Ben Knight @ Industrial 3D Robotics"
__maintainer__ = "Ben Knight"
__email__ = "bknight@i3drobotics.com"
__copyright__ = "Copyright 2020, Industrial 3D Robotics"
__license__ = "MIT"
__docformat__ = 'reStructuredText'

from Stereo3D.StereoCapture.PylonCapture import PylonCapture
from Stereo3D.StereoCapture.CVCapture import CVCapture
from Stereo3D.StereoCapture.CVImageCapture import CVImageCapture
from Stereo3D.StereoCapture.CVImageFolderCapture import CVImageFolderCapture
from Stereo3D.StereoCapture.VREPCapture import VREPCapture
from Stereo3D.StereoCapture.StereoCapturePylon import StereoCapturePylon
from Stereo3D.StereoCapture.StereoCaptureCVSplit import StereoCaptureCVSplit
from Stereo3D.StereoCapture.StereoCaptureCVDual import StereoCaptureCVDual
from Stereo3D.StereoCapture.StereoCaptureVREP import StereoCaptureVREP

from pymsgbox import *
import numpy as np
import cv2

class StereoCapture():
    """
    Python tool for capturing stereo pairs from difference sources
    See README for details on usages: https://github.com/i3drobotics/StereoCapture/blob/master/pyStereoCapture/StereoCapture/README.md
    """
    def __init__(self,stcam,param=None):
        """
        Initialisation function for StereoCapture class.
        :param cam: camera to use as stereo device. This should one of three types: StereoCapturePylon, StereoCaptureCVSplit or StereoCaptureCVDual. 
        Some prebuild stereo cameras can be used by using: "Deimos"/"Phobos"/"Image"/"Pylon"
        :type cam: StereoCapturePylon or StereoCaptureCVSplit or StereoCaptureCVDual or String
        """
        self.stcam = None
        if (stcam == "Deimos" or stcam == "deimos"):
            if (param is not None):
                print("Setting up as deimos stereo camera...")
                cam = CVCapture(param)
                stcam = StereoCaptureCVDual(cam)
            else:
                print("param MUST be defined when using pre-made stereo capture object. (int)usb_camera_index")
                return
        elif (stcam == "Phobos" or stcam == "phobos" or stcam == "Pylon" or stcam == "pylon"):
            print("Setting up as phobos stereo camera...")
            if (param is not None):
                left_camera_serial = param[0]
                right_camera_serial = param[1]
                camL = PylonCapture(left_camera_serial,trigger_mode=True)
                camR = PylonCapture(right_camera_serial,trigger_mode=True)
                stcam = StereoCapturePylon(camL,camR)
            else:
                print("param MUST be defined when using pre-made stereo capture object. (Array)[Left_serial,Right_serial]")
                return
        elif (stcam == "Image" or stcam == "image"):
            print("Setting up as image stereo camera...")
            if (param is not None):
                #TODO check files exist
                camL = CVImageCapture(param[0])
                camR = CVImageCapture(param[1])
                stcam = StereoCaptureCVSplit(camL,camR)
            else:
                print("param MUST be defined when using pre-made stereo capture object. (Array)[Left_image_file,Right_image_file]")
                return
        self.stcam = stcam
        self.flip_h = False
        self.flip_v = False

    def connect(self):
        """
        Connect to stereo camera.
        :returns: success of connection
        :rtype: bool
        """
        if (self.stcam is not None):
            res = self.stcam.connect()
        else:
            print("Stereo camera not defined")
            res = False
        return res

    def grab(self):
        """
        Grab images from stereo camera
        :returns: success of capture, image left, image right
        :rtype: bool, numpy.array, numpy.array
        """
        if (self.stcam is not None):
            res,imageL,imageR = self.stcam.grab()
            if (self.flip_h):
                imageL = self.flip_image_h(imageL)
                imageR = self.flip_image_h(imageR)
            if (self.flip_v):
                imageL = self.flip_image_v(imageL)
                imageR = self.flip_image_v(imageR)
        else:
            print("Stereo camera is not defined")
            res = False
        return res,imageL,imageR

    def flip_image_h(self,image):
        flipImage = cv2.flip(image, 1)
        return flipImage

    def flip_image_v(self,image):
        flipImage = cv2.flip(image, 0)
        return flipImage

    def save_images(self, image_left, image_right, defaultSaveFolder="", left_file_string="left.png", right_file_string="right.png", confirm_folder=True):
        """
        Save stereo images to files
        :param image_left: left camera image matrix
        :param image_right: right camera image matrix
        :param defaultSaveFolder: default folder to save the images to (will still ask for confirmation)
        :param left_file_string: left image filename
        :param right_file_string: right image filename
        :type image_left: numpy
        :type image_right: numpy
        :type defaultSaveFolder: string
        :type left_file_string: string
        :type right_file_string: string
        """
        # prompt user for save location
        if (confirm_folder):
            resp = prompt(text='Saving image pair to path: ', title='Save Image Pair' , default=defaultSaveFolder)
        else:
            resp = defaultSaveFolder
        if (resp is not None and image_left is not None and image_right is not None):
            # define name of output images
            left_image_filename = resp + left_file_string
            right_image_filename = resp + right_file_string

            print("Saving stereo image pair...")
            cv2.imwrite(left_image_filename,image_left)
            cv2.imwrite(right_image_filename,image_right)
            print("Stereo image pair saved")
            if (confirm_folder):
                alert('Stereo image pair saved.', 'Save Image Pair')
        else:
            print("Invalid prompt response or images are empty")

    def image_resize(self, image, width = None, height = None, inter = cv2.INTER_AREA):
        """
        Resize image based on height or width while maintaning aspect ratio
        :param image: image matrix
        :param width: desired width of output image (can only use width or height not both)
        :param height: desired height of output image (can only use width or height not both)
        :param inter: opencv resize method (default: cv2.INTER_AREA)
        :type image: numpy
        :type width: int
        :type height: int
        :type inter: int
        """
        # initialize the dimensions of the image to be resized and
        # grab the image size
        dim = None
        (h, w) = image.shape[:2]

        # if both the width and height are None, then return the
        # original image
        if width is None and height is None:
            return image

        # check to see if the width is None
        if width is None:
            # calculate the ratio of the height and construct the
            # dimensions
            r = height / float(h)
            dim = (int(w * r), height)

        # otherwise, the height is None
        else:
            # calculate the ratio of the width and construct the
            # dimensions
            r = width / float(w)
            dim = (width, int(h * r))

        # resize the image
        resized = cv2.resize(image, dim, interpolation = inter)

        # return the resized image
        return resized

    def runGui(self,defaultSaveFolder="",confirm_folder=True):
        """
        Display GUI for viewing stereo camera feed
        """
        image_right = None
        image_left = None
        if (self.stcam is not None):
            self.connect()
            save_index = 0
            while(True):
                res,in_image_left,in_image_right = self.stcam.grab()
                if (res):
                    image_right = in_image_right
                    image_left = in_image_left
                    stereo_image = np.concatenate((image_left, image_right), axis=1)
                    stereo_image_resized = self.image_resize(stereo_image,1280)
                    cv2.imshow('Stereo Image', stereo_image_resized)
                k = cv2.waitKey(1)
                if k == ord('q'):
                    break
                elif k == ord('s'): # save stereo image pair
                    left_file_string=str(save_index)+"_l.png"
                    right_file_string=str(save_index)+"_r.png"
                    self.save_images(image_left,image_right,defaultSaveFolder,left_file_string,right_file_string,confirm_folder)
                    save_index += 1
            self.stcam.close()
        else:
            print("Stereo camera is not defined")

    def close(self):
        """
        Close connection to camera
        """
        if (self.stcam is not None):
            self.stcam.close()

if __name__ == "__main__":
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
        stcap = StereoCapture("Image",["../SampleData/left.png","../SampleData/right.png"])
    else:
        print("Invalid camera type.")
        exit()

    stcap.runGui()