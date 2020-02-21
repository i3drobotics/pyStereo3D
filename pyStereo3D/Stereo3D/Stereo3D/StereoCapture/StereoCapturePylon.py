from Stereo3D.StereoCapture.PylonCapture import PylonCapture
import numpy as np
import cv2

class StereoCapturePylon():
    def __init__(self,camL,camR):
        """
        Initialisation function for StereoCapturePylon class.
        :param camL: left camera
        :param camR: right camera
        :type camL: PylonCapture
        :type camR: PylonCapture
        """
        self.camL = camL
        self.camR = camR

    def connect(self):
        """
        Connect to cameras.
        :returns: success of connection
        :rtype: bool
        """
        resL = self.camL.connect()
        resR = self.camR.connect()
        res = resL and resR
        return res

    def grab(self):
        """
        Grab images from each camera in stereo pair
        :returns: success of capture, image left, image right
        :rtype: bool, numpy.array, numpy.array
        """
        resL,image_left = self.camL.grab()
        resR,image_right = self.camR.grab()
        res = resL and resR
        return res,image_left,image_right

    def close(self):
        """
        Close connection to cameras
        """
        self.camL.close()
        self.camR.close()

if __name__ == "__main__":
    left_camera_serial = "22864917"
    right_camera_serial = "22864912"
    camL = PylonCapture(left_camera_serial)
    camR = PylonCapture(right_camera_serial)
    stcam = StereoCapturePylon(camL,camR)
    stcam.connect()
    while(True):
        res,imageL,imageR = stcam.grab()
        if (res):
            stereo_image = np.concatenate((imageL, imageR), axis=1)
            stereo_image_resized = cv2.resize(stereo_image,(1280,480))
            cv2.imshow('Stereo Image', stereo_image_resized)
            k = cv2.waitKey(1)
            if k == ord('q'):
                break
    stcam.close()