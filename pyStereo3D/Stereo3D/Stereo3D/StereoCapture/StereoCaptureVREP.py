from Stereo3D.StereoCapture.VREPCapture import VREPCapture
import numpy as np
import cv2

class StereoCaptureVREP():
    def __init__(self,camL,camR):
        """
        Initialisation function for StereoCaptureVREP class.
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
    left_api_port = "20000"
    right_api_port = "20001"
    camL = VREPCapture(left_api_port)
    camR = VREPCapture(right_api_port)
    stcam = StereoCaptureVREP(camL,camR)
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