from stereo3d.stereocapture.coppeliasim import VREPConnection, VREPStereoVisionSensor
import numpy as np
import cv2

class StereoCaptureVREP():
    def __init__(self,left_camera_name,right_camera_name,api_port):
        """
        Initialisation function for StereoCaptureVREP class.
        :param camL: left camera
        :param camR: right camera
        :type camL: PylonCapture
        :type camR: PylonCapture
        """
        self.vrep_connection = None
        self.left_camera_name = left_camera_name
        self.right_camera_name = right_camera_name
        self.api_port = api_port
        self.camera = None

    def connect(self):
        """
        Connect to cameras.
        :returns: success of connection
        :rtype: bool
        """
        self.vrep_connection = VREPConnection(self.api_port)
        res, clientID = self.vrep_connection.connect()
        if (res):
            self.camera = VREPStereoVisionSensor(self.left_camera_name,self.right_camera_name,clientID)
            return self.camera.connect()
        else:
            return False

    def grab(self):
        """
        Grab images from each camera in stereo pair
        :returns: success of capture, image left, image right
        :rtype: bool, numpy.array, numpy.array
        """
        if (self.camera.isConnected()):
            res, image_l, image_r = self.camera.capture()
            if (res):
                image_l_mono = cv2.cvtColor(image_l, cv2.COLOR_BGR2GRAY)
                image_r_mono = cv2.cvtColor(image_r, cv2.COLOR_BGR2GRAY)
                return res, image_l_mono, image_r_mono
            else:
                False, None, None
        else:
            return False, None, None

    def close(self):
        """
        Close connection to cameras
        """
        self.camera.close()

if __name__ == "__main__":
    api_port = 20000
    left_camera_name = "StereoCameraLeft"
    right_camera_name = "StereoCameraRight"
    stcam = StereoCaptureVREP(left_camera_name,right_camera_name,api_port)
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