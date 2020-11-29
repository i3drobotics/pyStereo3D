import cv2
import numpy as np
from stereo3d.stereocapture.coppeliasim import VREPConnection, VREPVisionSensor
import os

class VREPCapture():

    def __init__(self,api_port,vision_sensor_name):
        """
        Initialisation function for VREPCapture class.
        :param api_port: VREP api port
        :param vision_sensor_name: VREP Vision sensor name
        :type api_port: int
        :type vision_sensor_name: string
        """
        self.camera = None
        self.vrep_connection = None
        self.api_port = api_port
        self.vision_sensor_name = vision_sensor_name

    def connect(self):
        """
        Connect to vision sensor camera using VREP API interface.
        :returns: success
        :rtype: bool
        """
        self.vrep_connection = VREPConnection(self.api_port)
        res, clientID = self.vrep_connection.connect()

        if (res):
            self.camera = VREPVisionSensor(self.vision_sensor_name,clientID)
            self.camera.run_threaded()
        return res

    def grab(self):
        """
        Grab image from camera
        :returns: success of capture, image
        :rtype: bool, numpy.array
        """
        if(self.camera.isInitalised()):
            image = self.camera.cv_image
            image_mono = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            return True, image_mono
        else:
            return False, None

    def close(self):
        """
        Close connection to camera
        """
        self.camera.stop_threaded()

if __name__ == '__main__':
    api_port = 20000
    vision_sensor_name = "StereoCameraLeft"
    cam = VREPCapture(api_port,vision_sensor_name)
    cam.connect()
    while(True):
        res, image = cam.grab()
        if (res):
            image_resized = cv2.resize(image,(640,480))
            cv2.imshow('Image', image_resized)
            k = cv2.waitKey(1)
            if k == ord('q'):
                break
    cam.close()