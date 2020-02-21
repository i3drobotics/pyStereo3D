import cv2
import numpy as np

class CVCapture():
    def __init__(self,camera_index):
        """
        Initialisation function for CVCapture class.
        :param camera_index: id of the video capturing device to open
        :type camera_index: int
        """
        self.camera = None
        self.camera_index = camera_index

    def connect(self):
        """
        Connect to camera using opencv VideoCaputre
        :returns: success of connection
        :rtype: bool
        """
        self.camera = cv2.VideoCapture(self.camera_index)
        return True

    def grab(self):
        """
        Grab image from camera
        :returns: success of capture, image
        :rtype: bool, numpy.array
        """
        ret, image = self.camera.read()
        return ret, image

    def close(self):
        """
        Close connection to camera
        """
        self.camera.release()

if __name__ == '__main__':
    camera_index = 0
    cam = CVCapture(camera_index)
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