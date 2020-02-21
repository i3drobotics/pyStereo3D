from Stereo3D.StereoCapture.CVCapture import CVCapture
import numpy as np
import cv2

class StereoCaptureCVDual():
    def __init__(self,cam):
        """
        Initialisation function for StereoCaptureCVDual class.
        :param cam: camera to use as stereo device. This should be a camera that send the data as a single rgb image where red=left_image and green=right_image.
        :type cam: CVCapture
        """
        self.cam = cam

    def connect(self):
        """
        Connect to camera.
        :returns: success of connection
        :rtype: bool
        """
        res = self.cam.connect()
        return res

    def grab(self):
        """
        Grab images from stereo camera
        :returns: success of capture, image left, image right
        :rtype: bool, numpy.array, numpy.array
        """
        res,image = self.cam.grab()
        image_left = image.copy()
        image_right = image.copy()
        image_left = image_left[:, :, 1]
        image_right = image_right[:, :, 2]
        
        return res,image_left,image_right

    def close(self):
        """
        Close connection to camera
        """
        self.cam.close()

if __name__ == "__main__":
    cam = CVCapture(0)
    stcam = StereoCaptureCVDual(cam)
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