import cv2
import numpy as np

class CVImageCapture():
    def __init__(self,filename):
        """
        Initialisation function for CVCapture class.
        :param filename: filename of the image to read
        :type filename: string
        """
        self.image = None
        self.filename = filename

    def connect(self):
        """
        Connect to camera using opencv VideoCaputre
        :returns: success of connection
        :rtype: bool
        """
        self.image = cv2.imread(self.filename,cv2.IMREAD_GRAYSCALE)
        return True

    def grab(self):
        """
        Grab image from camera
        :returns: success of capture, image
        :rtype: bool, numpy.array
        """
        res = not np.shape(self.image) == ()
        return res, self.image

    def close(self):
        """
        Close connection
        """

if __name__ == '__main__':
    filename = "test.png"
    cam = CVImageCapture(filename)
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