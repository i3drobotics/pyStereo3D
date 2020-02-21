import cv2
import os
import numpy as np

class CVImageFolderCapture():
    def __init__(self,folder):
        """
        Initialisation function for CVCapture class.
        :param folder: filename of the image to read
        :type folder: string
        """
        self.images = None
        self.image_index = 0
        self.folder = folder

    def connect(self):
        """
        Connect to camera using opencv VideoCaputre
        :returns: success of connection
        :rtype: bool
        """
        self.images = self.load_images_from_folder(self.folder)
        return True

    def load_images_from_folder(self,folder):
        images = []
        for filename in os.listdir(folder):
            img = cv2.imread(os.path.join(folder,filename),cv2.IMREAD_GRAYSCALE)
            if img is not None:
                images.append(img)
        return images

    def grab(self):
        """
        Grab image from camera
        :returns: success of capture, image
        :rtype: bool, numpy.array
        """
        image = self.images[self.image_index]
        self.image_index += 1
        if (self.image_index >= len(self.images)):
            self.image_index = 0
        return True, image

    def close(self):
        """
        Close connection
        """

if __name__ == '__main__':
    folder = "pyStereoCapture/SampleData/"
    cam = CVImageFolderCapture(folder)
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