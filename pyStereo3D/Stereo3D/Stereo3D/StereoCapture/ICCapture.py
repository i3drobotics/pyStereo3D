#TODO TIS module is linux only so class not included in the package untill windows support is added

import cv2
import numpy as np
import os
import TIS
import time
from collections import namedtuple

# This sample shows, how to get an image in a callback and use trigger or software trigger
# needed packages:
# python-opencv
# python-gst-1.0
# tiscamera

class CustomData:
        ''' Example class for user data passed to the on new image callback function
        '''

        def __init__(self, newImageReceived, image):
                self.newImageReceived = newImageReceived
                self.image = image
                self.busy = False

class ICCapture():
    def __init__(self,serial,width,height,fps,color=False):
        self.serial = serial
        self.width = width
        self.height = height
        self.fps = fps
        self.color = color

    def connect(self):
        self.CD = CustomData(False, None)
        # Open camera, set video format, framerate and determine, whether the sink is color or bw
        # Parameters: Serialnumber, width, height, framerate (numerator only) , color
        # If color is False, then monochrome / bw format is in memory. If color is True, then RGB32
        # colorformat is in memory
        self.Tis = TIS.TIS(self.serial, self.width, self.height, self.fps, False)

        self.Tis.Set_Image_Callback(self.on_new_image, self.CD)

        # Tis.Set_Property("Trigger Mode", "Off") # Use this line for GigE cameras
        self.Tis.Set_Property("Trigger Mode", False)
        self.CD.busy = True # Avoid, that we handle image, while we are in the pipeline start phase
        # Start the pipeline
        print("starting pipeline...")
        self.Tis.Start_pipeline()
        print("pipeline setup")

        #self.Tis.List_Properties()

        # Tis.Set_Property("Trigger Mode", "On") # Use this line for GigE cameras
        self.Tis.Set_Property("Trigger Mode", False)
        cv2.waitKey(1000)

        self.CD.busy = False  # Now the callback function does something on a trigger


        # Query the gain auto and current value :
        print("Gain Auto : %s " % self.Tis.Get_Property("Gain Auto").value)
        print("Gain : %d" % self.Tis.Get_Property("Gain").value)

        # Check, whether gain auto is enabled. If so, disable it.

        if self.Tis.Get_Property("Gain Auto").value :
                self.Tis.Set_Property("Gain Auto",False)
                print("Gain Auto now : %s " % self.Tis.Get_Property("Gain Auto").value)

        self.Tis.Set_Property("Gain",0)

        # Now do the same with exposure. Disable automatic if it was enabled
        # then set an exposure time.
        if self.Tis.Get_Property("Exposure Auto").value :
                self.Tis.Set_Property("Exposure Auto", False)
                print("Exposure Auto now : %s " % self.Tis.Get_Property("Exposure Auto").value)

        self.Tis.Set_Property("Exposure Time (us)", 15000)

        print("Exposure Auto : %s " % self.Tis.Get_Property("Exposure Auto").value)
        print("Exposure : %d" % self.Tis.Get_Property("Exposure Time (us)").value)

    def grab(self):
        #time.sleep(1)
        self.Tis.Set_Property("Software Trigger",1) # Send a software trigger

        # Wait for a new image. Use 10 tries.
        tries = 10
        while self.CD.newImageReceived is False and tries > 0:
                time.sleep(0.1)
                tries -= 1

        # Check, whether there is a new image and handle it.
        if self.CD.newImageReceived is True:
                self.CD.newImageReceived = False
                image = self.CD.image
                return True,image
                #cv2.imshow('Window', CD.image)
        else:
                print("No image received")
                return False,None

    def on_new_image(self,tis, userdata):
        '''
        Callback function, which will be called by the TIS class
        :param tis: the camera TIS class, that calls this callback
        :param userdata: This is a class with user data, filled by this call.
        :return:
        '''
        # Avoid being called, while the callback is busy
        if userdata.busy is True:
                return

        userdata.busy = True
        userdata.newImageReceived = True
        userdata.image = tis.Get_image()
        userdata.busy = False
        
    def close(self):
        self.Tis.Stop_pipeline()

if __name__ == '__main__':
    camera_serial = "27810457"
    cam = ICCapture(camera_serial, 2448, 2048, 35)
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