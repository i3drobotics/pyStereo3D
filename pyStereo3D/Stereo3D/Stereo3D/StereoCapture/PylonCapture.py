import cv2
import numpy as np
from pypylon import pylon
import os

class PylonCapture():
    PIXEL_FORMAT_MONO8 = "Mono8"

    def __init__(self,serial,trigger_mode=None,pixel_format=None,packet_size=None,inter_packet_delay=None,binning=None,max_buffer=None):
        """
        Initialisation function for PylonCapture class.
        :param serial: serial name of the camera
        :param trigger_mode: camera trigger mode
        :param pixel_format: camera pixel format (e.g. Mono8)
        :param packet_size: network packet size. Can be useful to change this if you are getting lost packets.
        :param inter_packet_delay: network packet delay. Can be useful to change this if you are getting lost packets.
        :type serial: string
        :type trigger_mode: bool
        :type pixel_format: string
        :type packet_size: int
        :type inter_packet_delay: int
        """
        self.camera = None
        self.serial = serial
        self.trigger_mode = trigger_mode
        self.pixel_format = pixel_format
        self.packet_size = packet_size
        self.inter_packet_delay = inter_packet_delay
        self.binning = binning
        self.max_buffer = max_buffer

    def connect(self):
        """
        Connect to basler camera using Pylon interface.
        :returns: success
        :rtype: bool
        """
        # Get the transport layer factory.
        tlFactory = pylon.TlFactory.GetInstance()

        device = None

        for d in tlFactory.GetInstance().EnumerateDevices():
            if d.GetSerialNumber() == self.serial:
                device = d
                break
        else:
            print('Camera with {} serial number not found'.format(self.serial))
            return False

        self.camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateDevice(device))

        #TODO fix pypylon problem where settings are reset on startup (previous settings are wiped)
        self.camera.Open()

        if (self.trigger_mode is not None):
            print("Trigger mode: {}".format(self.trigger_mode))
            if (self.trigger_mode):
                self.camera.GetNodeMap().GetNode("TriggerMode").SetValue("On")
            else:
                self.camera.GetNodeMap().GetNode("TriggerMode").SetValue("Off")
        if (self.max_buffer is not None):
            print("Max buffer: {}".format(self.max_buffer))
            self.camera.MaxNumBuffer.SetValue(self.max_buffer)
        # TODO set exposure and gain in functions
        #self.camera.GetNodeMap().GetNode("ExposureAuto").SetValue("Off")
        #self.camera.GetNodeMap().GetNode("ExposureTimeRaw").SetValue(15000)
        #self.camera.GetNodeMap().GetNode("GainAuto").SetValue("Off")
        #self.camera.GetNodeMap().GetNode("GainRaw").SetValue(0)
        if (self.pixel_format is not None):
            print("Pixel format: {}".format(self.pixel_format))
            self.camera.GetNodeMap().GetNode("PixelFormat").SetValue(self.pixel_format)
        #self.camera.GetNodeMap().GetNode("Width").SetValue(2448)
        #self.camera.GetNodeMap().GetNode("Height").SetValue(2048)
        if (self.packet_size is not None):
            print("Packet size: {}".format(self.packet_size))
            self.camera.GetNodeMap().GetNode("GevSCPSPacketSize").SetValue(self.packet_size)
        if (self.inter_packet_delay is not None):
            print("Inter packet delay: {}".format(self.inter_packet_delay))
            self.camera.GetNodeMap().GetNode("GevSCPD").SetValue(self.inter_packet_delay)
        if (self.binning is not None):
            print("Binning: {}".format(self.binning))
            self.camera.GetNodeMap().GetNode("BinningHorizontalMode").SetValue("Average")
            self.camera.GetNodeMap().GetNode("BinningHorizontal").SetValue(self.binning)
            self.camera.GetNodeMap().GetNode("BinningVerticalMode").SetValue("Average")
            self.camera.GetNodeMap().GetNode("BinningVertical").SetValue(self.binning)

        self.camera.StartGrabbing()

        self.converter = pylon.ImageFormatConverter()
        #converter.OutputPixelFormat = pylon.PixelType_BGR8packed
        self.converter.OutputPixelFormat = pylon.PixelType_Mono8
        self.converter.OutputBitAlignment = pylon.OutputBitAlignment_MsbAligned
        return True

    def grab(self):
        """
        Grab image from camera
        :returns: success of capture, image
        :rtype: bool, numpy.array
        """
        if (self.camera is not None):
            if (self.camera.IsGrabbing()):
                grabResult = self.camera.RetrieveResult(
                    5000, pylon.TimeoutHandling_ThrowException)

                if grabResult.GrabSucceeded():
                    image = self.converter.Convert(grabResult)
                    frame = image.GetArray()
                    if frame is not None:
                        return True,frame
                    else:
                        return False,None
                else:
                    print("Error: ", grabResult.ErrorCode, grabResult.ErrorDescription)
                    return False,None
                grabResult.Release()
            else:
                return False,None
        else:
            return False,None

    def close(self):
        """
        Close connection to camera
        """
        self.camera.StopGrabbing()
        self.camera.Close()

if __name__ == '__main__':
    camera_serial = "22864917"
    cam = PylonCapture(camera_serial)
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