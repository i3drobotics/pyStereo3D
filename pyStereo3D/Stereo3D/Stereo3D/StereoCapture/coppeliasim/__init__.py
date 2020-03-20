# Make sure to have the server side running in V-REP:
# in a child script of a V-REP scene, add following command
# to be executed just once, at simulation start:
#
# simRemoteApi.start(19999)
#
# then start simulation, and run this program.
#
# IMPORTANT: for each successful call to simxStart, there
# should be a corresponding call to simxFinish at the end!

try:
    from Stereo3D.StereoCapture.coppeliasim import sim as vrep
except:
    print('--------------------------------------------------------------')
    print('"sim.py" could not be imported. This means very probably that')
    print('either "sim.py" or the remoteApi library could not be found.')
    print('Make sure both are in the same folder as this file,')
    print('or appropriately adjust the file "sim.py"')
    print('--------------------------------------------------------------')
    print('')

import time
import cv2
import os
import numpy as np
import threading

#script_path = os.path.dirname(os.path.realpath(__file__))

class VREPConnection:
    def __init__(self,port=19999,ip='127.0.0.1'):
        self.port = port
        self.ip = ip

    def vrep_connect(self):
        clientID=vrep.simxStart(self.ip,self.port,True,True,5000,1) # Connect to V-REP
        if clientID !=-1:
            print ('Connected to remote API server')
            return True, clientID
        else:
            print ('Failed to connect to API server')
            return False, clientID

    def connect(self,retry=False):
        if (retry):
            connected = False
            while(connected == False):
                connected,self.clientID = self.vrep_connect()
            return True,self.clientID
        else:
            return self.vrep_connect()

    def prep_object_pose(self, object_handle, relative_to_handle):
        res1, position = vrep.simxGetObjectPosition(self.clientID, object_handle, relative_to_handle, vrep.simx_opmode_streaming)
        res2, orienation = vrep.simxGetObjectOrientation(self.clientID, object_handle, relative_to_handle, vrep.simx_opmode_streaming)
        if (res1 == 1 and res2 == 1):
            return True
        else:
            print("Failed to setup object pose stream")
            print("{},{},{},{},{}".format(res1,res2))
            return False

    def send_int_signal(self,signal_name,value):
        vrep.simxSetIntegerSignal(self.clientID,signal_name,value,vrep.simx_opmode_oneshot_wait)

    def get_object_pose(self, object_handle, relative_to_handle):
        res = False
        res1, position = vrep.simxGetObjectPosition(self.clientID, object_handle, relative_to_handle, vrep.simx_opmode_buffer)
        res2, orienation = vrep.simxGetObjectOrientation(self.clientID, object_handle, relative_to_handle, vrep.simx_opmode_buffer)
        if (res1==vrep.simx_return_ok and res2==vrep.simx_return_ok):
            res = True
        return res, position, orienation
        
class VREPVisionSensor:
    def __init__(self,name,clientID):
        self.name = name
        self.clientID = clientID
        self.cv_image = None
        self.cv_dimage = None
        self.running = False

    def VREP_image_to_cv_rgb(self,h, w, img):
        cv_image = np.array(img, dtype=np.uint8)
        cv_image.resize([h, w, 3])
        cv_image = cv2.flip(cv_image, 0)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        return cv_image

    def VREP_depth_to_np(self,h,w,img):
        cv_image = np.array(img, dtype=np.float32)
        cv_image.resize([h, w])
        cv_image = cv2.flip(cv_image, 0)
        return cv_image

    def disconnect(self):
        # Before closing the connection to V-REP, make sure that the last command sent out had time to arrive. You can guarantee this with (for example):
        vrep.simxGetPingTime(self.clientID)

        # Now close the connection to V-REP:
        vrep.simxFinish(self.clientID)

    def connect(self):
        #print ("Connecting to VREP vision sensor...")
        #vrep.simxFinish(-1) # just in case, close all opened connections
        #self.clientID=vrep.simxStart('127.0.0.1',self.port,True,True,5000,1) # Connect to V-REP
        if self.clientID !=-1:
            #print ('Connected to remote API server')
            res,self.handle=vrep.simxGetObjectHandle(self.clientID,self.name,vrep.simx_opmode_oneshot_wait)
            if (res == vrep.simx_return_ok):
                res1, self.resolution, self.image = vrep.simxGetVisionSensorImage(self.clientID,self.handle,0,vrep.simx_opmode_streaming)
                res2, resol, self.depthImage = vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.handle,vrep.simx_opmode_streaming)
                res3, self.nearClip = vrep.simxGetObjectFloatParameter(self.clientID,self.handle,vrep.sim_visionfloatparam_near_clipping,vrep.simx_opmode_streaming)
                res4, self.farClip = vrep.simxGetObjectFloatParameter(self.clientID,self.handle,vrep.sim_visionfloatparam_far_clipping,vrep.simx_opmode_streaming)
                res5, self.perspective_angle = vrep.simxGetObjectFloatParameter(self.clientID,self.handle,vrep.sim_visionfloatparam_perspective_angle,vrep.simx_opmode_streaming)

                if (res1 == 1 and res2 == 1 and res3 == 1 and res4 == 1 and res5 == 1):
                    return True
                else:
                    print("Failed to set vision sensor properties")
                    print("{},{},{},{},{}".format(res1,res2,res3,res4,res5))
                    return False
            else:
                print("Failed to connect to vision sensor object: {}".format(res))
                return False
        else:
            print("Failed to connect VREP")
            return False

    def isConnected(self):
        if (vrep.simxGetConnectionId(self.clientID)!=-1):
            return True
        else:
            return False

    def isInitalised(self):
        return self.initalised

    def capture(self):
        if (vrep.simxGetConnectionId(self.clientID)!=-1):
            #print("Getting image from VREP...")
            res1, self.resolution, self.image = vrep.simxGetVisionSensorImage(self.clientID,self.handle,0,vrep.simx_opmode_buffer)
            #res1, self.resolution, self.image = vrep.simxGetVisionSensorImage(self.clientID,self.handle,0,vrep.simx_opmode_oneshot)
            #print("Getting depth image from VREP...")
            res2, resol, self.depthImage=vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.handle,vrep.simx_opmode_buffer)
            #res2, resol, self.depthImage=vrep.simxGetVisionSensorDepthBuffer(self.clientID,self.handle,vrep.simx_opmode_oneshot)

            if res1==vrep.simx_return_ok and res2==vrep.simx_return_ok:
                #print("Converting VREP image to numpy array...")
                cv_image = self.VREP_image_to_cv_rgb(self.resolution[1],self.resolution[0],self.image)
                cv_dimage = self.VREP_depth_to_np(self.resolution[1],self.resolution[0],self.depthImage)

                self.initalised = True
                
                return True, cv_image, cv_dimage
            else:
                #print("Failed to capture vision sensor images: {},{}".format(res1,res2))
                return False, None, None
        else:
            print("Failed to connect to VREP")
            return False, None, None

    def run(self):
        self.initalised = False
        res = self.connect()
        if (res):
            self.running = True
            while (self.isConnected() and self.running):
                res, cv_image, cv_dimage = self.capture()
                if (res):
                    self.cv_image = cv_image
                    self.cv_dimage = cv_dimage

    def stop_threaded(self):
        self.running = False

    def run_threaded(self):
        self.thread = threading.Thread(target=self.run)
        self.thread.daemon=True
        self.thread.start()
