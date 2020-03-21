from Stereo3D.StereoCapture import StereoCapture
from Stereo3D.StereoCalibration import StereoCalibration
import numpy as np
import cv2
import os
import time
import glob
from pymsgbox import *
from pyntcloud import PyntCloud
import pandas as pd

class Stereo3D():
    
    def __init__(self,stereo_camera,stereo_calibration,stereo_matcher=None):
        """
        Initialisation function for PickPlace3D. Defines the devices used.
        If you would like to use the methods without connecting a camera then initaialise Stereo3D as 's3D = Stereo3D()'.
        :param left_cal_file: filepath to left image calibration file (e.g. left.yaml)
        :param right_cal_file: filepath to right image calibration file (e.g. right.yaml)
        :param stereo_camera: stereo camera used for generating 3D and 2D detection
        :type left_cal_file: string
        :type right_cal_file: string
        :type stereo_camera: StereoCapture.StereoCapture
        """
        self.change_camera(stereo_camera)
        self.stereo_calibration = stereo_calibration
        self.Q = self.stereo_calibration.stereo_cal["q"]

        self.image_left = None
        self.image_right = None
        self.rect_image_left = None
        self.rect_image_right = None
        self.disparity = None
        self.depth = None

        self.EXIT_CODE_QUIT = -1
        self.EXIT_CODE_GRAB_3D_SUCCESS = 1
        self.EXIT_CODE_FAILED_TO_GRAB_3D = -2

        self.save_index = 0

        self.cv_window_name_Controls = "[Stereo3D] Controls"
        self.cv_window_name_Images = "[Stereo3D] Images"

        cv2.namedWindow(self.cv_window_name_Controls,cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.cv_window_name_Images,cv2.WINDOW_NORMAL)

        cv2.setMouseCallback(self.cv_window_name_Images, self.on_window_mouse)

        cv2.resizeWindow(self.cv_window_name_Controls, 400,0 )

        self.change_matcher(stereo_matcher)

        # Once init has been called matcher can be changed e.g.
        # s3D = Stereo3D()
        # s3D.ply_header = "ply..."
        self.ply_header = (
            "ply\n"
            "format ascii 1.0\n"
            "element vertex %(vert_num)d\n"
            "property float x\n"
            "property float y\n"
            "property float z\n"
            "property uchar red\n"
            "property uchar green\n"
            "property uchar blue\n"
            "end_header\n")

    def change_camera(self,stereo_camera):
        self.stereo_camera = stereo_camera

    def change_matcher(self,stereo_matcher):
        default_min_disp = 1000
        default_num_disparities = 20
        default_block_size = 12
        default_uniqueness_ratio = 15
        default_texture_threshold = 15
        default_speckle_size = 0
        default_speckle_range = 500

        if (stereo_matcher is None):
            self.matcher = cv2.StereoBM_create()
            calc_block = (2 * default_block_size + 5)
            self.matcher.setBlockSize(calc_block)
            self.matcher.setMinDisparity(int(default_min_disp - 1000))
            self.matcher.setNumDisparities(16*(default_num_disparities+1))
            self.matcher.setUniquenessRatio(default_uniqueness_ratio)
            self.matcher.setTextureThreshold(default_texture_threshold)
            self.matcher.setSpeckleWindowSize(default_speckle_size)
            self.matcher.setSpeckleRange(default_speckle_range)
        else:
            if stereo_matcher == "BM":
                self.matcher = cv2.StereoBM_create()
                calc_block = (2 * default_block_size + 5)
                self.matcher.setBlockSize(calc_block)
                self.matcher.setMinDisparity(int(default_min_disp - 1000))
                self.matcher.setNumDisparities(16*(default_num_disparities+1))
                self.matcher.setUniquenessRatio(default_uniqueness_ratio)
                self.matcher.setTextureThreshold(default_texture_threshold)
                self.matcher.setSpeckleWindowSize(default_speckle_size)
                self.matcher.setSpeckleRange(default_speckle_range)
            elif stereo_matcher == "SGBM":
                self.matcher = cv2.StereoSGBM_create()
                calc_block = (2 * default_block_size + 5)
                self.matcher.setBlockSize(calc_block)
                self.matcher.setMinDisparity(int(default_min_disp - 1000))
                self.matcher.setNumDisparities(16*(default_num_disparities+1))
                self.matcher.setUniquenessRatio(default_uniqueness_ratio)
                #self.matcher.setTextureThreshold(default_texture_threshold)
                self.matcher.setSpeckleWindowSize(default_speckle_size)
                self.matcher.setSpeckleRange(default_speckle_range)
            else:
                self.matcher = stereo_matcher

        cv2.destroyWindow(self.cv_window_name_Controls)
        cv2.namedWindow(self.cv_window_name_Controls,cv2.WINDOW_NORMAL)

        cv2.createTrackbar("Min disp", self.cv_window_name_Controls , default_min_disp, 2000, self.on_min_disparity_trackbar)
        cv2.createTrackbar("Disp", self.cv_window_name_Controls , default_num_disparities, 30, self.on_num_disparities_trackbar)
        cv2.createTrackbar("Blck sze", self.cv_window_name_Controls , default_block_size, 100, self.on_block_size_trackbar)

        cv2.createTrackbar("Uniq", self.cv_window_name_Controls , default_uniqueness_ratio, 100, self.on_uniqueness_ratio_trackbar)
        if stereo_matcher == "BM":
            cv2.createTrackbar("Texture", self.cv_window_name_Controls , default_texture_threshold, 100, self.on_texture_threshold_trackbar)

        cv2.createTrackbar("Sp size", self.cv_window_name_Controls , default_speckle_size, 30, self.on_speckle_size_trackbar)
        cv2.createTrackbar("Sp range", self.cv_window_name_Controls , default_speckle_range, 1000, self.on_speckle_range_trackbar)

    def connect(self):
        """
        Connect to devices needed for the pick and place process.
        :returns: success
        :rtype: bool
        """
        res = self.stereo_camera.connect()
        return res

    def gen3D(self,left_image, right_image):
        disparity = self.matcher.compute(left_image,right_image).astype(np.float32) / 16.0
        return disparity

    def genDepth(self,disparity):
        depth = cv2.reprojectImageTo3D(disparity, self.Q)
        return depth

    def write_ply(self,filename, disp, depth, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        image = np.float32(image)
        image = image/255

        mask = disp > disp.min()
        points = depth[mask]
        image = image[mask]

        points = points.reshape(-1, 3)
        image = image.reshape(-1, 3)

        points = np.hstack((points, image))

        cloud = PyntCloud(pd.DataFrame(
            # same arguments that you are passing to visualize_pcl
            data=points,
            columns=["x", "y", "z", "red", "green", "blue"]))
        
        cloud.to_file(filename)
        
        #with open(filename, 'wb') as f:
        #    f.write((self.ply_header % dict(vert_num=len(points))).encode('utf-8'))
        #    #np.save(f, points, fmt='%f %f %f %d %d %d ')
        #    np.savetxt(f, points, fmt='%f %f %f %d %d %d ')

    def scale_disparity(self,disparity):
        minV, maxV,_,_ = cv2.minMaxLoc(disparity)
        if (maxV - minV != 0):
            scaled_disp = cv2.convertScaleAbs(disparity, alpha=255.0/(maxV - minV), beta=-minV * 255.0/(maxV - minV))
            return scaled_disp
        else:
            return np.zeros(disparity.shape, np.uint8)

    def on_min_disparity_trackbar(self,val):
        min_disp = int(val - 1000)
        self.matcher.setMinDisparity(min_disp)

    def on_block_size_trackbar(self,val):
        self.matcher.setBlockSize(2 * val + 5)

    def on_num_disparities_trackbar(self,val):
        self.matcher.setNumDisparities(16*(val+1))

    def on_texture_threshold_trackbar(self,val):
        self.matcher.setTextureThreshold(val)

    def on_uniqueness_ratio_trackbar(self,val):
        self.matcher.setUniquenessRatio(val)

    def on_speckle_size_trackbar(self,val):
        self.matcher.setSpeckleWindowSize(val)

    def on_speckle_range_trackbar(self,val):
        self.matcher.setSpeckleRange(val)

    def on_window_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            print("click")

    def grab3D(self,isRectified=False):
        res, image_left, image_right = self.stereo_camera.grab()
        if (res):
            self.image_left = image_left
            self.image_right = image_right
            if (isRectified):
                self.rect_image_left, self.rect_image_right = self.image_left, self.image_right
            else:
                self.rect_image_left, self.rect_image_right = self.stereo_calibration.rectify_pair(image_left,image_right)

            disp = self.gen3D(self.rect_image_left, self.rect_image_right)
            self.disparity = disp

            return True, disp
        else:
            return False, None

    def save_point_cloud(self, disparity, image, defaultSaveFolder="", points_file_string="output.ply", confirm_folder=True):
        # prompt user for save location
        resp = prompt(text='Saving 3D Point Cloud to path: ', title='Save 3D Point Cloud' , default=defaultSaveFolder)
        if (resp is not None and disparity is not None and image is not None):
            # define name of output point cloud ply file
            ply_filename = resp + points_file_string

            # generate depth from disparity
            print("Generating depth from disparity...")
            depth = self.genDepth(disparity)
            print("Saving point cloud...")
            # write 3D data to ply with color from image on points
            self.write_ply(ply_filename,disparity,depth,image)
            print("Point cloud save complete.")
            if (confirm_folder):
                alert('3D point cloud saved.', 'Save 3D Point Cloud')
        else:
            print("invalid prompt response or disparity/image is empty")

    def run_frame(self,defaultSaveFolder="",isRectified=False,confirm_folder=True):
        # grab 3D disparity from stereo camera
        res, disp = self.grab3D(isRectified)
        if res:
            # prepare images for displaying
            display_image = np.zeros((640, 480), np.uint8)

            rect_image_left_resized = self.stereo_camera.image_resize(self.rect_image_left, height=640)
            rect_image_right_resized = self.stereo_camera.image_resize(self.rect_image_right, height=640)

            disp_resized = self.scale_disparity(self.stereo_camera.image_resize(disp, height=640))
            left_right_dual = np.concatenate((rect_image_left_resized, rect_image_right_resized), axis=1)

            (lr_dual_h,lr_dual_w) = left_right_dual.shape
            (d_h,d_w) = disp_resized.shape

            spacer_width_raw = (lr_dual_w - d_w)
            if (spacer_width_raw % 2) == 0:
                #even
                spacer_width_1 = int((spacer_width_raw / 2))
                spacer_width_2 = int((spacer_width_raw / 2))
            else:
                #odd
                spacer_width_1 = int((spacer_width_raw / 2))
                spacer_width_2 = int((spacer_width_raw / 2) + 1)

            disp_spacer_1 = np.zeros((640, spacer_width_1), np.uint8)
            disp_spacer_2 = np.zeros((640, spacer_width_2), np.uint8)
            disp_spaced = np.concatenate((disp_spacer_1, disp_resized, disp_spacer_2), axis=1)

            display_image = np.concatenate((disp_spaced, left_right_dual), axis=0)
            display_image_resize = self.stereo_camera.image_resize(display_image, height=640)
            
            # display disparity with stereo images
            cv2.imshow(self.cv_window_name_Images, display_image_resize)

        k = cv2.waitKey(1)          
        if k == ord('q'): # exit if 'q' key pressed
            return self.EXIT_CODE_QUIT
        elif k == ord('s'): # save stereo image pair
            left_file_string=str(self.save_index)+"_l.png"
            right_file_string=str(self.save_index)+"_r.png"
            self.stereo_camera.save_images(self.image_left,self.image_right,defaultSaveFolder,left_file_string,right_file_string,confirm_folder)
            self.save_index += 1
        elif k == ord('r'): # save rectified stereo image pair
            left_file_string="rect_"+str(self.save_index)+"_l.png"
            right_file_string="rect_"+str(self.save_index)+"_r.png"
            self.stereo_camera.save_images(self.rect_image_left,self.rect_image_right,defaultSaveFolder,left_file_string,right_file_string,confirm_folder)
            self.save_index += 1
        elif k == ord('p'): # save 3D data as point cloud
            points_file_string = "points_"+str(self.save_index)+".ply"
            self.save_point_cloud(disp,self.rect_image_left,defaultSaveFolder,points_file_string,confirm_folder)
            self.save_index += 1
        elif k == ord('1'): # change tp OpenCV BM
            self.change_matcher("BM")
        elif k == ord('2'): # change to OpenCV SGBM
            self.change_matcher("SGBM")
        if cv2.getWindowProperty(self.cv_window_name_Images,cv2.WND_PROP_VISIBLE) < 1:        
            return self.EXIT_CODE_QUIT
        if cv2.getWindowProperty(self.cv_window_name_Controls,cv2.WND_PROP_VISIBLE) < 1:        
            return self.EXIT_CODE_QUIT

        if res:
            return self.EXIT_CODE_GRAB_3D_SUCCESS
        else:
            return self.EXIT_CODE_FAILED_TO_GRAB_3D

    def run(self,defaultSaveFolder="",isRectified=False,frame_delay=0,confirm_folder=True):
        # connect to stereo camera
        connected = False
        while(not connected):
            connected = self.connect()
            time.sleep(1)
        while(True):
            exit_code = self.run_frame(defaultSaveFolder,isRectified,confirm_folder)
            if (exit_code == self.EXIT_CODE_QUIT):
                break
            if (exit_code == self.EXIT_CODE_FAILED_TO_GRAB_3D):
                time.sleep(1)
            time.sleep(frame_delay)
        
        self.stereo_camera.close()