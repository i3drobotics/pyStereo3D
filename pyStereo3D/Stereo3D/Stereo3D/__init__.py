import StereoCapture
import numpy as np
import cv2
import os
import time
from pymsgbox import *

class Stereo3D():
    
    def __init__(self,left_cal_file,right_cal_file,stereo_camera=None):
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
        self.stereo_camera = stereo_camera
        self.calLoaded, self.m_l, self.m_r, self.d_l, self.d_r, self.r_r, self.r_l, self.p_r, self.p_l = self.get_cal_from_file(left_cal_file,right_cal_file)
        self.Q = self.calc_q(self.m_l, self.p_r, self.p_l)

        self.image_left = None
        self.image_right = None
        self.rect_image_left = None
        self.rect_image_right = None
        self.disparity = None
        self.depth = None

        self.cv_window_name_Controls = "[Stereo3D] Controls"
        self.cv_window_name_Images = "[Stereo3D] Images"

        cv2.namedWindow(self.cv_window_name_Controls,cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.cv_window_name_Images,cv2.WINDOW_NORMAL)

        default_min_disp = 500
        default_num_disparities = 20
        default_block_size = 21
        default_uniqueness_ratio = 15
        default_texture_threshold = 15
        default_speckle_size = 0
        default_speckle_range = 500

        cv2.createTrackbar("Min disp", self.cv_window_name_Controls , default_min_disp, 1000, self.on_min_disparity_trackbar)
        cv2.createTrackbar("Disp", self.cv_window_name_Controls , default_num_disparities, 30, self.on_num_disparities_trackbar)
        cv2.createTrackbar("Blck sze", self.cv_window_name_Controls , default_block_size, 100, self.on_block_size_trackbar)

        cv2.createTrackbar("Uniq", self.cv_window_name_Controls , default_uniqueness_ratio, 100, self.on_uniqueness_ratio_trackbar)
        cv2.createTrackbar("Texture", self.cv_window_name_Controls , default_texture_threshold, 100, self.on_texture_threshold_trackbar)

        cv2.createTrackbar("Sp size", self.cv_window_name_Controls , default_speckle_size, 30, self.on_speckle_size_trackbar)
        cv2.createTrackbar("Sp range", self.cv_window_name_Controls , default_speckle_range, 100, self.on_speckle_range_trackbar)

        cv2.setMouseCallback(self.cv_window_name_Images, self.on_window_mouse)

        cv2.resizeWindow(self.cv_window_name_Controls, 400,0 )

        # Once init has been called matcher can be changed e.g.
        # s3D = Stereo3D()
        # s3D.matcher = cv2.StereoSGBM_create()
        self.matcher = cv2.StereoBM_create()
        calc_block = ( int(default_block_size / 2) * 2) + 5
        self.matcher.setBlockSize(calc_block)
        self.matcher.setMinDisparity(int(default_min_disp - 500))
        self.matcher.setNumDisparities(16*(default_num_disparities+1))
        self.matcher.setUniquenessRatio(default_uniqueness_ratio)
        self.matcher.setTextureThreshold(default_texture_threshold)
        self.matcher.setSpeckleWindowSize(default_speckle_size)
        self.matcher.setSpeckleRange(default_speckle_range)

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

    def connect(self):
        """
        Connect to devices needed for the pick and place process.
        :returns: success
        :rtype: bool
        """
        res = self.stereo_camera.connect()
        return res

    def get_cal_from_file(self, left_cal_file, right_cal_file):
        res = False
        fs_l = cv2.FileStorage(left_cal_file, cv2.FILE_STORAGE_READ)
        fs_r = cv2.FileStorage(right_cal_file, cv2.FILE_STORAGE_READ)
        if fs_l.isOpened() and fs_r.isOpened():
            m_l = fs_l.getNode("camera_matrix").mat()
            m_r = fs_r.getNode("camera_matrix").mat()
            d_l = fs_l.getNode("distortion_coefficients").mat()
            d_r = fs_r.getNode("distortion_coefficients").mat()
            r_l = fs_l.getNode("rectification_matrix").mat()
            r_r = fs_r.getNode("rectification_matrix").mat()
            p_l = fs_l.getNode("projection_matrix").mat()
            p_r = fs_r.getNode("projection_matrix").mat()
            res = True
        else:
            print("Failed to open calibration files")
            return False, None, None, None, None, None, None, None, None
        fs_l.release()
        fs_r.release()
        return res, m_l, m_r, d_l, d_r, r_r, r_l, p_r, p_l

    def calc_q(self, m_l, p_r, p_l):
        q = np.zeros(shape=(4,4))
        self.cx = p_l[0,2]
        self.cxr = p_r[0,2]
        self.cy = p_l[1,2]
        self.fx = m_l[0,0]
        self.fy = m_l[1,1]

        p14 = p_r[0,3]
        self.T = -p14 / self.fx

        q33 = -(self.cx - self.cxr)/self.T

        # calucalte Q values
        q[0,0] = 1.0
        q[0,3] = -self.cx
        q[1,1] = 1.0
        q[1,3] = -self.cy #501
        q[2,3] = self.fx
        q[3,2] = 1.0 / self.T
        q[3,3] = q33

        return q

    def rectify(self,left_image,right_image):
        mapL1, mapL2 = cv2.initUndistortRectifyMap(self.m_l, self.d_l, self.r_l, self.p_l, left_image.shape[::-1], cv2.CV_32FC1)
        mapR1, mapR2 = cv2.initUndistortRectifyMap(self.m_r, self.d_r, self.r_r, self.p_r, right_image.shape[::-1], cv2.CV_32FC1)
        rect_image_l = cv2.remap(left_image, mapL1, mapL2, cv2.INTER_LINEAR)
        rect_image_r = cv2.remap(right_image, mapR1, mapR2, cv2.INTER_LINEAR)
        return rect_image_l, rect_image_r

    def gen3D(self,left_image, right_image):
        disparity = self.matcher.compute(left_image,right_image).astype(np.float32) / 16.0
        return disparity

    def genDepth(self,disparity):
        depth = cv2.reprojectImageTo3D(disparity, self.Q)
        return depth

    def write_ply(self,filename, disp, depth, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        mask = disp > disp.min()
        points = depth[mask]
        image = image[mask]

        points = points.reshape(-1, 3)
        image = image.reshape(-1, 3)

        points = np.hstack([points, image])
        
        with open(filename, 'wb') as f:
            f.write((self.ply_header % dict(vert_num=len(points))).encode('utf-8'))
            np.savetxt(f, points, fmt='%f %f %f %d %d %d ')

    def scale_disparity(self,disparity):
        minV, maxV,_,_ = cv2.minMaxLoc(disparity)
        if (maxV - minV != 0):
            scaled_disp = cv2.convertScaleAbs(disparity, alpha=255.0/(maxV - minV), beta=-minV * 255.0/(maxV - minV))
            return scaled_disp
        else:
            return np.zeros(disparity.shape, np.uint8)

    def on_min_disparity_trackbar(self,val):
        min_disp = int(val - 500)
        self.matcher.setMinDisparity(min_disp)

    def on_block_size_trackbar(self,val):
        self.matcher.setBlockSize(( int(val / 2) * 2) + 5)

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

    def grab3D(self,isDisplayImages=False):
        res, image_left, image_right = self.stereo_camera.grab()
        if (res):
            self.image_left = image_left
            self.image_right = image_right
            self.rect_image_left, self.rect_image_right = self.rectify(image_left, image_right)

            disp = self.gen3D(self.rect_image_left, self.rect_image_right)
            self.disparity = disp

            display_image = np.zeros((640, 480), np.uint8)

            rect_image_left_resized = cv2.resize(self.rect_image_left, (1280, 960), interpolation = cv2.INTER_AREA)
            rect_image_right_resized = cv2.resize(self.rect_image_right, (1280, 960), interpolation = cv2.INTER_AREA)

            disp_resized = self.scale_disparity(cv2.resize(disp, (1280, 960), interpolation = cv2.INTER_AREA))
            left_right_dual = np.concatenate((rect_image_left_resized, rect_image_right_resized), axis=1)
            disp_spacer = np.zeros((960, 640), np.uint8)
            disp_spaced = np.concatenate((disp_spacer, disp_resized, disp_spacer), axis=1)
            display_image = np.concatenate((disp_spaced, left_right_dual), axis=0)

            display_image_resize = cv2.resize(display_image, (1280, 1280), interpolation = cv2.INTER_AREA)
            
            cv2.imshow(self.cv_window_name_Images, display_image_resize)

            return True, disp
        else:
            return False, None

    def save_images(self, image_left, image_right, defaultSaveFolder="", left_file_string="left.png", right_file_string="right.png"):
        # prompt user for save location
        resp = prompt(text='Saving image pair to path: ', title='Save Image Pair' , default=defaultSaveFolder)
        # define name of output images
        left_image_filename = resp + left_file_string
        right_image_filename = resp + right_file_string

        print("Saving stereo image pair...")
        cv2.imwrite(left_image_filename,image_left)
        cv2.imwrite(right_image_filename,image_right)
        print("Stereo image pair saved")
        alert('Stereo image pair saved.', 'Save Image Pair')

    def save_point_cloud(self, disparity, image, defaultSaveFolder="", points_file_string="output.ply"):
        # prompt user for save location
        resp = prompt(text='Saving 3D Point Cloud to path: ', title='Save 3D Point Cloud' , default=defaultSaveFolder)
        # define name of output point cloud ply file
        ply_filename = resp + points_file_string

        # generate depth from disparity
        print("Generating depth from disparity...")
        depth = self.genDepth(disparity)
        print("Saving point cloud...")
        # write 3D data to ply with color from image on points
        self.write_ply(ply_filename,disparity,depth,image)
        print("Point cloud save complete.")
        alert('3D point cloud saved.', 'Save 3D Point Cloud')

    def run(self,defaultSaveFolder="",frame_delay=0):
        if (self.calLoaded):
            # connect to stereo camera
            self.connect()
            save_index = 0
            while(True):
                # grab 3D disparity from stereo camera
                res, disp = self.grab3D()
                k = cv2.waitKey(1)          
                if k == ord('q'): # exit if 'q' key pressed
                    break
                elif k == ord('s'): # save stereo image pair
                    left_file_string=str(save_index)+"_l.png"
                    right_file_string=str(save_index)+"_r.png"
                    self.save_images(self.image_left,self.image_right,defaultSaveFolder,left_file_string,right_file_string)
                    save_index += 1
                elif k == ord('r'): # save rectified stereo image pair
                    left_file_string="rect_"+str(save_index)+"_l.png"
                    right_file_string="rect_"+str(save_index)+"_r.png"
                    self.save_images(self.rect_image_left,self.rect_image_right,defaultSaveFolder,left_file_string,right_file_string)
                    save_index += 1
                elif k == ord('p'): # save 3D data as point cloud
                    points_file_string = "points_"+str(save_index)+".ply"
                    self.save_point_cloud(disp,self.rect_image_left,defaultSaveFolder,points_file_string)
                    save_index += 1
                if cv2.getWindowProperty(self.cv_window_name_Images,cv2.WND_PROP_VISIBLE) < 1:        
                    break
                if cv2.getWindowProperty(self.cv_window_name_Controls,cv2.WND_PROP_VISIBLE) < 1:        
                    break
                time.sleep(frame_delay)
        else:
            print("Failed to calibration files so cannot continue.")

if __name__ == "__main__":
    # create opencv camera (CVCapture object)
    cvcam = StereoCapture.CVCapture(0)
    # create opencv stereo camera (StereoCaptureCVDual object)
    stcvcam = StereoCapture.StereoCaptureCVDual(cvcam)
    # create generic stereo camera (StereoCapture object)
    stcam = StereoCapture.StereoCapture(stcvcam)

    # define inout folder
    folder = "pyStereo3D/SampleData/"
    # define calibration files for left and right image
    left_cal_file = folder + "left.yaml"
    right_cal_file = folder + "right.yaml"
    
    s3D = Stereo3D(left_cal_file,right_cal_file,stcam)
    s3D.run(folder)