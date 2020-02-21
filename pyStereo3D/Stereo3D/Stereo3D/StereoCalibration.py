import cv2
import numpy as np
import glob
import os

class StereoCalibration():
    def __init__(self):
        self.flags = 0
        self.flags |= cv2.CALIB_FIX_INTRINSIC
        # self.flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
        self.flags |= cv2.CALIB_USE_INTRINSIC_GUESS
        self.flags |= cv2.CALIB_FIX_FOCAL_LENGTH
        # self.flags |= cv2.CALIB_FIX_ASPECT_RATIO
        self.flags |= cv2.CALIB_ZERO_TANGENT_DIST
        # self.flags |= cv2.CALIB_RATIONAL_MODEL
        # self.flags |= cv2.CALIB_SAME_FOCAL_LENGTH
        # self.flags |= cv2.CALIB_FIX_K3
        # self.flags |= cv2.CALIB_FIX_K4
        # self.flags |= cv2.CALIB_FIX_K5

        self.criteria = (cv2.TERM_CRITERIA_EPS +
                         cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        self.stereocalib_criteria = (cv2.TERM_CRITERIA_MAX_ITER +
                                cv2.TERM_CRITERIA_EPS, 100, 1e-5)
        self.stereo_cal = None
        self.cv_window_name = "Calibration"

    def read_images(self, folder, wildcard):
        #TODO check folder exists
        images = glob.glob(os.path.join(folder,wildcard))
        images.sort()
        images_cv = []
        for fname in images:
            img = cv2.imread(fname,cv2.IMREAD_COLOR)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            images_cv.append(gray)
        return images_cv

    def write_calibration(self,output_folder):
        print("Writing calibration to output folder...")

        # write calibration xmls
        left_cal_xml_file = output_folder + "/left_calibration.xml"
        right_cal_xml_file = output_folder + "/right_calibration.xml"
        stereo_cal_xml_file = output_folder + "/stereo_calibration.xml"
        fs_l = cv2.FileStorage(left_cal_xml_file, cv2.FILE_STORAGE_WRITE)
        fs_l.write("cameraMatrix",self.stereo_cal["left"]["m"])
        fs_l.write("distCoeffs",self.stereo_cal["left"]["d"])
        fs_l.write("rms_error",self.stereo_cal["left"]["error"])
        fs_l.release()
        fs_r = cv2.FileStorage(right_cal_xml_file, cv2.FILE_STORAGE_WRITE)
        fs_r.write("cameraMatrix",self.stereo_cal["right"]["m"])
        fs_r.write("distCoeffs",self.stereo_cal["right"]["d"])
        fs_r.write("rms_error",self.stereo_cal["right"]["error"])
        fs_r.release()
        fs_s = cv2.FileStorage(stereo_cal_xml_file, cv2.FILE_STORAGE_WRITE)
        fs_s.write("R",self.stereo_cal["r"])
        fs_s.write("T",self.stereo_cal["t"])
        fs_s.write("Q",self.stereo_cal["q"])
        fs_s.write("E",self.stereo_cal["e"])
        fs_s.write("F",self.stereo_cal["f"])
        fs_s.write("rms_error",self.stereo_cal["error"])
        fs_s.release()
        # write rectification xmls
        left_rect_xml_file = output_folder + "/left_rectification.xml"
        right_rect_xml_file = output_folder + "/right_rectification.xml"
        fs_l = cv2.FileStorage(left_rect_xml_file, cv2.FILE_STORAGE_WRITE)
        fs_l.write("x",self.stereo_cal["left"]["map1"])
        fs_l.write("y",self.stereo_cal["left"]["map2"])
        fs_l.release()
        fs_r = cv2.FileStorage(right_rect_xml_file, cv2.FILE_STORAGE_WRITE)
        fs_r.write("x",self.stereo_cal["right"]["map1"])
        fs_r.write("y",self.stereo_cal["right"]["map2"])
        fs_r.release()

        # write yamls
        left_cal_yml_file = output_folder + "/left.yaml"
        right_cal_yml_file = output_folder + "/right.yaml"
        fs_l = cv2.FileStorage(left_cal_yml_file, cv2.FILE_STORAGE_WRITE)
        fs_l.write("image_width",self.stereo_cal["image size"][0])
        fs_l.write("image_height",self.stereo_cal["image size"][1])
        fs_l.write("camera_name","leftCamera")
        fs_l.write("camera_matrix",self.stereo_cal["left"]["m"])
        fs_l.write("distortion_model","plumb_bob")
        fs_l.write("distortion_coefficients",self.stereo_cal["left"]["d"])
        fs_l.write("rectification_matrix",self.stereo_cal["left"]["r"])
        fs_l.write("projection_matrix",self.stereo_cal["left"]["p"])
        # rms error for stereo reprojection as yaml doesn't have seperate stereo calibration file
        fs_l.write("rms_error",self.stereo_cal["error"]) 
        fs_l.release()
        fs_r = cv2.FileStorage(right_cal_yml_file, cv2.FILE_STORAGE_WRITE)
        fs_r.write("image_width",self.stereo_cal["image size"][0])
        fs_r.write("image_height",self.stereo_cal["image size"][1])
        fs_r.write("camera_name","rightCamera")
        fs_r.write("camera_matrix",self.stereo_cal["right"]["m"])
        fs_r.write("distortion_model","plumb_bob")
        fs_r.write("distortion_coefficients",self.stereo_cal["right"]["d"])
        fs_r.write("rectification_matrix",self.stereo_cal["right"]["r"])
        fs_r.write("projection_matrix",self.stereo_cal["right"]["p"])
        # rms error for stereo reprojection as yaml doesn't have seperate stereo calibration file
        fs_r.write("rms_error",self.stereo_cal["error"])
        fs_r.release()

        print("Calibration written to output folder.")

    def get_cal_from_yaml(self, left_cal_file, right_cal_file):
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
            return False, None
        fs_l.release()
        fs_r.release()

        q = self.calc_q(m_l,p_r,p_l)

        # missing data from calibration files (not possible to know if just using yaml)
        camera_l_cal = {'m':m_l, 'd':d_l, 'r':r_l, 'p':p_l, 'map1':None, 'map2':None, 'error':None}
        camera_r_cal = {'m':m_r, 'd':d_r, 'r':r_r, 'p':p_r, 'map1':None, 'map2':None, 'error':None}
        stereo_cal = {'image size':None, 'left':camera_l_cal, 'right':camera_r_cal, 'q':q, 'r':None, 't':None, 'e':None, 'f':None, 'error':None}

        self.stereo_cal = stereo_cal
        return res, stereo_cal

    def get_cal_from_xml(self, left_cal_file, right_cal_file, stereo_cal_file, right_rect_file, left_rect_file):
        res = False
        fs_l = cv2.FileStorage(left_cal_file, cv2.FILE_STORAGE_READ)
        fs_r = cv2.FileStorage(right_cal_file, cv2.FILE_STORAGE_READ)
        fs_s = cv2.FileStorage(stereo_cal_file, cv2.FILE_STORAGE_READ)
        fs_r_l = cv2.FileStorage(left_rect_file, cv2.FILE_STORAGE_READ)
        fs_r_r = cv2.FileStorage(right_rect_file, cv2.FILE_STORAGE_READ)
        if fs_l.isOpened() and fs_r.isOpened() and fs_s.isOpened() and fs_r_l.isOpened() and fs_r_r.isOpened():
            m_l = fs_l.getNode("cameraMatrix").mat()
            m_r = fs_r.getNode("cameraMatrix").mat()
            d_l = fs_l.getNode("distCoeffs").mat()
            d_r = fs_r.getNode("distCoeffs").mat()
            error_l = fs_l.getNode("rms_error").real()
            error_r = fs_r.getNode("rms_error").real()
            r_s = fs_s.getNode("R").mat()
            t_s = fs_s.getNode("T").mat()
            q_s = fs_s.getNode("Q").mat()
            e_s = fs_s.getNode("E").mat()
            f_s = fs_s.getNode("F").mat()
            error_s = fs_s.getNode("rms_error").real()
            mapL1 = fs_r_l.getNode("x").mat()
            mapL2 = fs_r_l.getNode("y").mat()
            mapR1 = fs_r_r.getNode("x").mat()
            mapR2 = fs_r_r.getNode("y").mat()
            res = True
        else:
            print("Failed to open calibration files")
            return False, None
        fs_l.release()
        fs_r.release()
        fs_s.release()
        fs_r_l.release()
        fs_r_r.release()

        # missing data from calibration files (not possible to know if just using xml)
        camera_l_cal = {'m':m_l, 'd':d_l, 'r':None, 'p':None, 'map1':mapL1, 'map2':mapL2, 'error':error_l}
        camera_r_cal = {'m':m_r, 'd':d_r, 'r':None, 'p':None, 'map1':mapR1, 'map2':mapR2, 'error':error_r}
        stereo_cal = {'image size':None, 'left':camera_l_cal, 'right':camera_r_cal, 'q':q_s, 'r':r_s, 't':t_s, 'e':e_s, 'f':f_s, 'error':error_s}

        self.stereo_cal = stereo_cal
        return res, stereo_cal

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

    def calibrate_camera(self, images, grid_rows, grid_cols, objp):
        objpoints = []  # 3d point in real world space
        imgpoints = []  # 2d points in image plane.

        for img in images:
            ret, corners = cv2.findChessboardCorners(img, (grid_cols, grid_rows), None)
            if ret:
                objpoints.append(objp)
                ret = cv2.cornerSubPix(img, corners, (11, 11),(-1, -1), self.criteria)
                imgpoints.append(corners)
        
        img_shape = images[0].shape[::-1]

        ret, m, d, r, p = cv2.calibrateCamera(
            objpoints, imgpoints, img_shape, None, None)

        camera_l_cal = {'imgpoints':imgpoints,'m':m, 'd':d, 'r':r, 'p':p}

        return objpoints, camera_l_cal

    def calibrate_cameras(self, images_l, images_r, grid_rows, grid_cols, objp):
        objpoints = []  # 3d point in real world space
        imgpoints_l = []  # 2d points in image plane.
        imgpoints_r = []  # 2d points in image plane.

        cv2.namedWindow(self.cv_window_name)

        for img_l, img_r in zip(images_l,images_r):
            ret_l, corners_l = cv2.findChessboardCorners(img_l, (grid_cols, grid_rows), None)
            ret_r, corners_r = cv2.findChessboardCorners(img_r, (grid_cols, grid_rows), None)
            if (ret_l and ret_r):
                objpoints.append(objp)
                corners_l = cv2.cornerSubPix(img_l, corners_l, (11, 11),(-1, -1), self.criteria)
                corners_r = cv2.cornerSubPix(img_r, corners_r, (11, 11),(-1, -1), self.criteria)
                imgpoints_l.append(corners_l)
                imgpoints_r.append(corners_r)

                ret_l = cv2.drawChessboardCorners(img_l, (grid_cols, grid_rows),
                                                  corners_l, ret_l)
                ret_r = cv2.drawChessboardCorners(img_r, (grid_cols, grid_rows),
                                                  corners_r, ret_r)

                image_left_resized = cv2.resize(img_l, (640,480))
                image_right_resized = cv2.resize(img_r, (640,480))

                left_right_dual = np.concatenate((image_left_resized, image_right_resized), axis=1)

                cv2.imshow(self.cv_window_name, left_right_dual)
                cv2.waitKey(1)

        cv2.destroyWindow(self.cv_window_name)

        img_shape = images_l[0].shape[::-1]

        ret_l, m_l, d_l, r_l, t_l = cv2.calibrateCamera(
            objpoints, imgpoints_l, img_shape, None, None)
        ret_r, m_r, d_r, r_r, t_r = cv2.calibrateCamera(
            objpoints, imgpoints_r, img_shape, None, None)

        camera_l_cal = {'imgpoints':imgpoints_l,'m':m_l, 'd':d_l, 'r':r_l, 't':t_l, 'error':ret_l}
        camera_r_cal = {'imgpoints':imgpoints_r,'m':m_r, 'd':d_r, 'r':r_r, 't':t_r, 'error':ret_r}

        return objpoints, camera_l_cal, camera_r_cal

    def calibrate(self,left_images_folder, right_images_folder, output_folder, left_wildcard="*_l", right_wildcard="*_r", grid_size=39.0, grid_rows=6, grid_cols=9):
        print("Calibrating stereo images...")

        self.grid_size = grid_size
        
        objp = np.zeros((grid_rows*grid_cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:grid_cols, 0:grid_rows].T.reshape(-1, 2)

        images_left = self.read_images(left_images_folder,left_wildcard)
        images_right = self.read_images(right_images_folder,right_wildcard)
        objpoints, camera_l_cal, camera_r_cal = self.calibrate_cameras(images_left,images_right,grid_rows,grid_cols,objp)

        img_shape = images_left[0].shape[::-1]

        ret, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
            objpoints, camera_l_cal["imgpoints"], camera_r_cal["imgpoints"], 
            camera_l_cal["m"], camera_l_cal["d"], camera_r_cal["m"], camera_r_cal["d"], 
            img_shape, criteria=self.stereocalib_criteria, flags=self.flags)

        R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(camera_l_cal["m"],camera_l_cal["d"],camera_r_cal["m"],camera_r_cal["d"],img_shape, R, T)

        stcamera_l_cal = {'m':camera_l_cal["m"], 'd':camera_l_cal["d"], 'r':R1, 'p':P1, 'map1':None, 'map2':None, 'error':camera_l_cal["error"]}
        stcamera_r_cal = {'m':camera_r_cal["m"], 'd':camera_r_cal["d"], 'r':R2, 'p':P2, 'map1':None, 'map2':None, 'error':camera_r_cal["error"]}
        
        mapL1, mapL2 = cv2.initUndistortRectifyMap(
                stcamera_l_cal["m"], stcamera_l_cal["d"], stcamera_l_cal["r"], stcamera_l_cal["p"], 
                img_shape, cv2.CV_32FC1)

        mapR1, mapR2 = cv2.initUndistortRectifyMap(
                stcamera_r_cal["m"], stcamera_r_cal["d"], stcamera_r_cal["r"], stcamera_r_cal["p"], 
                img_shape, cv2.CV_32FC1)

        stcamera_l_cal["map1"] = mapL1
        stcamera_l_cal["map2"] = mapL2
        stcamera_r_cal["map1"] = mapR1
        stcamera_r_cal["map2"] = mapR2

        stereo_cal = {'image size':img_shape, 'left':stcamera_l_cal, 'right':stcamera_r_cal, 'q':Q, 'r':R, 't':T, 'e':E, 'f':F, 'error':ret}
        
        self.stereo_cal = stereo_cal

        self.write_calibration(output_folder)

        print("Calibration complete")
        return stereo_cal

    def rectify_pair(self,image_left,image_right):
        image_l_rect = self.rectify(image_left,self.stereo_cal["left"])
        image_r_rect = self.rectify(image_right,self.stereo_cal["right"])
        return image_l_rect, image_r_rect

    def rectify(self,image,camera_cal):
        if (camera_cal["map1"] is None or camera_cal["map2"] is None):
            # create rectification maps if not read from calibration file
            map1, map2 = cv2.initUndistortRectifyMap(
                camera_cal["m"], camera_cal["d"], camera_cal["r"], camera_cal["p"], 
                image.shape[::-1], cv2.CV_32FC1)
            camera_cal["map1"] = map1
            camera_cal["map2"] = map2
        else:
            map1 = camera_cal["map1"]
            map2 = camera_cal["map2"]
        rect_image = cv2.remap(image, map1, map2, cv2.INTER_CUBIC, borderMode=cv2.BORDER_CONSTANT)
        return rect_image