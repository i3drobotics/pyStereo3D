#ifndef STEREO3D_H
#define STEREO3D_H

#include <stdio.h>
//#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/stereo.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
//#include <pcl/visualization/pcl_visualizer.h>
//#include <StereoCapture/StereoCapture.h>
//#include <StereoCalibration/StereoCalibration.h>

using namespace cv;

class Stereo3D {
    public:
        std::string stereo_calibration; //TODO: change type
        std::string stereo_camera; //TODO: change type

        Ptr<StereoMatcher> matcher;

        Mat Q;

        Mat image_left;
        Mat image_right;
        Mat rect_image_left;
        Mat rect_image_right;
        Mat disparity;
        Mat depth;

        std::string cv_window_name_Controls;
        std::string cv_window_name_Images;

        std::string ply_header;

        int default_min_disp;
        int default_num_disparities;
        int default_block_size;
        int default_uniqueness_ratio;
        int default_texture_threshold;
        int default_speckle_size;
        int default_speckle_range;

        Stereo3D(std::string stereo_camera, std::string stereo_calibration, std::string stereo_matcher); //TODO change type

        void change_camera(std::string stereo_camera); //TODO change type

        void change_matcher();
        void change_matcher(std::string stereo_matcher_name);
        void change_matcher(Ptr<StereoMatcher> stereo_matcher);

        void init_matcher();

        bool connect();

        Mat gen3D(Mat left_image, Mat right_image);

        Mat genDepth(Mat disparity);

        void write_ply(std::string filename, Mat disp, Mat depth, Mat image);

        Mat scale_disparity(Mat disparity);

        void on_min_disparity_trackbar(int val);

        void on_block_size_trackbar(int val);

        void on_num_disparities_trackbar(int val);

        void on_texture_threshold_trackbar(int val);

        void on_uniqueness_ratio_trackbar(int val);

        void on_speckle_size_trackbar(int val);

        void on_speckle_range_trackbar(int val);

        void on_window_mouse(int event, int x, int y, int flags, void* param);

        bool grab3D(OutputArray output_disp, bool isrectified=false);

        void save_point_cloud(Mat disparity, Mat image, std::string defaultSaveFolder="", std::string points_file_string="output.ply");

        void run();
};

#endif  // STEREO3D_H