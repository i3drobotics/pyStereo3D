#include <Stereo3D.h>

Stereo3D::Stereo3D(std::string stereo_camera, std::string stereo_calibration, std::string stereo_matcher)
{
    this->change_camera(stereo_camera);
    this->stereo_calibration = stereo_calibration;
    //Q = stereo_calibration.stereo_cal["q"] //TODO get this when correct class being used

    this->cv_window_name_Controls = "[Stereo3D] Controls";
    this->cv_window_name_Images = "[Stereo3D] Images";

    namedWindow(this->cv_window_name_Controls,WINDOW_NORMAL);
    namedWindow(this->cv_window_name_Images,WINDOW_NORMAL);

    setMouseCallback(this->cv_window_name_Images, this->on_window_mouse);

    resizeWindow(this->cv_window_name_Controls, 400,0 );

    this->change_matcher(stereo_matcher);

    this->ply_header = "ply\n"
        "format ascii 1.0\n"
        "element vertex %(vert_num)d\n"
        "property float x\n"
        "property float y\n"
        "property float z\n"
        "property uchar red\n"
        "property uchar green\n"
        "property uchar blue\n"
        "end_header\n";

    this->default_min_disp = 1000;
    this->default_num_disparities = 20;
    this->default_block_size = 12;
    this->default_uniqueness_ratio = 15;
    this->default_texture_threshold = 15;
    this->default_speckle_size = 0;
    this->default_speckle_range = 500;
}

void Stereo3D::change_camera(std::string stereo_camera){ //TODO change type
    this->stereo_camera = stereo_camera;
}

void Stereo3D::change_matcher(){
    std::string matcher_name = "BM";
    this->change_matcher(matcher_name);
    this->init_matcher();
}

void Stereo3D::change_matcher(std::string stereo_matcher_name){
    if (stereo_matcher_name == "BM"){
        this->matcher = StereoBM::create();
    } else if (stereo_matcher_name == "SGBM"){
       this->matcher = StereoSGBM::create();
    } else {
        return;
    }
    this->init_matcher();
}

void Stereo3D::change_matcher(Ptr<StereoMatcher> stereo_matcher){
    this->matcher = stereo_matcher;
    this->init_matcher();
}

void Stereo3D::init_matcher(){
    int calc_block = (2 * this->default_block_size + 5);
    this->matcher->setBlockSize(calc_block);
    this->matcher->setMinDisparity(int(this->default_min_disp - 1000));
    this->matcher->setNumDisparities(16*(this->default_num_disparities+1));
    //this->matcher->setUniquenessRatio(this->default_uniqueness_ratio);
    //this->matcher->setTextureThreshold(this->default_texture_threshold);
    this->matcher->setSpeckleWindowSize(this->default_speckle_size);
    this->matcher->setSpeckleRange(this->default_speckle_range);

    destroyWindow(this->cv_window_name_Controls);
    namedWindow(this->cv_window_name_Controls,WINDOW_NORMAL);

    createTrackbar("Min disp", this->cv_window_name_Controls , &this->default_min_disp, 2000, this->on_min_disparity_trackbar);
    createTrackbar("Disp", this->cv_window_name_Controls , &this->default_num_disparities, 30, this->on_num_disparities_trackbar);
    createTrackbar("Blck sze", this->cv_window_name_Controls , &this->default_block_size, 100, this->on_block_size_trackbar);

    createTrackbar("Uniq", this->cv_window_name_Controls , &this->default_uniqueness_ratio, 100, this->on_uniqueness_ratio_trackbar);
    createTrackbar("Texture", this->cv_window_name_Controls , &this->default_texture_threshold, 100, this->on_texture_threshold_trackbar);

    createTrackbar("Sp size", this->cv_window_name_Controls , &this->default_speckle_size, 30, this->on_speckle_size_trackbar);
    createTrackbar("Sp range", this->cv_window_name_Controls , &this->default_speckle_range, 1000, this->on_speckle_range_trackbar);
}

bool Stereo3D::connect(){
    /*
    TODO
    res = self.stereo_camera.connect()
    return res
    */
   return false;
}

Mat Stereo3D::gen3D(Mat left_image, Mat right_image){
    Mat disparity;
    this->matcher->compute(left_image,right_image,disparity);
    disparity.convertTo(disparity, CV_32F);
    Mat disparity = disparity / 16;
    return disparity;
}

Mat Stereo3D::genDepth(Mat disparity){
    Mat depth;
    reprojectImageTo3D(disparity, depth, this->Q);
    return depth;
}

void Stereo3D::write_ply(std::string filename, Mat disp, Mat depth, Mat image){
    
    //TODO 
    int row = (disp.rows-1)/2;
    int column = (disp.cols-1)/2;
    float disp_val_f =  disp.at<float>(row, column);
    //qDebug() << "IMAGE CENTER: " << column+1 << "," << row+1;
    //qDebug() << "DISPARITY AT CENTER: " << disp_val_f;
    
    cv::Mat image_rgb;
    cvtColor(image, image_rgb, COLOR_BGR2RGB);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr ptCloudTemp(
                new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::PointXYZRGB point;
    uint32_t rgb = 0;
    uchar col = 0;

    point.x = 0;
    point.y = 0;
    point.z = 0;

    rgb = ((int)255) << 16 | ((int)255) << 8 | ((int)255);
    point.rgb = *reinterpret_cast<float *>(&rgb);
    ptCloudTemp->points.push_back(point);

    for (int i = 0; i < depth.rows; i++) {
        float *reconst_ptr = depth.ptr<float>(i);
        float *disp_ptr = disp.ptr<float>(i);
        uchar *rgb_ptr = image_rgb.ptr<uchar>(i);

        // TODO: Figure out why this can trigger, assume the disparity map isn't generated
        // when this function gets called.
        if(!disp_ptr || !rgb_ptr || !reconst_ptr) return;

        for (int j = 0; j < depth.cols; j++) {
            if (disp_ptr[j] == 0) continue;
            if (rgb_ptr[j] == 0) continue;

            if (j == column){
                if (i == row){
                    float reproj_val_2 =  reconst_ptr[3 * j + 2];
                    //qDebug() << "REPROJECT3D AT CENTER 2: " << reproj_val_2;
                }
            }

            point.x = -reconst_ptr[3 * j];
            point.y = -reconst_ptr[3 * j + 1];
            point.z = reconst_ptr[3 * j + 2];

            if(abs(point.x) > 10) continue;
            if(abs(point.y) > 10) continue;
            if(point.z == 10000) continue;
            if(point.z == -10000) continue;
            col = rgb_ptr[j];

            rgb = ((int)col) << 16 | ((int)col) << 8 | ((int)col);
            point.rgb = *reinterpret_cast<float *>(&rgb);

            ptCloudTemp->points.push_back(point);
        }
    }

    if(ptCloudTemp->points.empty())
        return;

    //pcl::PassThrough<pcl::PointXYZRGB> pass;
    //pass.setInputCloud (ptCloudTemp);
    //pass.setFilterFieldName ("z");
    //pass.setFilterLimits(visualisation_min_z, visualisation_max_z);
    //pass.filter(*ptCloudTemp);



    /*
    Mat mask = disp > disp.min();
    Mat points = depth[mask];
    Mat image = image[mask];

    Mat points = points.reshape(-1, 3);
    Mat image = image.reshape(-1, 3);

    Mat points = np.hstack([points, image]);
    
    with open(filename, 'wb') as f:
        f.write((self.ply_header % dict(vert_num=len(points))).encode('utf-8'))
        np.savetxt(f, points, fmt='%f %f %f %d %d %d ')
    */
}

Mat Stereo3D::scale_disparity(Mat disparity){
    /*
    TODO
    minV, maxV,_,_ = cv2.minMaxLoc(disparity)
    if (maxV - minV != 0):
        scaled_disp = cv2.convertScaleAbs(disparity, alpha=255.0/(maxV - minV), beta=-minV * 255.0/(maxV - minV))
        return scaled_disp
    else:
        return np.zeros(disparity.shape, np.uint8)
    */
}

void Stereo3D::on_min_disparity_trackbar(int val){
    int min_disp = int(val - 1000);
    this->matcher->setMinDisparity(min_disp);
}

void Stereo3D::on_block_size_trackbar(int val){
    this->matcher->setBlockSize(2 * val + 5);
}

void Stereo3D::on_num_disparities_trackbar(int val){
    this->matcher->setNumDisparities(16*(val+1));
}

void Stereo3D::on_texture_threshold_trackbar(int val){
    //this->matcher->setTextureThreshold(val);
}

void Stereo3D::on_uniqueness_ratio_trackbar(int val){
    //this->matcher->setUniquenessRatio(val);
}

void Stereo3D::on_speckle_size_trackbar(int val){
    this->matcher->setSpeckleWindowSize(val);
}

void Stereo3D::on_speckle_range_trackbar(int val){
    this->matcher->setSpeckleRange(val);
}

void Stereo3D::on_window_mouse(int event, int x, int y, int flags, void* param){
   if (event == EVENT_LBUTTONDOWN){
       std::cout << "click" << std::endl;
   }
}

bool Stereo3D::grab3D(OutputArray output_disp, bool isrectified){
    /*
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
    */
}

void Stereo3D::save_point_cloud(Mat disparity, Mat image, std::string defaultSaveFolder="", std::string points_file_string="output.ply"){
    /*
    // prompt user for save location
    resp = prompt(text='Saving 3D Point Cloud to path: ', title='Save 3D Point Cloud' , default=defaultSaveFolder)
    if (resp is not None):
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
    */
}   

void Stereo3D::run()
{
    Mat image(500, 1000, CV_8UC3, Scalar(0,0, 100));

    namedWindow("Display Image", WINDOW_AUTOSIZE );
    imshow("Display Image", image);

    waitKey(0);

    /*
    # connect to stereo camera
    self.connect()
    save_index = 0
    while(True):
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
            break
        elif k == ord('s'): # save stereo image pair
            left_file_string=str(save_index)+"_l.png"
            right_file_string=str(save_index)+"_r.png"
            self.stereo_camera.save_images(self.image_left,self.image_right,defaultSaveFolder,left_file_string,right_file_string)
            save_index += 1
        elif k == ord('r'): # save rectified stereo image pair
            left_file_string="rect_"+str(save_index)+"_l.png"
            right_file_string="rect_"+str(save_index)+"_r.png"
            self.stereo_camera.save_images(self.rect_image_left,self.rect_image_right,defaultSaveFolder,left_file_string,right_file_string)
            save_index += 1
        elif k == ord('p'): # save 3D data as point cloud
            points_file_string = "points_"+str(save_index)+".ply"
            self.save_point_cloud(disp,self.rect_image_left,defaultSaveFolder,points_file_string)
            save_index += 1
        elif k == ord('1'): # change tp OpenCV BM
            self.change_matcher("BM")
        elif k == ord('2'): # change to OpenCV SGBM
            self.change_matcher("SGBM")
        if cv2.getWindowProperty(self.cv_window_name_Images,cv2.WND_PROP_VISIBLE) < 1:        
            break
        if cv2.getWindowProperty(self.cv_window_name_Controls,cv2.WND_PROP_VISIBLE) < 1:        
            break
        time.sleep(frame_delay)     
    */
}