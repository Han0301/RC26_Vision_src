#ifndef __VISION_CPP_
#define __VISION_CPP_




#include "serial.h"
#include "openvino.h"
#include "threadpool.h"
#include "./livox_ros_driver2/src/livox_ros_driver.h"
#include "./point_lio/src/laserMapping2.h"
#include "lidar.h"
#include "camera.h"
#include "test.cpp"
#include "test2.cpp"
#include "relocation.h"
#include "coordinate.h"
#include "recognition/camera_calibration.h"
#include "recognition/world_to_camera.h"
#include "recognition/occlusion_handing.h"
#include "velocity.h"
#include "calibration.h"
#include "log/logger.h"
#include "yolo/yolo_v5.h"
#include "yolo/yolo_han.h"
#include "orb/orb_overall_exhaust.h"
#include "method_math.h"


void orb_test()
{
    urcu_memb_register_thread();
    //orb
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);

    // int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
    // cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
    // cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
    // cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
    // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
    // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);
    
    // cv::Mat img = cv::imread("/home/maple/study2/hou/image1/img1.jpg");
    // cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.36314436, -1.01831353,  0.96407981);
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.58769593,  0.78453107,  0.6021223);
    // cv::Mat img2 = cv::imread("/home/maple/study2/hou/image1/img2.jpg");
    // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.19823447, -1.19880829,  1.22625982);
    // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -2.99413314,  0.72745418, -2.86621733);

    cv::Mat img = cv::imread("/home/maple/study2/hou/image1/2.jpg");
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.15538523, -1.06445562,  1.18891875);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -3.1360852,   0.65997508, -1.54297095);
    cv::Mat img2 = cv::imread("/home/maple/study2/hou/image1/1.jpg");
    cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.39449508, -1.03322981,  0.93960988);
    cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -4.7881104,   0.77966552,  0.75880949);

    Ten::ORB::orb_optimize_exhaust ooe("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);



    std::vector<Ten::ORB::orb_exhaust_element> oees;
    Ten::ORB::orb_exhaust_element oee;
    oee.image_ = img;
    oee.rvec_ = rvec;
    oee.tvec_ = tvec;
    oees.push_back(oee);
    oee.image_ = img2;
    oee.rvec_ = rvec2;
    oee.tvec_ = tvec2;
    oees.push_back(oee);
    std::vector<int> place = ooe.getplace(oees);



    std::cout << "place: ";
    for(auto it : place)
    {
        std::cout << it << ",";
    }
    std::cout << std::endl;


    std::vector<double> losses = ooe.get_loss();
    for(size_t i = 0; i < losses.size(); i++)
    {
        std::cout << "loss[" << i << "]: " << losses[i] << std::endl;
    }

    std::vector<Ten::ORB::orb_exhaust_element> oprt = ooe.get_RT(oees);
    log.record_imageRT(oees);
    log.record_imageRT(oprt);

    //检验
    int exist_box[12] = {0};
    for(size_t k = 0; k < 12 && k < place.size(); k++)
    {
        exist_box[k] = place[k];
    }

    {
        //zubbfer
        Ten::init_3d_box world_point;
        Ten::Ten_occlusion_handing zbuffer;
        for(size_t i = 0; i < oees.size(); i++)
        {
            Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_RT(oees[i].rvec_, oees[i].tvec_);
            Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
            world_point.pcl_to_C();
            //zbuffer.set_exist_boxes(exist_box);
            //zbuffer.set_interested_boxes(exist_box);
            zbuffer.set_box_lists_(oees[i].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
            cv::Mat debug1  = cv::Mat::zeros(480, 640, CV_8UC3); 
            zbuffer.set_debug_roi_image(world_point.box_lists_, debug1);
            cv::imshow("debug1", debug1);
            cv::waitKey(0);
        }
        log.record_image(world_point.box_lists_);
    }

    {
        //zubbfer
        Ten::init_3d_box world_point;
        Ten::Ten_occlusion_handing zbuffer;
        for(size_t i = 0; i < oprt.size(); i++)
        {
            Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_RT(oprt[i].rvec_, oprt[i].tvec_);
            Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
            world_point.pcl_to_C();
            zbuffer.set_exist_boxes(exist_box);
            zbuffer.set_box_lists_(oprt[i].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
            cv::Mat debug2  = cv::Mat::zeros(480, 640, CV_8UC3); 
            zbuffer.set_debug_roi_image(world_point.box_lists_, debug2);
            cv::imshow("debug2", debug2);
            cv::waitKey(0); 
        }
        log.record_image(world_point.box_lists_);
    }
    urcu_memb_unregister_thread();
}


void test_calibration_lidar_camera()
{
    urcu_memb_register_thread();
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
    int flag = 0;
    std::vector<Ten::ORB::orb_exhaust_element> imageRTs;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        Ten::ORB::orb_exhaust_element imageRT;
        cv::Mat image = camera.camera_read();
        cv::imshow("debug", image);
        flag = cv::waitKey(0);
        if(flag == 's')
        {
            cv::Mat image = camera.camera_read();
            imageRT.image_ = image;
            imageRTs.push_back(imageRT);
            log.record_odometry(Ten::_TF_GET_.read_data());
        }
        else if(flag == 'x')
        {
            break;
        }

    }
    log.record_imageRT(imageRTs);
    urcu_memb_unregister_thread();
}
void vision_test1()
{
    urcu_memb_register_thread();
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
    log.record_odometry(nav_msgs::Odometry());
    log.record_odometry(nav_msgs::Odometry());
    log.record_odometry(nav_msgs::Odometry());
    urcu_memb_unregister_thread();
}

void vision_test2()
{
    urcu_memb_register_thread();
    std::vector<Ten::Ten_camerainfo> vc = Ten::readCameraInfosFromDir("/home/maple/study2/merge_ws32/src/merge/log/2026_3_9/11/imageRT/imagert2");
    for(size_t i = 0; i < vc.size(); i++)
    {
        std::cout << i << std::endl;
        std::cout << vc[i].rvec() << std::endl;
        std::cout << vc[i].tvec() << std::endl;
        std::cout << vc[i].extrinsic() << std::endl;
    }
    std::vector<Ten::XYZRPY> vpose = Ten::readPoseFromTxt("/home/maple/study2/merge_ws32/src/merge/log/2026_3_11/17/odometry.txt");
    std::cout << "vpose.size(): " << vpose.size() << std::endl;
    for(auto tf : vpose)
    {
        std::cout << "---------------------------" << std::endl; 
        std::cout << "x: " << tf._xyz._x << std::endl;
        std::cout << "y: " << tf._xyz._y << std::endl;
        std::cout << "z: " << tf._xyz._z << std::endl;
        std::cout << "roll: " << tf._rpy._roll << std::endl;
        std::cout << "pitch: " << tf._rpy._pitch << std::endl;
        std::cout << "yaw: " << tf._rpy._yaw << std::endl;
    }
    urcu_memb_unregister_thread();
}

#endif


