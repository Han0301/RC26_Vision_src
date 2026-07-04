#include "./../rcekf/rcekf.h"
#include "./../serial.h"
#include "./../openvino.h"
#include "./../threadpool.h"
#include "./..//livox_ros_driver2/src/livox_ros_driver.h"
#include "./..//point_lio/src/laserMapping2.h"
#include "./../lidar.h"
#include "./../camera.h"
#include "./../usb_cam.h"
#include "./../usb_cam_multi.h"
#include "./../relocation.h"
#include "./../coordinate.h"
#include "./../recognition/camera_calibration.h"
#include "./../recognition/world_to_camera.h"
#include "./../velocity.h"
#include "./../calibration.h"
#include "./../log/logger.h"
#include <cctype>     // isdigit() 字符判断（必须包含，否则部分编译器报错）
#include <fstream>    // 文件读写
#include "./../filter.h"
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "./../rcekf/predicted.h"
#include "./../mapping/mapping.h"
#include "./../superstratum/controlR2.h"

void test_mapping()
{

    ros::Rate sl(20);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        ros::spinOnce();
        sl.sleep();
    }

}

void test_vision_cam()
{
    Ten::Ten_usb_cam& usbcam = Ten::Ten_usb_cam::GetInstance(Ten::_usb_device_num1_,640,480,30);
    ros::Rate sl(60);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        cv::Mat img = usbcam.camera_read();
        if(img.empty())
        {
            sl.sleep();
            continue;
        }
        cv::imshow("debug", img);
        cv::waitKey(1000 / 60);
    }

}

void test_vision_cam_multi()
{
    std::vector<int> idxs = {Ten::_usb_device_num1_, Ten::_usb_device_num2_, Ten::_usb_device_num3_};
    Ten::Ten_usb_cam_multi& usbcam = Ten::Ten_usb_cam_multi::GetInstance(idxs,640,480,30);

    ros::Rate sl(60 * idxs.size());
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        for(int idx : idxs)
        {
            cv::Mat img = usbcam.camera_read(idx);
            if(img.empty())
            {
                sl.sleep();
                continue;
            }
            if(idx == Ten::_usb_device_num1_)
            {
                cv::imshow("debug1", img);
                cv::waitKey(1000 / 60 / idxs.size());
            }
            else if(idx == Ten::_usb_device_num2_)
            {
                cv::imshow("debug2", img);
                cv::waitKey(1000 / 60 / idxs.size());
            }
            else if(idx == Ten::_usb_device_num3_)
            {
                cv::imshow("debug3", img);
                cv::waitKey(1000 / 60 / idxs.size());
            }
            else
            {
                sl.sleep();
            }
            
        }
        
    }

}


void test_vision_d435()
{
    urcu_memb_register_thread();
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance(640,480,60);
    ros::Rate sl(60);
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        cv::Mat img = camera.camera_read();
        if(img.empty())
        {
            sl.sleep();
            continue;
        }
        cv::imshow("debug", img);
        cv::waitKey(1000 / 60);
    }
}


void test_path()
{
    std::vector<int> path = Ten::readFileToAsciiVector(std::string(ROOT_DIR) + std::string("path/map.txt"));
    
    for(size_t i = 0; i < path.size(); i++)
    {
        std::cout << path[i] << " ";
    }
    std::cout << std::endl;
    
    for(size_t i = 0; i < Ten::_global_path_.size(); i++)
    {
        std::cout << Ten::_global_path_[i] << " ";
    }
    std::cout << std::endl;
}

