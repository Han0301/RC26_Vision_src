#ifndef __USB_CAM_H_
#define __USB_CAM_H_
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <unistd.h>
#include "threadpool.h"
#include "camera_virtual.h"

namespace Ten
{

class Ten_usb_cam : public camera_virtual
{
public:
    //禁用拷贝构造
    Ten_usb_cam(const Ten_usb_cam&) = delete;
    //禁用赋值
    Ten_usb_cam& operator=(const Ten_usb_cam&) = delete;

    static Ten_usb_cam& GetInstance(int dev_idx = 0, size_t w = 640, size_t h = 480, size_t fps = 30);

    cv::Mat camera_read();

    void reset_camera(int dev_idx, size_t w, size_t h, size_t fps);

    ~Ten_usb_cam()
    {
        if(_cap.isOpened())
            _cap.release();
    }
private:
    bool reset_camera_once();

    Ten_usb_cam(int dev_idx, size_t w, size_t h, size_t fps);

    static std::unique_ptr<Ten_usb_cam> create(int dev_idx = 0, size_t w = 640, size_t h = 480, size_t fps = 30);

    cv::VideoCapture _cap;
    std::mutex read_mtx_;
    int _dev_idx = 0;
    size_t _w = 0;
    size_t _h = 0;
    size_t _fps = 0;
    static std::once_flag camera_flag_;
};

}

#endif

