#ifndef __USB_CAM_CPP_
#define __USB_CAM_CPP_
#include "usb_cam.h"

namespace Ten
{

std::once_flag Ten_usb_cam::camera_flag_;

Ten_usb_cam& Ten_usb_cam::GetInstance(int dev_idx, size_t w, size_t h, size_t fps)
{
    static std::unique_ptr<Ten_usb_cam> usb_cam = nullptr;
    std::call_once(camera_flag_, [dev_idx, w, h, fps]() 
    {
        usb_cam = create(dev_idx, w, h, fps);
        std::cout << "init_usb_cam" << std::endl;
    });
    return *usb_cam;
} 

std::unique_ptr<Ten_usb_cam> Ten_usb_cam::create(int dev_idx, size_t w, size_t h, size_t fps) {
    return std::unique_ptr<Ten_usb_cam>(new Ten_usb_cam(dev_idx, w, h, fps));
}

Ten_usb_cam::Ten_usb_cam(int dev_idx, size_t w, size_t h, size_t fps)
{
    try
    {
        _dev_idx = dev_idx;
        _w = w;
        _h = h;
        _fps = fps;
        if(!reset_camera_once())
        {
            throw std::runtime_error("USB相机打开失败");
        }
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        while(!reset_camera_once() && Ten::_TREADPOOL_FLAG_.read_flag())
        {
            usleep(100000);
        }
    }
    catch (...)
    {
        std::cerr << "USB相机：未知致命异常" << std::endl;
        while(!reset_camera_once() && Ten::_TREADPOOL_FLAG_.read_flag())
        {
            usleep(100000);
        }
    }
}

cv::Mat Ten_usb_cam::camera_read()
{
    std::lock_guard<std::mutex> lock(read_mtx_);
    try
    {
        cv::Mat frame;
        if(!_cap.read(frame))
        {
            throw std::runtime_error("读取帧失败");
        }
        return frame; 
    }
    catch (...)
    {
        reset_camera_once();
        return cv::Mat();
    }
}

void Ten_usb_cam::reset_camera(int dev_idx, size_t w, size_t h, size_t fps)
{
    _dev_idx = dev_idx;
    _w = w;
    _h = h;
    _fps = fps;
    try
    {
        std::lock_guard<std::mutex> lock(read_mtx_);
        reset_camera_once();
        //ROS_INFO("USB相机重置成功");
    }
    catch (...)
    {
        while (!reset_camera_once() && Ten::_TREADPOOL_FLAG_.read_flag())
            usleep(100000);
    }
}

// ====================== 核心修复：强制使用V4L2后端 ======================
bool Ten_usb_cam::reset_camera_once()
{
    try
    {
        if(_cap.isOpened())
            _cap.release();
        
        // 强制使用Linux V4L2后端，彻底关闭GStreamer
        if(!_cap.open(_dev_idx, cv::CAP_V4L2))
        {
            return false;
        }

        // 设置相机参数
        _cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        _cap.set(cv::CAP_PROP_FRAME_WIDTH, _w);
        _cap.set(cv::CAP_PROP_FRAME_HEIGHT, _h);
        _cap.set(cv::CAP_PROP_FPS, _fps);
        _cap.set(cv::CAP_PROP_BUFFERSIZE, 3);

        
        return true;
    }
    catch(...)
    {
        return false;
    }
}

}

#endif
