#ifndef __CAMERA_H_
#define __CAMERA_H_
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <unistd.h>

namespace Ten
{

class Ten_camera
{
public:
    //禁用拷贝构造
    Ten_camera(const Ten_camera& serial) = delete;
    //禁用赋值
    Ten_camera& operator=(const Ten_camera& serial) = delete;
    /**
        @brief 设置分辨率和帧率
        @param w: 1920 640
        @param h: 1080 480
        @param fps: 帧率
        @return Ten_camera& 返回Ten_camera实例
    */
    static Ten_camera& GetInstance(size_t w = 1920, size_t h = 1080, size_t fps = 30);

    /** 
        @brief 读取图片 
        @return cv::Mat 
    */
    cv::Mat camera_read();

    /**
        @brief 高效读取图片
        @param int: 无实际意义，用于函数重载
        @return cv::Mat* 外面要delete 对象
    */

    cv::Mat* camera_read(int);

    ~Ten_camera()
    {
        pipe.stop();
    }
private:
    Ten_camera(size_t w, size_t h, size_t fps)
    {
        config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
        pipe.start(config);
        _w = w;
        _h = h;
    }

    static std::unique_ptr<Ten_camera> create(size_t w = 1920, size_t h = 1080, size_t fps = 30) {
        // 静态函数可访问私有构造函数，直接new对象后封装为unique_ptr
        return std::unique_ptr<Ten_camera>(new Ten_camera(w, h, fps));
    }

rs2::pipeline pipe;
rs2::config config;
std::mutex read_mtx_;
size_t _w = 0;
size_t _h = 0;
static std::once_flag camera_flag_;
};





}








#endif


