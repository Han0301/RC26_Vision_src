#ifndef __CAMERA_CPP_
#define __CAMERA_CPP_
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <unistd.h>
#include "camera.h"

namespace Ten
{

// class Ten_camera
// {
// public:
//     //禁用拷贝构造
//     Ten_camera(const Ten_camera& serial) = delete;
//     //禁用赋值
//     Ten_camera& operator=(const Ten_camera& serial) = delete;
//     /**
//         @brief 设置分辨率和帧率
//         @param w: 1920 640
//         @param h: 1080 480
//         @param fps: 帧率
//         @return Ten_camera& 返回Ten_camera实例
//     */
//     static Ten_camera& GetInstance(size_t w = 1920, size_t h = 1080, size_t fps = 30)
//     {
//         static std::unique_ptr<Ten_camera> ten_camera = nullptr;
//         std::call_once(camera_flag_, [w, h, fps]() 
//         {
//             ten_camera = create(w, h, fps);
//             std::cout << "init_camera" << std::endl;
//         });
//         return *ten_camera;
//     } 

//     /** 
//         @brief 读取图片 
//         @return cv::Mat 
//     */
//     cv::Mat camera_read()
//     {
//         std::lock_guard<std::mutex> lock(read_mtx_);
//         // 等待并获取帧数据
//         rs2::frameset frames = pipe.wait_for_frames();
//         // 获取彩色帧
//         rs2::frame color_frame = frames.get_color_frame();
//         // 转换为OpenCV矩阵格式
//         cv::Mat color_image(cv::Size(_w, _h), CV_8UC3, 
//                            (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
//         return color_image;
//     }

//     /**
//         @brief 高效读取图片
//         @param int: 无实际意义，用于函数重载
//         @return cv::Mat* 外面要delete 对象
//     */
//     cv::Mat* camera_read(int)
//     {
//         std::lock_guard<std::mutex> lock(read_mtx_);
//         // 等待并获取帧数据
//         rs2::frameset frames = pipe.wait_for_frames();
//         // 获取彩色帧
//         rs2::frame color_frame = frames.get_color_frame();
//         // 转换为OpenCV矩阵格式
//         cv::Mat* color_image = new cv::Mat(
//             cv::Size(_w, _h), 
//             CV_8UC3, 
//             (void*)color_frame.get_data(), 
//             cv::Mat::AUTO_STEP
//         );
//         return color_image;
//     }


//     ~Ten_camera()
//     {
//         pipe.stop();
//     }
// private:
//     Ten_camera(size_t w, size_t h, size_t fps)
//     {
//         config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
//         pipe.start(config);
//         _w = w;
//         _h = h;
//     }

//     static std::unique_ptr<Ten_camera> create(size_t w = 1920, size_t h = 1080, size_t fps = 30) {
//         // 静态函数可访问私有构造函数，直接new对象后封装为unique_ptr
//         return std::unique_ptr<Ten_camera>(new Ten_camera(w, h, fps));
//     }

// rs2::pipeline pipe;
// rs2::config config;
// std::mutex read_mtx_;
// size_t _w = 0;
// size_t _h = 0;
// static std::once_flag camera_flag_;
// };
// std::once_flag Ten_camera::camera_flag_;

    std::once_flag Ten_camera::camera_flag_;

    Ten_camera& Ten_camera::GetInstance(size_t w, size_t h, size_t fps)
    {
        static std::unique_ptr<Ten_camera> ten_camera = nullptr;
        std::call_once(camera_flag_, [w, h, fps]() 
        {
            ten_camera = create(w, h, fps);
            std::cout << "init_camera" << std::endl;
        });
        return *ten_camera;
    } 

    cv::Mat Ten_camera::camera_read()
    {
        std::lock_guard<std::mutex> lock(read_mtx_);
        // 等待并获取帧数据
        rs2::frameset frames = pipe.wait_for_frames();
        // 获取彩色帧
        rs2::frame color_frame = frames.get_color_frame();
        // 转换为OpenCV矩阵格式
        cv::Mat color_image(cv::Size(_w, _h), CV_8UC3, 
                           (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        return color_image;
    }

    cv::Mat* Ten_camera::camera_read(int)
    {
        std::lock_guard<std::mutex> lock(read_mtx_);
        // 等待并获取帧数据
        rs2::frameset frames = pipe.wait_for_frames();
        // 获取彩色帧
        rs2::frame color_frame = frames.get_color_frame();
        // 转换为OpenCV矩阵格式
        cv::Mat* color_image = new cv::Mat(
            cv::Size(_w, _h), 
            CV_8UC3, 
            (void*)color_frame.get_data(), 
            cv::Mat::AUTO_STEP
        );
        return color_image;
    }


}








#endif


