#ifndef __CAMERA2_H_
#define __CAMERA2_H_
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <unistd.h>
#include <vector>
#include "threadpool.h"
#include "camera.h"

namespace Ten
{


// 类名修改为 Ten_camera2
class Ten_camera2
{
public:
    Ten_camera2(const Ten_camera2&) = delete;
    Ten_camera2& operator=(const Ten_camera2&) = delete;

    // 构造函数
    Ten_camera2(const std::string& serial, size_t w = 640, size_t h = 480, size_t fps = 30);

    // 原有接口
    cv::Mat camera_read();
    camera_frame camera_read_depth();
    void reset_camera(size_t w, size_t h, size_t fps);
    void reset_camera_depth(size_t w, size_t h, size_t fps);
    rs2_intrinsics get_color_intrinsics();

    static std::vector<std::string> get_all_camera_serials();

    ~Ten_camera2()
    {
        try { pipe.stop(); } catch (...) {}
    }

private:
    bool reset_camera_once(size_t w, size_t h, size_t fps);
    bool reset_camera_depth_once(size_t w, size_t h, size_t fps);

    std::string serial_;
    rs2::pipeline pipe;
    rs2::config config;
    rs2_intrinsics color_intr_;
    std::mutex read_mtx_;
    size_t _w = 0, _h = 0, _fps = 0;
    rs2::align align_to_color_{RS2_STREAM_COLOR};
};

}

#endif
