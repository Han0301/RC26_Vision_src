#ifndef __CAMERA2_CPP_
#define __CAMERA2_CPP_
#include "camera2.h"

namespace Ten
{

// 获取所有相机序列号
std::vector<std::string> Ten_camera2::get_all_camera_serials()
{
    std::vector<std::string> serials;
    rs2::context ctx;
    for (auto&& dev : ctx.query_devices())
    {
        // 核心修复：旧SDK兼容写法
        const char* serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (serial) serials.push_back(serial);
    }
    return serials;
}

// 构造函数
Ten_camera2::Ten_camera2(const std::string& serial, size_t w, size_t h, size_t fps)
    : serial_(serial)
{
    _w = w;
    _h = h;
    _fps = fps;

    try
    {
        config.enable_device(serial_);
        config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
        pipe.start(config);
        ROS_INFO("相机 [%s] 初始化成功", serial_.c_str());
    }
    catch (const std::exception& e)
    {
        std::cerr << "相机 [" << serial_ << "] 初始化失败: " << e.what() << '\n';
        while (!reset_camera_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag())
        {
            usleep(100000);
        }
    }
    catch (...)
    {
        std::cerr << "相机 [" << serial_ << "] 未知异常\n";
        while (!reset_camera_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag())
        {
            usleep(100000);
        }
    }
}

// 读取RGB图像
cv::Mat Ten_camera2::camera_read()
{
    std::lock_guard<std::mutex> lock(read_mtx_);
    try
    {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frame color_frame = frames.get_color_frame();
        cv::Mat color_image(cv::Size(_w, _h), CV_8UC3,
                           (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        return color_image;
    }
    catch (...)
    {
        reset_camera_once(_w, _h, _fps);
        return cv::Mat();
    }
}

// 读取RGB+深度图像
camera_frame Ten_camera2::camera_read_depth()
{
    std::lock_guard<std::mutex> lock(read_mtx_);
    try
    {
        camera_frame frame;
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::frameset aligned_frames = align_to_color_.process(frames);

        rs2::frame color_frame = aligned_frames.get_color_frame();
        frame.bgr_image = cv::Mat(cv::Size(_w, _h), CV_8UC3,
                                 (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

        rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
        frame.depth_image = cv::Mat(cv::Size(_w, _h), CV_16UC1,
                                   (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        frame.raw_depth_frame = std::make_shared<rs2::depth_frame>(depth_frame);

        return frame;
    }
    catch (...)
    {
        reset_camera_depth_once(_w, _h, _fps);
        return camera_frame();
    }
}

// 获取相机内参
rs2_intrinsics Ten_camera2::get_color_intrinsics()
{
    return color_intr_;
}

// 重置RGB模式
void Ten_camera2::reset_camera(size_t w, size_t h, size_t fps)
{
    _w = w; _h = h; _fps = fps;
    try
    {
        std::lock_guard<std::mutex> lock(read_mtx_);
        pipe.stop();
        config = rs2::config();
        config.enable_device(serial_);
        config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
        pipe.start(config);
        ROS_INFO("相机 [%s] 重置成功", serial_.c_str());
    }
    catch (...)
    {
        while (!reset_camera_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag())
            usleep(100000);
    }
}

// 重置RGB+深度模式
void Ten_camera2::reset_camera_depth(size_t w, size_t h, size_t fps)
{
    _w = w; _h = h; _fps = fps;
    try
    {
        std::lock_guard<std::mutex> lock(read_mtx_);
        pipe.stop();
        config = rs2::config();
        config.enable_device(serial_);
        config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
        config.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);

        auto profile = pipe.start(config);
        auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
        if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET))
            depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 3.0f);
        if (depth_sensor.supports(RS2_OPTION_AUTO_EXPOSURE_MODE))
        {
            depth_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_MODE, 0.0f);
            depth_sensor.set_option(RS2_OPTION_EXPOSURE, 7500.0f);
        }

        auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        color_intr_ = color_profile.get_intrinsics();
        ROS_INFO("相机 [%s] 深度模式重置成功", serial_.c_str());
    }
    catch (...)
    {
        while (!reset_camera_depth_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag())
            usleep(100000);
    }
}

// 单次重置RGB
bool Ten_camera2::reset_camera_once(size_t w, size_t h, size_t fps)
{
    try
    {
        config = rs2::config();
        config.enable_device(serial_);
        config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
        pipe.start(config);
        return true;
    } catch (...) { return false; }
}

// 单次重置RGB+深度
bool Ten_camera2::reset_camera_depth_once(size_t w, size_t h, size_t fps)
{
    try
    {
        config = rs2::config();
        config.enable_device(serial_);
        config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
        config.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);
        auto profile = pipe.start(config);
        _w = w; _h = h;

        auto depth_sensor = profile.get_device().first<rs2::depth_sensor>();
        if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET))
            depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 3.0f);
        if (depth_sensor.supports(RS2_OPTION_AUTO_EXPOSURE_MODE))
        {
            depth_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_MODE, 0.0f);
            depth_sensor.set_option(RS2_OPTION_EXPOSURE, 7500.0f);
        }

        auto color_profile = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
        color_intr_ = color_profile.get_intrinsics();
        return true;
    } catch (...) { return false; }
}

}

#endif
