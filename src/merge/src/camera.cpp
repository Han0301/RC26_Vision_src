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
        try
        {
            // 等待并获取帧数据
            rs2::frameset frames = pipe.wait_for_frames();
            // 获取彩色帧
            rs2::frame color_frame = frames.get_color_frame();
            // 转换为OpenCV矩阵格式
            cv::Mat color_image(cv::Size(_w, _h), CV_8UC3, 
            (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            return color_image;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            reset_camera_once(_w, _h, _fps);
            return cv::Mat();
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            reset_camera_once(_w, _h, _fps);
            return cv::Mat();
        }
        return cv::Mat();
    }

    camera_frame Ten_camera::camera_read_depth()
    {
        std::lock_guard<std::mutex> lock(read_mtx_);
        
        try
        {
            camera_frame frame;
            // 等待并获取帧数据
            rs2::frameset frames = pipe.wait_for_frames();

            rs2::frameset aligned_frames = align_to_color_.process(frames);

            // 获取彩色帧
            rs2::frame color_frame = aligned_frames.get_color_frame();
            // 转换为OpenCV矩阵格式
            frame.bgr_image = cv::Mat (cv::Size(_w, _h), CV_8UC3, 
                            (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);

            // 获取深度帧
            rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
            frame.depth_image = cv::Mat(cv::Size(_w, _h), CV_16UC1, 
                        (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
            // 保存原生深度帧
            frame.raw_depth_frame = std::make_shared<rs2::depth_frame>(depth_frame);

            // int center_x = _w / 2;
            // int center_y = _h / 2;
            // 直接从realsense接口获取米级深度（比opencv转算更准）
            //float z_distance = depth_frame.get_distance(center_x, center_y);
            //std::cout << "✅ 相机坐标系Z轴（中心深度）= " << z_distance << " 米" << std::endl;
            return frame;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            reset_camera_depth_once(_w, _h, _fps);
            return camera_frame();
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            reset_camera_depth_once(_w, _h, _fps);
            return camera_frame();
        }
        return camera_frame();
    }


    void Ten_camera::reset_camera(size_t w, size_t h, size_t fps)
    {
        _w = w;
        _h = h;
        _fps = fps;
        try
        {
            std::lock_guard<std::mutex> lock(read_mtx_);
        
            // 1. 停止当前管道
            pipe.stop();
            
            // 2. 重置配置对象 (创建新的 config 以清除旧设置)
            config = rs2::config();
            
            // 3. 使能新的流配置
            config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
            
            // 4. 重新启动管道
            pipe.start(config);
            
            std::cout << "camera reset to " << w << "x" << h << "@" << fps << std::endl;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            while(!reset_camera_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag())
            {
                usleep(100000);
            }
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            std::cerr << "相机接受：未知致命异常" << std::endl;
            while(!reset_camera_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag())
            {
                usleep(100000);
            }
        }
    }

    void Ten_camera::reset_camera_depth(size_t w, size_t h, size_t fps)
    {
        _w = w;
        _h = h;
        _fps = fps;
        try
        {
            std::lock_guard<std::mutex> lock(read_mtx_);
        
            // 1. 停止当前管道
            pipe.stop();
            
            // 2. 重置配置对象 (创建新的 config 以清除旧设置)
            config = rs2::config();
            
            // 3. 使能新的流配置
            config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);    // RGB
            config.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);     // 深度图（16位）
            
            // 4. 重新启动管道
            auto active_profile = pipe.start(config);
            // ====================== 新增：减少深度噪声的配置 ======================
            auto depth_sensor = active_profile.get_device().first<rs2::depth_sensor>();
        
            // 1. 开启官方「高精度模式」（D400全系支持，最关键！）
            if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET))
            {
                // RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY = 高精准模式（减少斜视角噪声）
                depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 3.0f); 
            }
            // 2. 关闭自动曝光（固定曝光，斜视角深度更稳定）
            if (depth_sensor.supports(RS2_OPTION_AUTO_EXPOSURE_MODE))
            {
                depth_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_MODE, 0.0f);
                depth_sensor.set_option(RS2_OPTION_EXPOSURE, 7500.0f); 
            }
            // ====================================================================
    
            auto color_profile = active_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            color_intr_ = color_profile.get_intrinsics();                // 彩色内参
    
            std::cout << "camera reset to " << w << "x" << h << "@" << fps << std::endl;

        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            while(!reset_camera_depth_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag())
            {
                usleep(100000);
            }
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            std::cerr << "相机接受：未知致命异常" << std::endl;
            while(!reset_camera_depth_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag())
            {
                usleep(100000);
            }
        }
    }

    // 获取彩色相机内参
    rs2_intrinsics Ten_camera::get_color_intrinsics() {
        return color_intr_;
    }


    // cv::Mat* Ten_camera::camera_read(int)
    // {
    //     std::lock_guard<std::mutex> lock(read_mtx_);
    //     // 等待并获取帧数据
    //     rs2::frameset frames = pipe.wait_for_frames();
    //     // 获取彩色帧
    //     rs2::frame color_frame = frames.get_color_frame();
    //     // 转换为OpenCV矩阵格式
    //     cv::Mat* color_image = new cv::Mat(
    //         cv::Size(_w, _h), 
    //         CV_8UC3, 
    //         (void*)color_frame.get_data(), 
    //         cv::Mat::AUTO_STEP
    //     );
    //     return color_image;
    // }


    bool Ten_camera::reset_camera_once(size_t w, size_t h, size_t fps)
    {
        try
        {
            // 1. 停止当前管道
            //pipe.stop();
            
            // 2. 重置配置对象 (创建新的 config 以清除旧设置)
            config = rs2::config();
            
            // 3. 使能新的流配置
            config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
            
            // 4. 重新启动管道
            pipe.start(config);
            
            std::cout << "camera reset to " << w << "x" << h << "@" << fps << std::endl;
            return true;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            return false;
        }
    }

    bool Ten_camera::reset_camera_depth_once(size_t w, size_t h, size_t fps)
    {
        try
        {     
            // 1. 停止当前管道
            //pipe.stop();
            
            // 2. 重置配置对象 (创建新的 config 以清除旧设置)
            config = rs2::config();
            
            // 3. 使能新的流配置
            config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);    // RGB
            config.enable_stream(RS2_STREAM_DEPTH, w, h, RS2_FORMAT_Z16, fps);     // 深度图（16位）
            
            // 4. 重新启动管道
            auto active_profile = pipe.start(config);
            _w = w;
            _h = h;
            // ====================== 新增：减少深度噪声的配置 ======================
            auto depth_sensor = active_profile.get_device().first<rs2::depth_sensor>();
        
            // 1. 开启官方「高精度模式」（D400全系支持，最关键！）
            if (depth_sensor.supports(RS2_OPTION_VISUAL_PRESET))
            {
                // RS2_RS400_VISUAL_PRESET_HIGH_ACCURACY = 高精准模式（减少斜视角噪声）
                depth_sensor.set_option(RS2_OPTION_VISUAL_PRESET, 3.0f); 
            }
            // 2. 关闭自动曝光（固定曝光，斜视角深度更稳定）
            if (depth_sensor.supports(RS2_OPTION_AUTO_EXPOSURE_MODE))
            {
                depth_sensor.set_option(RS2_OPTION_AUTO_EXPOSURE_MODE, 0.0f);
                depth_sensor.set_option(RS2_OPTION_EXPOSURE, 7500.0f); 
            }
            // ====================================================================
    
            auto color_profile = active_profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
            color_intr_ = color_profile.get_intrinsics();                // 彩色内参
    
            std::cout << "camera reset to " << w << "x" << h << "@" << fps << std::endl;
            return true;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            return false;
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            return false;
        }
    }


}








#endif


