#ifndef __CAMERA_H_
#define __CAMERA_H_
#include <librealsense2/rs.hpp>
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

struct camera_frame
{
    cv::Mat bgr_image;
    cv::Mat depth_image;
    std::shared_ptr<rs2::depth_frame> raw_depth_frame;
};
    


class Ten_camera : public camera_virtual
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
        @brief 读取bgr和depth图片
        @return 结构体 camera_frame
    */
    camera_frame camera_read_depth();

    /**
        @brief 设置分辨率和帧率
        @param w: 1920 640
        @param h: 1080 480
        @param fps: 帧率
    */
    void reset_camera(size_t w, size_t h, size_t fps);

    // /**
    //     @brief 高效读取图片
    //     @param int: 无实际意义，用于函数重载
    //     @return cv::Mat* 外面要delete 对象
    // */

    // cv::Mat* camera_read(int);
    /**
        @brief 设置分辨率和帧率（启用深度和bgr双加载）
        @param w: 1920 640
        @param h: 1080 480
        @param fps: 帧率
    */
    void reset_camera_depth(size_t w, size_t h, size_t fps);

    rs2_intrinsics get_color_intrinsics();

    ~Ten_camera()
    {
        try
        {
            pipe.stop();
        }
        catch(const std::exception& e)
        {
            std::cerr << "~Ten_camera pipe.stop() failed: " << e.what() << '\n';
        }
        catch (...)
        {
            std::cerr << "~Ten_camera pipe.stop() failed: unknown error" << '\n';
        }
    }
private:

    /**
        @brief 设置分辨率和帧率
        @param w: 1920 640
        @param h: 1080 480
        @param fps: 帧率
        @return bool:设置是否成功
    */
   bool reset_camera_once(size_t w, size_t h, size_t fps);

   // /**
   //     @brief 高效读取图片
   //     @param int: 无实际意义，用于函数重载
   //     @return cv::Mat* 外面要delete 对象
   // */

   // cv::Mat* camera_read(int);
   /**
       @brief 设置分辨率和帧率（启用深度和bgr双加载）
       @param w: 1920 640
       @param h: 1080 480
       @param fps: 帧率
       @return bool:设置是否成功
   */
   bool reset_camera_depth_once(size_t w, size_t h, size_t fps);

    Ten_camera(size_t w, size_t h, size_t fps)
    {
        try
        {
            _w = w;
            _h = h;
            _fps = fps;
            config.enable_stream(RS2_STREAM_COLOR, w, h, RS2_FORMAT_BGR8, fps);
            pipe.start(config);
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            int retry = 0;
            const int max_retry = 30;
            while(!reset_camera_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag() && retry < max_retry)
            {
                usleep(100000);
                retry++;
            }
            if (retry >= max_retry)
                std::cerr << "D435 初始化失败，已重试 " << max_retry << " 次，跳过" << std::endl;
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            std::cerr << "相机接受：未知致命异常" << std::endl;
            int retry = 0;
            const int max_retry = 30;
            while(!reset_camera_once(_w, _h, _fps) && Ten::_TREADPOOL_FLAG_.read_flag() && retry < max_retry)
            {
                usleep(100000);
                retry++;
            }
            if (retry >= max_retry)
                std::cerr << "D435 初始化失败，已重试 " << max_retry << " 次，跳过" << std::endl;
        }
    }

    static std::unique_ptr<Ten_camera> create(size_t w = 1920, size_t h = 1080, size_t fps = 30) {
        // 静态函数可访问私有构造函数，直接new对象后封装为unique_ptr
        return std::unique_ptr<Ten_camera>(new Ten_camera(w, h, fps));
    }

rs2::pipeline pipe;
rs2::config config;
rs2_intrinsics color_intr_;
std::mutex read_mtx_;
size_t _w = 0;
size_t _h = 0;
size_t _fps = 0;
static std::once_flag camera_flag_;
rs2::align align_to_color_{RS2_STREAM_COLOR};
};





}








#endif


