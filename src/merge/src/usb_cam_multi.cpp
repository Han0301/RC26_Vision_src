#include "usb_cam_multi.h"

namespace Ten
{

// 静态变量初始化
std::once_flag Ten_usb_cam_multi::camera_flag_;

// 单例获取
Ten_usb_cam_multi& Ten_usb_cam_multi::GetInstance(std::vector<int> dev_idxs, size_t w, size_t h, size_t fps)
{
    static std::unique_ptr<Ten_usb_cam_multi> usb_cam_multi = nullptr;
    std::call_once(camera_flag_, [&]()
    {
        usb_cam_multi = create(dev_idxs, w, h, fps);
        std::cout << "init_usb_cam_multi" << std::endl;
    });
    return *usb_cam_multi;
}

// 创建实例
std::unique_ptr<Ten_usb_cam_multi> Ten_usb_cam_multi::create(std::vector<int> dev_idxs, size_t w, size_t h, size_t fps)
{
    return std::unique_ptr<Ten_usb_cam_multi>(new Ten_usb_cam_multi(dev_idxs, w, h, fps));
}

// 构造函数
Ten_usb_cam_multi::Ten_usb_cam_multi(std::vector<int> dev_idxs, size_t w, size_t h, size_t fps)
    : w_(w), h_(h), fps_(fps), dev_idxs_(std::move(dev_idxs))
{
    // 遍历初始化所有相机，仅尝试一次
    for (int idx : dev_idxs_)
    {
        try
        {
            init_single_camera(idx);
        }
        catch (const std::exception& e)
        {
            std::cerr << "相机 /dev/video" << idx << " 初始化异常: " << e.what() << std::endl;
        }
        catch (...)
        {
            std::cerr << "相机 /dev/video" << idx << " 未知异常" << std::endl;
        }
    }
}

// 析构函数
Ten_usb_cam_multi::~Ten_usb_cam_multi()
{
    std::lock_guard<std::mutex> lock(read_mtx_);
    for (auto& pair : caps_)
    {
        if (pair.second.isOpened())
            pair.second.release();
    }
}

// 单个摄像头初始化
bool Ten_usb_cam_multi::init_single_camera(int dev_idx)
{
    try
    {
        // 释放旧资源
        if (caps_[dev_idx].isOpened())
            caps_[dev_idx].release();

        // 强制V4L2后端
        if (!caps_[dev_idx].open(dev_idx, cv::CAP_V4L2))
        {
            std::cerr << "相机 /dev/video" << dev_idx << " 打开失败" << std::endl;
            return false;
        }

        // 核心参数配置
        caps_[dev_idx].set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        caps_[dev_idx].set(cv::CAP_PROP_FRAME_WIDTH, w_);
        caps_[dev_idx].set(cv::CAP_PROP_FRAME_HEIGHT, h_);
        caps_[dev_idx].set(cv::CAP_PROP_FPS, fps_);
        caps_[dev_idx].set(cv::CAP_PROP_BUFFERSIZE, 3);

        std::cout << "相机 /dev/video" << dev_idx << " 初始化成功" << std::endl;
        return true;
    }
    catch (...)
    {
        std::cerr << "相机 /dev/video" << dev_idx << " 初始化失败" << std::endl;
        return false;
    }
}

// 读取指定摄像头
cv::Mat Ten_usb_cam_multi::camera_read(int dev_idx)
{
    std::lock_guard<std::mutex> lock(read_mtx_);

    // 不在初始化列表，返回空图片
    if (std::find(dev_idxs_.begin(), dev_idxs_.end(), dev_idx) == dev_idxs_.end())
    {
        return cv::Mat();
    }

    try
    {
        cv::Mat frame;
        if (!caps_[dev_idx].read(frame))
        {
            // 读取失败，重新初始化一次
            init_single_camera(dev_idx);
            return cv::Mat();
        }
        return frame;
    }
    catch (...)
    {
        // 异常，重新初始化一次
        init_single_camera(dev_idx);
        return cv::Mat();
    }
}

// 重置指定摄像头
bool Ten_usb_cam_multi::reset_camera(int dev_idx)
{
    std::lock_guard<std::mutex> lock(read_mtx_);

    // 不在初始化列表，直接返回false
    if (std::find(dev_idxs_.begin(), dev_idxs_.end(), dev_idx) == dev_idxs_.end())
    {
        return false;
    }

    // 仅重置一次，返回结果
    return init_single_camera(dev_idx);
}

}
