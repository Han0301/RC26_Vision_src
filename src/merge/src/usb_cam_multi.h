#ifndef __USB_CAM_MULTI_H_
#define __USB_CAM_MULTI_H_

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <iostream>
#include <string>
#include <mutex>
#include <unistd.h>
#include <vector>
#include <map>
#include <memory>
#include <algorithm>
#include "threadpool.h"

namespace Ten
{

class Ten_usb_cam_multi
{
public:
    // 禁用拷贝构造
    Ten_usb_cam_multi(const Ten_usb_cam_multi&) = delete;
    // 禁用赋值运算符
    Ten_usb_cam_multi& operator=(const Ten_usb_cam_multi&) = delete;

    /**
     * @brief 单例获取：传入所有摄像头ID数组
     * @param dev_idxs 摄像头设备号列表
     * @param w 分辨率宽
     * @param h 分辨率高
     * @param fps 帧率
     */
    static Ten_usb_cam_multi& GetInstance(std::vector<int> dev_idxs, size_t w = 640, size_t h = 480, size_t fps = 30);

    /**
     * @brief 读取指定摄像头画面
     * @param dev_idx 设备号
     * @return 帧图片（无效返回空Mat）
     */
    cv::Mat camera_read(int dev_idx);

    /**
     * @brief 重置指定摄像头
     * @param dev_idx 设备号
     * @return 成功true，失败false
     */
    bool reset_camera(int dev_idx);

    ~Ten_usb_cam_multi();

private:
    // 私有构造
    Ten_usb_cam_multi(std::vector<int> dev_idxs, size_t w, size_t h, size_t fps);

    // 单例创建
    static std::unique_ptr<Ten_usb_cam_multi> create(std::vector<int> dev_idxs, size_t w, size_t h, size_t fps);

    // 初始化单个摄像头（内部调用）
    bool init_single_camera(int dev_idx);

private:
    static std::once_flag camera_flag_;

    std::vector<int> dev_idxs_;               // 初始化的摄像头ID列表
    std::map<int, cv::VideoCapture> caps_;    // 摄像头句柄映射
    std::mutex read_mtx_;                     // 线程安全锁
    size_t w_;
    size_t h_;
    size_t fps_;
};

}

#endif
