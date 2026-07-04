#ifndef __LIVOX_ROS_DRIVER_H_
#define __LIVOX_ROS_DRIVER_H_
#include <iostream>
#include <chrono>
#include <vector>
#include <csignal>
#include <thread>
#include <future>
#include "include/livox_ros_driver2.h"
#include "include/ros_headers.h"
#include "include/ros1_headers.h"
#include "driver_node.h"
#include "lddc.h"
#include "lds_lidar.h"
#include "./../../lidar.h"

using namespace livox_ros;

namespace Ten
{


class Ten_lidar
{
public:
    Ten_lidar(const Ten_lidar& lidar) = delete;
    Ten_lidar& operator=(const Ten_lidar& lidar) = delete;
    /**
        @brief: 获取雷达实例
        @param: user_config_path: 雷达配置文件路径
        @param: xfer_format: 发送雷达消息用的格式： kPointCloud2Msg, kLivoxCustomMsg, kPclPxyziMsg, kLivoxImuMsg.
        @param: publish_freq: 发布频率
        @param: multi_topic: 默认只发布一个话题
        @param: data_src: 数据来源， 默认（kSourceRawLidar）硬件雷达
        @param: output_type: 输出到哪， 默认（kOutputToRos）Ros话题
        @param: frame_id: 点云坐标系
        @param：lidar_bag： 是否录制雷达bag
        @param: imu_bag: 是否录制imu bag;
        @return Ten_lidar& 返回Ten_lidar实例
    */
    static Ten_lidar& GetInstance(std::string user_config_path = "../config/MID360_config.json", int xfer_format = kLivoxCustomMsg, double publish_freq  = 10.0,
    int multi_topic = 0, int data_src = kSourceRawLidar, int output_type = kOutputToRos, std::string frame_id = "livox_frame",
    bool lidar_bag = false, bool imu_bag = false);
    
    ~Ten_lidar()
    {

    }

    
private:
    Ten_lidar(){}
    static std::unique_ptr<Ten_lidar> create() {
        // 静态函数可访问私有构造函数，直接new对象后封装为unique_ptr
        return std::unique_ptr<Ten_lidar>(new Ten_lidar());
    }

livox_ros::DriverNode livox_node_;
static std::once_flag lidar_flag_;


};








}












#endif 

