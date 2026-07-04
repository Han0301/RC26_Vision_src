#ifndef __LIVOX_ROS_DRIVER_CPP_
#define __LIVOX_ROS_DRIVER_CPP_
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
#include "livox_ros_driver.h"
#include "./../../threadpool.h"
using namespace livox_ros;

namespace Ten
{


// class Ten_lidar
// {
// public:
//     Ten_lidar(const Ten_lidar& lidar) = delete;
//     Ten_lidar& operator=(const Ten_lidar& lidar) = delete;
//     /**
//         @brief: 获取雷达实例
//         @param: user_config_path: 雷达配置文件路径
//         @param: xfer_format: 发送雷达消息用的格式： kPointCloud2Msg, kLivoxCustomMsg, kPclPxyziMsg, kLivoxImuMsg.
//         @param: publish_freq: 发布频率
//         @param: multi_topic: 默认只发布一个话题
//         @param: data_src: 数据来源， 默认（kSourceRawLidar）硬件雷达
//         @param: output_type: 输出到哪， 默认（kOutputToRos）Ros话题
//         @param: frame_id: 点云坐标系
//         @param：lidar_bag： 是否录制雷达bag
//         @param: imu_bag: 是否录制imu bag;
//         @return Ten_lidar& 返回Ten_lidar实例
//     */
//     static Ten_lidar& GetInstance(std::string user_config_path = "../config/MID360_config.json", int xfer_format = kLivoxCustomMsg, double publish_freq  = 10.0,
//     int multi_topic = 0, int data_src = kSourceRawLidar, int output_type = kOutputToRos, std::string frame_id = "livox_frame",
//     bool lidar_bag = false, bool imu_bag = false)
//     {
//         static std::unique_ptr<Ten_lidar> ten_lidar = nullptr;
//         std::call_once(lidar_flag_, 
//             [user_config_path, xfer_format, publish_freq, multi_topic, data_src, output_type, frame_id, lidar_bag, imu_bag]
//             () mutable
//             {
//                 ten_lidar = create();
//                 if (publish_freq > 100.0) {
//                     publish_freq = 100.0;
//                 } else if (publish_freq < 0.5) {
//                     publish_freq = 0.5;
//                 } else {
//                     publish_freq = publish_freq;
//                 }
//                 ten_lidar->livox_node_.future_ = ten_lidar->livox_node_.exit_signal_.get_future();
//                 /** Lidar data distribute control and lidar data source set */
//                 ten_lidar->livox_node_.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
//                                         publish_freq, frame_id, lidar_bag, imu_bag);
//                 ten_lidar->livox_node_.lddc_ptr_->SetRosNode(&(ten_lidar->livox_node_));

//                 if (data_src == kSourceRawLidar) {
//                     DRIVER_INFO(ten_lidar->livox_node_, "Data Source is raw lidar.");
//                     DRIVER_INFO(ten_lidar->livox_node_, "Config file : %s", user_config_path.c_str());

//                     LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
//                     ten_lidar->livox_node_.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

//                     if ((read_lidar->InitLdsLidar(user_config_path))) {
//                     DRIVER_INFO(ten_lidar->livox_node_, "Init lds lidar successfully!");
//                     } else {
//                     DRIVER_ERROR(ten_lidar->livox_node_, "Init lds lidar failed!");
//                     }
//                 } else {
//                     DRIVER_ERROR(ten_lidar->livox_node_, "Invalid data src (%d), please check the launch file", data_src);
//                 }

//                 ten_lidar->livox_node_.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &(ten_lidar->livox_node_));
//                 ten_lidar->livox_node_.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &(ten_lidar->livox_node_));
//                 std::cout << "init_lidar" << std::endl;
        
//             });

//         return *ten_lidar;
//     }
    
//     ~Ten_lidar()
//     {

//     }

    
// private:
//     Ten_lidar(){}
//     static std::unique_ptr<Ten_lidar> create() {
//         // 静态函数可访问私有构造函数，直接new对象后封装为unique_ptr
//         return std::unique_ptr<Ten_lidar>(new Ten_lidar());
//     }

// livox_ros::DriverNode livox_node_;
// static std::once_flag lidar_flag_;


// };

    std::once_flag Ten_lidar::lidar_flag_;


    Ten_lidar& Ten_lidar::GetInstance(std::string user_config_path, int xfer_format, double publish_freq,
    int multi_topic, int data_src, int output_type, std::string frame_id,
    bool lidar_bag, bool imu_bag)
    {
        static std::unique_ptr<Ten_lidar> ten_lidar = nullptr;
        std::call_once(lidar_flag_, 
            [user_config_path, xfer_format, publish_freq, multi_topic, data_src, output_type, frame_id, lidar_bag, imu_bag]
            () mutable
            {
                ten_lidar = create();
                if (publish_freq > 100.0) {
                    publish_freq = 100.0;
                } else if (publish_freq < 0.5) {
                    publish_freq = 0.5;
                } else {
                    publish_freq = publish_freq;
                }
                ten_lidar->livox_node_.future_ = ten_lidar->livox_node_.exit_signal_.get_future();
                /** Lidar data distribute control and lidar data source set */
                ten_lidar->livox_node_.lddc_ptr_ = std::make_unique<Lddc>(xfer_format, multi_topic, data_src, output_type,
                                        publish_freq, frame_id, lidar_bag, imu_bag);
                ten_lidar->livox_node_.lddc_ptr_->SetRosNode(&(ten_lidar->livox_node_));

                if (data_src == kSourceRawLidar) {
                    DRIVER_INFO(ten_lidar->livox_node_, "Data Source is raw lidar.");
                    DRIVER_INFO(ten_lidar->livox_node_, "Config file : %s", user_config_path.c_str());

                    LdsLidar *read_lidar = LdsLidar::GetInstance(publish_freq);
                    ten_lidar->livox_node_.lddc_ptr_->RegisterLds(static_cast<Lds *>(read_lidar));

                    if ((read_lidar->InitLdsLidar(user_config_path))) {
                    DRIVER_INFO(ten_lidar->livox_node_, "Init lds lidar successfully!");
                    } else {
                    DRIVER_ERROR(ten_lidar->livox_node_, "Init lds lidar failed!");
                    }
                } else {
                    DRIVER_ERROR(ten_lidar->livox_node_, "Invalid data src (%d), please check the launch file", data_src);
                }

                ten_lidar->livox_node_.pointclouddata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::PointCloudDataPollThread, &(ten_lidar->livox_node_));
                ten_lidar->livox_node_.imudata_poll_thread_ = std::make_shared<std::thread>(&DriverNode::ImuDataPollThread, &(ten_lidar->livox_node_));
                std::cout << "init_lidar" << std::endl;
        
            });

        return *ten_lidar;
    }





}

void DriverNode::PointCloudDataPollThread()
{
    //urcu_memb_register_thread();
    std::future_status status;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    do {
        lddc_ptr_->DistributePointCloudData();
        status = future_.wait_for(std::chrono::microseconds(0));
    } while ((status == std::future_status::timeout));
    //urcu_memb_unregister_thread();
}

void DriverNode::ImuDataPollThread()
{
    //urcu_memb_register_thread();
    std::future_status status;
    std::this_thread::sleep_for(std::chrono::seconds(3));
    do {
        lddc_ptr_->DistributeImuData();
        status = future_.wait_for(std::chrono::microseconds(0));
    } while ((status == std::future_status::timeout));
    //urcu_memb_unregister_thread();
}














#endif 

