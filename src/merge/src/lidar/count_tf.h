#ifndef __COUNT_TF_H_
#define __COUNT_TF_H_

// ROS核心
#include <ros/ros.h>
// Livox自定义消息头
#include <livox_ros_driver/CustomMsg.h>
// PCL点云相关头文件
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
// ROS与PCL格式转换
#include <pcl_conversions/pcl_conversions.h>
#include "./../serial.h"
#include "./../threadpool.h"
#include "./../lidar.h"
#include "./../relocation.h"
#include "./../coordinate.h"
#include "./../method_math.h"
// 新增：PCL 内置点云变换函数（必须加）
#include <pcl/common/transforms.h>
// 浮点合法性判断头文件
#include <cmath>

namespace Ten
{
    class count_tf
    {
    
    };
}

#endif
