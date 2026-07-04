#ifndef __ROS_DEBUG_H_
#define __ROS_DEBUG_H_

// ROS 基础 & 消息
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

// OpenCV
#include <opencv2/opencv.hpp>

// PCL & ROS-PCL 转换（已包含 Ptr/ConstPtr 定义）
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

// 标准库
#include <string>
#include <map>
#include <mutex>
#include <memory>

#include <cstring>

namespace Ten
{
    namespace debug
    {
        /**
         * @brief 图像调试发布单例类
         * @details 入参 cv::Mat，自动转ROS图像，多线程安全
         */
        class DebugImagePub
        {
        public:
            // 禁用拷贝 & 赋值
            DebugImagePub(const DebugImagePub&) = delete;
            DebugImagePub& operator=(const DebugImagePub&) = delete;

            static DebugImagePub& GetInstance();

            /**
             * @brief 图像发布接口
             * @param mat OpenCV 图像
             * @param topic_name 话题名
             * @return 发布结果
             */
            bool publish(const cv::Mat& mat, const std::string& topic_name);

            ~DebugImagePub() = default;

        private:
            DebugImagePub();
            static std::unique_ptr<DebugImagePub> create();

            // cv::Mat 转 sensor_msgs::Image 工具函数
            void matToImage(const cv::Mat& mat, sensor_msgs::Image& ros_img);

            ros::NodeHandle nh_;
            std::map<std::string, ros::Publisher> pub_map_;
            std::mutex mtx_;
            static std::once_flag init_flag_;
        };

        /**
         * @brief 点云调试发布单例类（非类模板，使用模板成员函数）
         * @details 全局唯一单例，使用PCL原生 PointCloud<PointT>::ConstPtr
         */
        class DebugPointCloudPub
        {
        public:
            // 禁用拷贝 & 赋值
            DebugPointCloudPub(const DebugPointCloudPub&) = delete;
            DebugPointCloudPub& operator=(const DebugPointCloudPub&) = delete;

            static DebugPointCloudPub& GetInstance();

            /**
             * @brief 点云发布【函数模板】
             * @tparam PointT PCL点类型(PointXYZ/PointXYZRGB/PointXYZI...)
             * @param cloud_ptr PCL 原生只读智能指针 pcl::PointCloud<PointT>::ConstPtr
             * @param topic_name ROS 发布话题名
             * @return bool 发布成功/失败
             */
            template<typename PointT>
            bool publish(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr,
                         const std::string& topic_name);

            ~DebugPointCloudPub() = default;

        private:
            DebugPointCloudPub();
            static std::unique_ptr<DebugPointCloudPub> create();

            ros::NodeHandle nh_;
            // 所有点类型、所有话题共用一套发布者容器
            std::map<std::string, ros::Publisher> pub_map_;
            std::mutex mtx_;
            static std::once_flag init_flag_;
        };

        // ===================== 模板成员函数实现（PCL 原生 ConstPtr 版本） =====================
        template<typename PointT>
        bool DebugPointCloudPub::publish(typename pcl::PointCloud<PointT>::ConstPtr cloud_ptr,
                                         const std::string& topic_name)
        {
            // 多线程锁保护 map 读写
            std::lock_guard<std::mutex> lock(mtx_);

            // 1. 空指针 + 基础参数校验（PCL boost::shared_ptr 判空语法一致）
            if (!cloud_ptr)
            {
                std::cerr << "[DebugPointCloudPub] Error: cloud pointer is null!" << std::endl;
                return false;
            }
            if (topic_name.empty())
            {
                std::cerr << "[DebugPointCloudPub] Error: empty topic name!" << std::endl;
                return false;
            }
            if (cloud_ptr->empty())
            {
                std::cerr << "[DebugPointCloudPub] Error: empty point cloud!" << std::endl;
                return false;
            }

            try
            {
                // 查找话题对应的发布者，不存在则新建
                auto iter = pub_map_.find(topic_name);
                if (iter == pub_map_.end())
                {
                    ros::Publisher pub = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, 10);
                    pub_map_.emplace(topic_name, pub);
                    std::cout << "[DebugPointCloudPub] Create new publisher: " << topic_name << std::endl;
                }

                // 内部用 std::unique_ptr 管理临时 ROS 点云消息
                std::unique_ptr<sensor_msgs::PointCloud2> ros_cloud_ptr =
                    std::make_unique<sensor_msgs::PointCloud2>();

                // PCL点云 转 ROS PointCloud2
                pcl::toROSMsg(*cloud_ptr, *ros_cloud_ptr);
                ros_cloud_ptr->header.stamp = ros::Time::now();
                ros_cloud_ptr->header.frame_id = "camera_init";
                // 执行发布
                pub_map_[topic_name].publish(*ros_cloud_ptr);

                return true;
            }
            catch (const std::exception& e)
            {
                std::cerr << "[DebugPointCloudPub] Exception: " << e.what() << std::endl;
                return false;
            }
            catch (...)
            {
                std::cerr << "[DebugPointCloudPub] Unknown fatal exception!" << std::endl;
                return false;
            }
        }

    } // namespace debug
} // namespace Ten

#endif // __ROS_DEBUG_H_
