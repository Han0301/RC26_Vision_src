#ifndef __LIDAR_RECOGNITION_H_
#define __LIDAR_RECOGNITION_H_

#include <ros/ros.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include<iostream>
#include<thread>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_msgs/TFMessage.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>  // 体素滤波类
#include <pcl/common/transforms.h> // 包含变换函数
#include <pcl/segmentation/sac_segmentation.h>  // RANSAC平面分割
#include <pcl/filters/extract_indices.h>  // 提取地面/非地面点
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/registration/icp.h>          // ICP配准类


//my .h
#include"./../coordinate.h"
#include"./../lidar.h"

namespace Ten
{

    namespace lidar
    {
        #define _external_box_x_min_ 0.05
        #define _external_box_y_min_ -5.8
        #define _external_box_z_min_ 0.05
        #define _external_box_x_max_ 3.15
        #define _external_box_y_max_ -0.05
        #define _external_box_z_max_ 1.3

        #define _internal_box_x_min_ -1.2
        #define _internal_box_y_min_ -0.6
        #define _internal_box_z_min_ -0.1
        #define _internal_box_x_max_ 0.1
        #define _internal_box_y_max_ 0.6
        #define _internal_box_z_max_ 1.8


        //分割区间
        struct segmentation
        {
            segmentation(float x_min = 0.0, float y_min = 0.0, float z_min = 0.0, float x_max = 0.0, float y_max = 0.0, float z_max = 0.0)
                :_x_min(x_min)
                ,_y_min(y_min)
                ,_z_min(z_min)
                ,_x_max(x_max)
                ,_y_max(y_max)
                ,_z_max(z_max)
            {}

            /**
             * @brief 重新设置分割区间
             */
            void reset_segmentation(float x_min = 0.0, float y_min = 0.0, float z_min = 0.0, float x_max = 0.0, float y_max = 0.0, float z_max = 0.0)
            {
                _x_min = x_min;
                _y_min = y_min;
                _z_min = z_min;
                _x_max = x_max;
                _y_max = y_max;
                _z_max = z_max;
            }

            float _x_min;
            float _y_min;
            float _z_min;
            float _x_max;
            float _y_max;
            float _z_max;
        };
        //设置包围盒
        class bounding_box
        {
        public:
            bounding_box()
                :_external_box(_external_box_x_min_, _external_box_y_min_, _external_box_z_min_, _external_box_x_max_, _external_box_y_max_, _external_box_z_max_)
                ,_internal_box(_internal_box_x_min_, _internal_box_y_min_, _internal_box_z_min_, _internal_box_x_max_, _internal_box_y_max_, _internal_box_z_max_)
            {

            }

            /**
             * @brief 重新设置external_box
             */
            void reset_external_box(float x_min = 0.0, float y_min = 0.0, float z_min = 0.0, float x_max = 0.0, float y_max = 0.0, float z_max = 0.0)
            {
                _external_box.reset_segmentation(x_min, y_min, z_min, x_max, y_max, z_max);
            }

            /**
             * @brief 重新设置internal_box
             */
            void reset_internal_box(float x_min = 0.0, float y_min = 0.0, float z_min = 0.0, float x_max = 0.0, float y_max = 0.0, float z_max = 0.0)
            {
                _internal_box.reset_segmentation(x_min, y_min, z_min, x_max, y_max, z_max);
            }

            /**
             * @brief 是否在外部包围盒的里面
             * @param x: x
             * @param y: y
             * @param z; z
             * @return bool: true 在包围盒内
             */
            bool inside_the_box(float x, float y, float z)
            {
                if(_external_box._x_min < x && _external_box._x_max > x)
                {
                    if(_external_box._y_min < y && _external_box._y_max > y)
                    {
                        if(_external_box._z_min < z && _external_box._z_max > z)
                        {
                            return true;
                        }
                    }
                }
                return false;
            }

            /**
             * @brief 是否在外部包围盒的外面
             * @param x: x
             * @param y: y
             * @param z; z
             * @return bool: true 在包围盒外
             */
            bool outside_the_box(float x, float y, float z)
            {
                if(_internal_box._x_min > x || _internal_box._x_max < x)
                {
                    return true;
                }
                if(_internal_box._y_min > y || _internal_box._y_max < y)
                {
                    return true;
                }
                if(_internal_box._z_min > z || _internal_box._z_max < z)
                {
                    return true;
                }
                return false;
            }
        private:
            segmentation _external_box; //外部包围盒
            segmentation _internal_box; //内部包围盒
        };

        //将雷达数据初步处理转成点云，提供调试功能
        class lidar_recogniton
        {
        public:
            lidar_recogniton()
                :_cloud_current(new pcl::PointCloud<pcl::PointXYZI>)
                ,_nh("/")
            {
                _cloud_current_pub = _nh.advertise<sensor_msgs::PointCloud2>("/maple/point_cloud_current", 10);
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr get_current_cloud()
            {
                //获取雷达在当前坐标系或者重定位坐标系下的变换矩阵
                Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data())); 
                Eigen::Matrix4d world2tolidar = Ten::_COORDINATE_TRANSFORMATION_.get_world2tolidar();
                //调试打印位置
                Ten::XYZRPY xyzrpy = Ten::transform_matrixtoXYZRPY(world2tolidar);
                std::cout << "---------------------------" << std::endl; 
                std::cout << "x: " << xyzrpy._xyz._x << std::endl;
                std::cout << "y: " << xyzrpy._xyz._y << std::endl;
                std::cout << "z: " << xyzrpy._xyz._z << std::endl;
                std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
                std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
                std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;
                std::lock_guard<std::mutex> lock(mtx_);
                //得到点云
                _cloud_current = livoxCustomMsgToPCLXYZI(_LIVOX_GET_.read_data(), world2tolidar.inverse());
                return _cloud_current;
            }

            void point_cloud_debug()
            {
                std::lock_guard<std::mutex> lock(mtx_);
                publishPCLXYZItoROS(_cloud_current, _cloud_current_pub);
            }

        private: 
            bounding_box _filter;
            pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_current;
            mutable std::mutex mtx_;
            ros::NodeHandle _nh;
            ros::Publisher _cloud_current_pub;
            /**
             * @brief Livox CustomMsg 转换为 PCL PointXYZI 点云（坐标+反射强度）
             * @param msg 输入：Livox 雷达自定义消息（普通引用）
             * @return 输出：PCL PointXYZI 点云智能指针（全全称写法）
             */
            pcl::PointCloud<pcl::PointXYZI>::Ptr livoxCustomMsgToPCLXYZI(const livox_ros_driver::CustomMsg& msg, Eigen::Matrix4d transform_matrix = Eigen::Matrix4d())
            {
                // 初始化PCL点云对象
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
                
                // 继承原始消息的坐标系和时间戳
                cloud->header.frame_id = msg.header.frame_id;
                cloud->header.stamp = pcl_conversions::toPCL(msg.header.stamp);
                
                // 预分配内存，提升转换效率
                cloud->points.reserve(msg.points.size());

                // 遍历所有激光点，完成数据转换
                for (const auto& livox_point : msg.points)
                {
                    //坐标转化
                    //过滤自身点云
                    if(!_filter.outside_the_box(livox_point.x, livox_point.y, livox_point.z))
                    {
                        continue;
                    }
                    pcl::PointXYZI p = transformLivoxPointToPCL(livox_point, transform_matrix);
                    //过滤外部点云
                    if(_filter.inside_the_box(p.x, p.y, p.z))
                    {
                        // std::cout << "debug-lidar-point-pose" << std::endl;
                        // std::cout << "x: " << p.x << std::endl;
                        // std::cout << "y: " << p.y << std::endl;
                        // std::cout << "z: " << p.z << std::endl;
                        cloud->push_back(p);
                    }
                }
                std::cout << "cloud->size(): " << cloud->size() << std::endl;
                // 设置点云基础属性（无序点云）
                cloud->width = cloud->points.size();
                cloud->height = 1;
                cloud->is_dense = true;
                return cloud;
            }

            /**
             * @brief 发布 PCL PointXYZI 点云 到 ROS 标准话题
             * @param pcl_cloud 输入：PCL PointXYZI 点云指针
             * @param publisher 输入：ROS 发布器（用于发送话题）
             * @brief 功能：自动转换为 sensor_msgs/PointCloud2 并发布
             */
            void publishPCLXYZItoROS(pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud, ros::Publisher& publisher)
            {
                if (!pcl_cloud || pcl_cloud->empty())
                {
                    std::cout << "点云为空，跳过发布" << std::endl;
                    return;
                }

                sensor_msgs::PointCloud2 ros_cloud_msg;

                // 1️⃣ 第一步：先做PCL→ROS转换
                pcl::toROSMsg(*pcl_cloud, ros_cloud_msg);

                // 2️⃣ 第二步：再设置坐标系和时间戳（必须在转换之后！）
                // 随便写一个名字，不需要存在，不需要TF，纯自定义
                ros_cloud_msg.header.frame_id = "test_lidar"; 
                ros_cloud_msg.header.stamp = ros::Time::now();

                // 3️⃣ 发布
                publisher.publish(ros_cloud_msg);
            }


            /**
             * @brief 单点坐标变换 + 强度赋值
             * @param livox_point 输入：Livox原始单个激光点（常量引用，不修改原值）
             * @param transform_matrix 输入：4x4齐次变换矩阵（常量引用）
             * @return 输出：变换后的PCL PointXYZI点
             */
            pcl::PointXYZI transformLivoxPointToPCL(
                const livox_ros_driver::CustomPoint& livox_point,
                const Eigen::Matrix4d& transform_matrix
            )
            {
                // 1. 构建【齐次坐标】(x,y,z,1)，适配4x4矩阵变换
                // Eigen::Vector4d = [x, y, z, 1.0]
                Eigen::Vector4d homogeneous_point(
                    static_cast<double>(livox_point.x),
                    static_cast<double>(livox_point.y),
                    static_cast<double>(livox_point.z),
                    1.0
                );

                // 2. 执行矩阵变换（核心：左乘齐次变换矩阵）
                Eigen::Vector4d transformed_point = transform_matrix * homogeneous_point;

                // 3. 初始化PCL点，赋值变换后的坐标
                pcl::PointXYZI pcl_point;
                // 变换后的坐标转回float（PCL PointXYZI坐标类型为float）
                pcl_point.x = static_cast<float>(transformed_point.x());
                pcl_point.y = static_cast<float>(transformed_point.y());
                pcl_point.z = static_cast<float>(transformed_point.z());

                // 4. 赋值反射强度（沿用你修正的reflectivity字段）
                pcl_point.intensity = static_cast<float>(livox_point.reflectivity);

                // 5. 返回变换完成的PCL点
                return pcl_point;
            }


        };


    }


}

#endif