#ifndef __GET_CLOUD_H_
#define __GET_CLOUD_H_

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
    namespace cloud
    {
        struct cuboid
        {
            cuboid(float x_min = 0.0, float x_max = 0.0, float y_min = 0.0, float y_max = 0.0, float z_min = 0.0, float z_max = 0.0)
                :_x_min(x_min)
                ,_x_max(x_max)
                ,_y_min(y_min)
                ,_y_max(y_max)
                ,_z_min(z_min)
                ,_z_max(z_max)
            {}

            /**
             * @brief 重置区间
             */
            void reset_segmentation(float x_min = 0.0, float x_max = 0.0, float y_min = 0.0, float y_max = 0.0, float z_min = 0.0, float z_max = 0.0)
            {
                _x_min = x_min;
                _x_max = x_max;
                _y_min = y_min;
                _y_max = y_max;
                _z_min = z_min;
                _z_max = z_max;
            }

            /**
             * @brief 打印所有成员变量
             * @note 加 const 修饰：保证打印操作不会修改结构体成员
             */
            void print() const
            {
                std::cout << "===== Cuboid 立方体区间信息 =====" << std::endl;
                std::cout << "x_min: " << _x_min << "  |  x_max: " << _x_max << std::endl;
                std::cout << "y_min: " << _y_min << "  |  y_max: " << _y_max << std::endl;
                std::cout << "z_min: " << _z_min << "  |  z_max: " << _z_max << std::endl;
                std::cout << "==================================" << std::endl;
            }

            float _x_min;
            float _x_max;
            float _y_min;
            float _y_max;
            float _z_min;
            float _z_max;
        };

        class get_cloud
        {
        public:
            // ===================== 新增：构造函数 初始化累加点云 =====================
            get_cloud()
            {
                // 初始化过滤累加点云
                filter_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>);
            }

            /**
             * @brief 返回指定区域点的数量（原有逻辑完全保留）
             * @param areas 指定区域
             * @return std::vector<int>指定区域点的数量
             */
            std::vector<int> get_hope_cloud_num(std::vector<cuboid>& areas)
            {
                std::vector<int> global_area_point_nums;
                global_area_point_nums.resize(areas.size(), 0);
                int total_num = 10;
                Eigen::Matrix4d T1 = Ten::_COORDINATE_TRANSFORMATION_.get_world1toworld2();
                ros::Rate sl(Ten::_laser_pub_hz_*2);
                ros::Rate cloudsl(20);
                while(Ten::_TREADPOOL_FLAG_.read_flag() && total_num > 0)
                {
                    nav_msgs::Odometry odo;
                    if(!Ten::_TF_GET_.get_latest(odo))
                    {
                        sl.sleep();
                        continue;
                    }
                    Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
                    Eigen::Matrix4d T = T1 * Ten::math::XYZRPYtotransform_matrix_fixed(pose);
                    livox_ros_driver::CustomMsg msg;
                    if(!Ten::_LIVOX_GET_.pop(msg))
                    {
                        cloudsl.sleep();
                        continue;
                    }
                    pcl::PointCloud<pcl::PointXYZI>::Ptr raw = livoxCustomMsgToPointXYZI(msg);
                    std::cout << "raw_size: " << raw->size() << std::endl;
                    std::vector<int> area_point_nums = countPointsInCuboids(raw, T, areas);
                    for(size_t i = 0; i < area_point_nums.size() && i < global_area_point_nums.size(); i++)
                    {
                        global_area_point_nums[i] += area_point_nums[i];
                    }
                    total_num--;
                }
                return global_area_point_nums;
            }

            // ===================== 新增：获取累加后的过滤点云 =====================
            /**
             * @brief 获取所有帧累加后的目标区域点云
             * @return 常指针，禁止外部篡改内部点云
             */
            pcl::PointCloud<pcl::PointXYZI>::Ptr get_filter_cloud() 
            {
                return filter_cloud_;
            }

            // ===================== 新增：清空累加点云 =====================
            /**
             * @brief 清空累加的过滤点云，重置状态
             */
            void clear()
            {
                if(filter_cloud_)
                {
                    filter_cloud_->clear();
                }
            }

        private:
            // ===================== 新增：私有成员 逐帧累加的过滤点云 =====================
            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_;

            /**
             * @brief 将 Livox CustomMsg 转换为 PCL PointXYZI 点云
             * @param custom_msg Livox雷达自定义消息（普通引用）
             * @return PCL PointXYZI 点云智能指针
             */
            pcl::PointCloud<pcl::PointXYZI>::Ptr livoxCustomMsgToPointXYZI(const livox_ros_driver::CustomMsg& custom_msg)
            {
                // 初始化PCL点云智能指针
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

                // 同步头部信息：时间戳 + 坐标系ID
                cloud->header.stamp = pcl_conversions::toPCL(custom_msg.header.stamp);
                cloud->header.frame_id = custom_msg.header.frame_id;
                
                // 预分配内存，提高转换效率
                cloud->reserve(custom_msg.points.size());

                // 遍历所有点并转换
                for (const auto& livox_point : custom_msg.points)
                {
                    pcl::PointXYZI p;
                    // 赋值XYZ坐标
                    p.x = livox_point.x;
                    p.y = livox_point.y;
                    p.z = livox_point.z;
                    // 反射率转为强度值
                    p.intensity = static_cast<float>(livox_point.reflectivity);
                    
                    cloud->push_back(p);
                }

                return cloud;
            }

            /**
             * @brief  点云坐标变换 + 多立方体区域点数量统计 + 逐帧累加区域内点云
             * @param raw        输入原始点云
             * @param T          4x4 坐标变换矩阵（点云左乘该矩阵）
             * @param areas      立方体区域数组
             * @return vector<int> 每个区域内的点数量，大小与 areas 一致
             * @note 新增逻辑：每一帧中落在任意立方体内的点，都会累加到 filter_cloud_
             */
            std::vector<int> countPointsInCuboids(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& raw,
                const Eigen::Matrix4d& T,
                const std::vector<cuboid>& areas
            )
            {
                // 1. PCL 内置函数：坐标变换（左乘矩阵T，核心要求）
                pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                pcl::transformPointCloud(*raw, *transformed_cloud, T);

                // 2. 初始化结果：长度和区域数组一致，默认值0
                std::vector<int> point_counts(areas.size(), 0);

                // 3. 遍历所有变换后的点
                for (const auto& p : transformed_cloud->points)
                {
                    // 过滤无效点（增强鲁棒性）
                    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                        continue;

                    bool is_in_any_area = false; // 标记当前点是否在任意目标区域内

                    // 4. 遍历所有区域，判断点是否在立方体内，并统计数量
                    for (size_t i = 0; i < areas.size(); ++i)
                    {
                        const cuboid& box = areas[i];
                        if (p.x >= box._x_min && p.x <= box._x_max &&
                            p.y >= box._y_min && p.y <= box._y_max &&
                            p.z >= box._z_min && p.z <= box._z_max)
                        {
                            point_counts[i]++;
                            is_in_any_area = true;
                        }
                    }

                    // ===================== 新增：区域内点 累加至全局点云 =====================
                    // 只要点在任意一个立方体内，就追加到累加点云（一帧内同一个点只存一次）
                    if (is_in_any_area)
                    {
                        filter_cloud_->push_back(p);
                    }
                }

                //*filter_cloud_ += *transformed_cloud;
                // 5. 返回每个区域的点数
                return point_counts;
            }
        };


        class counting_stars
        {
        public:
            counting_stars(size_t num_threshold = 100)
            :num_threshold_(num_threshold)
            {
                areas_.resize(9);

                const float x = 9.94;
                const float y = -6.025;
                const float z = 0.85;
                const float narrow = 0.05;
                const float depth_1_of_2 = 0.3 / 2;
                const float dy = 0.20;
                const float dx = 0.54;
                const float dz = 0.54;

                //初始化
                //列
                for(size_t j = 0; j < 3; j++)
                {
                    //行
                    for(size_t i = 0; i < 3; i++)
                    {
                        cuboid area;
                        area.reset_segmentation(x+j*dx+narrow, x+(j+1)*dx-narrow, y-depth_1_of_2, y+depth_1_of_2+dy, z+i*dz+narrow, z+(i+1)*dz-narrow);                   
                        areas_[j + i*3] = area;
                    }
                }

                //打印
                for(size_t i = 0; i < areas_.size(); i++)
                {
                    std::cout << "num: " << i << std::endl;
                    areas_[i].print();
                }
            }

            std::vector<bool> get_exist()
            {
                // 每次统计前建议清空历史累加点云（根据业务需求选择是否开启）
                // get_num_.clear();

                std::vector<int> cloud_num = get_num_.get_hope_cloud_num(areas_);
                for(size_t i = 0; i < cloud_num.size(); i++)
                {
                    std::cout << "i: " << i << " cloud_num: " << cloud_num[i] << std::endl;
                }

                std::vector<bool> exist;
                exist.resize(9, 0);
                //填充
                for(size_t i = 0; i < cloud_num.size() && i < exist.size(); i++)
                {
                    if(cloud_num[i] > num_threshold_)
                    {
                        exist[i] = true;
                    }
                }
                return exist;
            }

            // 可选：对外暴露点云获取/清空接口，方便上层调用
            pcl::PointCloud<pcl::PointXYZI>::Ptr get_filter_cloud()
            {
                return get_num_.get_filter_cloud();
            }

            void clear_cloud()
            {
                get_num_.clear();
            }

        private:
            std::vector<cuboid> areas_;
            get_cloud get_num_;
            size_t num_threshold_;
        };
    }
}

#endif
