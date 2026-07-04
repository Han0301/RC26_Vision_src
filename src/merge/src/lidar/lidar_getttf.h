#ifndef __LIDAR_GETTF_H_
#define __LIDAR_GETTF_H_
#include"lidar_recognition.h"
// 半径离群点滤波核心头文件
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include "./../method_math.h"

namespace Ten
{

    namespace lidar
    {

        #define _rate_ 1.3
        #define _area_threshold_ 0.64
        #define _the_min_cloud_threshold_ 300
        #define _the_barrier_multiple_ 10

        struct rectangle
        {
            rectangle(float x_min = 0.0, float y_min = 0.0, float x_max = 0.0, float y_max = 0.0)
                :_x_min(x_min)
                ,_y_min(y_min)
                ,_x_max(x_max)
                ,_y_max(y_max)
                ,_rate(_rate_)
                ,_area_threshold(_area_threshold_)
            {}

            /**
             * @brief 重新设置矩形区间
             */
            void reset_segmentation(float x_min = 0.0, float y_min = 0.0, float x_max = 0.0, float y_max = 0.0)
            {
                _x_min = x_min;
                _y_min = y_min;
                _x_max = x_max;
                _y_max = y_max;
            }

            /**
             * @brief 自动设置分割区间
             */
            void find_the_max_circumscribed_rectangle(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
            {
                if(cloud->size() <= 0)
                {
                    std::cout << "cloud empty" << std::endl;
                    return;
                }
                _x_min = _x_max = cloud->points[0].x;
                _y_min = _y_max = cloud->points[0].y;
                for (size_t i = 0; i < cloud->size(); ++i) {
                    const auto& point = cloud->points[i];
                    if(_x_min > point.x)
                    {
                        _x_min = point.x;
                    }
                    if(_x_max < point.x)
                    {
                        _x_max = point.x;
                    }
                    if(_y_min > point.y)
                    {
                        _y_min = point.y;
                    }
                    if(_y_max < point.y)
                    {
                        _y_max = point.y;
                    }
                }

                float length = _x_max - _x_min;
                float width = _y_max - _y_min;
                if(length < 1e-6 || width < 1e-6)
                {
                    std::cout << "length < 1e-6 || width < 1e-6" << std::endl;
                    return;
                }
                //如果小于阈值——障碍物
                if(length * width < _area_threshold)
                {
                    _x_min = (_x_max + _x_min) / 2.0 - length * _rate / 2.0;
                    _x_max = (_x_max + _x_min) / 2.0 + length * _rate / 2.0;
                    _y_min = (_y_max + _y_min) / 2.0 - width * _rate / 2.0;
                    _y_max = (_y_max + _y_min) / 2.0 + width * _rate / 2.0;
                    _flag = 1;
                    _cloud_size = cloud->size();
                    std::cout << "length * width: " << length * width << std::endl;
                    std::cout << "_cloud_size: " << _cloud_size << std::endl;
                }
                else
                {
                    _cloud_size = cloud->size();
                    std::cout << "_cloud_size: " << _cloud_size << std::endl;
                }
            }

            float _x_min;
            float _y_min;
            float _x_max;
            float _y_max;
            float _rate;
            float _area_threshold;
            size_t _cloud_size = 0;
            int _flag = 0; //1 静态
        };

        class point_getcloud
        {
        public:
            point_getcloud(size_t static_object_size = 0)
            {
                _static_objects.resize(static_object_size);
                _filters.resize(static_object_size);
                //_points_sizes.resize(static_object_size,0);
            }

            /**
             * @brief 设置障碍物个数
             * @param static_object_size: 障碍物个数
             */
            void set_size_of_static_objects_and_filters(size_t static_object_size)
            {
                _static_objects.resize(static_object_size);
                _filters.resize(static_object_size);
                //_points_sizes.resize(static_object_size,0);
            }

            /**
             * @brief 设置障碍物区间
             * @param v_clouds: 所有有效聚类簇
             */
            void set_static_object(const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& v_clouds)
            {
                //设置障碍物个数
                set_size_of_static_objects_and_filters(v_clouds.size());

                if(_static_objects.size() != v_clouds.size())
                {
                    std::cout << "_static_object.size() != v_clouds.size()" << std::endl;
                    return;
                }
                for(size_t i = 0; i < v_clouds.size(); i++)
                {
                    _static_objects[i].find_the_max_circumscribed_rectangle(v_clouds[i]);
                }
                //重置滤波器
                reset_bounding_boxs();
            }

            pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud_of_static_area(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
            {
                //记录点的数量
                std::vector<size_t> points_sizes; 
                points_sizes.resize(_static_objects.size(), 0);
                // 初始化PCL点云对象
                pcl::PointCloud<pcl::PointXYZI>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZI>());
                // 预分配内存，提升转换效率
                filter_cloud->points.reserve(cloud->points.size());
                // 遍历所有激光点，完成数据转换
                for (size_t i = 0; i < cloud->points.size(); i++)
                {
                    auto point = cloud->points[i];

                    //过滤障碍物点云
                    int out_of_box = 1;
                    for(size_t j = 0; j < _filters.size(); j++)
                    {
                        //是障碍物
                        if(_static_objects[j]._flag == 1)
                        {
                            //如果不在外面
                            if(!_filters[j].outside_the_box(point.x, point.y, point.z))
                            {
                                out_of_box = 0;
                                points_sizes[j]++;
                            }
                        }
                    }
                    //在外面，插入
                    if(out_of_box)
                    {
                        filter_cloud->push_back(point);
                    }
                }

                //判断障碍物是否还存在
                for(size_t j = 0; j < _static_objects.size(); j++)
                {
                    //是障碍物
                    if(_static_objects[j]._flag == 1)
                    {
                        if(points_sizes[j] < _static_objects[j]._cloud_size / _the_barrier_multiple_)
                        {
                            _static_objects[j]._flag = 0;
                        }
                    }
                }
                return filter_cloud;
            }

        private:
            std::vector<rectangle> _static_objects;
            std::vector<bounding_box> _filters; 

            //重置滤波器
            void reset_bounding_boxs()
            {
                for(size_t i = 0; i < _static_objects.size(); i++)
                {
                    rectangle& rc = _static_objects[i];
                    //是障碍物
                    if(rc._flag == 1)
                    {
                        _filters[i].reset_internal_box(rc._x_min, rc._y_min, _internal_box_z_min_, rc._x_max, rc._y_max, _internal_box_z_max_);
                    }
                }
            }
        };

        class lidar_getttf
        {
        public:
            lidar_getttf()
            :_cloud_filter(new pcl::PointCloud<pcl::PointXYZI>)
            ,_nh("/")
            {
                _cloud_current_sub =  _nh.subscribe("/maple/point_cloud_current", 1, &lidar_getttf::pointCloudCallback, this);
                _cloud_filter_pub = _nh.advertise<sensor_msgs::PointCloud2>("/maple/point_cloud_filter", 10);
            }

            Ten::XYZRPY get_tf(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
            {
                Ten::XYZRPY car_pose;
                //初始化地图
                if(_flag == 0)
                {
                    if(cloud->size() < _the_min_cloud_threshold_)
                    {
                        std::cout << "cloud->size() < _the_min_cloud_threshold_: "<<  cloud->size() << std::endl;
                        return Ten::XYZRPY();
                    }
                    _flag = 1;
                    pcl::PointCloud<pcl::PointXYZI>::Ptr filter = radiusFilter(cloud);
                    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vc = euclideanClusterAll(filter);
                    _p_filter.set_static_object(vc);
                }

                //滤波
                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> vc = euclideanClusterAll(radiusFilter(_p_filter.filter_cloud_of_static_area(cloud)));
                if(vc.size() == 0 || vc[0]->size() < _the_min_cloud_threshold_ / 2)
                {
                    return Ten::XYZRPY();
                }
                _cloud_filter = vc[0];
                XYZ centroid = computeCloudCentroid(voxelFilter(_cloud_filter));
                car_pose._xyz = centroid;
                return car_pose;
            }

        private:
            pcl::PointCloud<pcl::PointXYZI>::Ptr _cloud_filter;
            mutable std::mutex mtx_;
            point_getcloud _p_filter;
            int _flag = 0; //是否初始化
            ros::NodeHandle _nh;
            tf2_ros::TransformBroadcaster _tf_broadcaster;
            ros::Subscriber _cloud_current_sub;
            ros::Publisher _cloud_filter_pub;


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

            // ==================== 核心函数：发布动态TF ====================
            /**
             * @brief 发布动态TF坐标变换
             * @param translation 平移向量（你的XYZ结构体）
             * @param broadcaster TF广播器（tf2_ros::TransformBroadcaster&，ROS标准TF发布器）
             * @param parent_frame 父坐标系，默认值: map
             * @param child_frame  子坐标系，默认值: base_link
             */
            void publishDynamicTF(
                const XYZ& translation,
                tf2_ros::TransformBroadcaster& broadcaster,
                const std::string& parent_frame = "test_lidar",
                const std::string& child_frame = "car"
            )
            {
                // 1. 创建TF消息
                geometry_msgs::TransformStamped tf_msg;

                // 2. 设置基础信息（默认值）
                tf_msg.header.stamp = ros::Time::now();    // 时间戳：当前时间（动态TF必备）
                tf_msg.header.frame_id = parent_frame;     // 父坐标系（默认map）
                tf_msg.child_frame_id = child_frame;        // 子坐标系（默认base_link）

                // 3. 设置平移：直接使用你的XYZ结构体
                tf_msg.transform.translation.x = translation._x;
                tf_msg.transform.translation.y = translation._y;
                tf_msg.transform.translation.z = translation._z;

                // 4. 设置旋转：默认值 0（无旋转，四元数 0,0,0,1）
                tf2::Quaternion q;
                q.setRPY(0, 0, 0);  // 横滚/俯仰/偏航 全部默认0
                tf_msg.transform.rotation.x = q.x();
                tf_msg.transform.rotation.y = q.y();
                tf_msg.transform.rotation.z = q.z();
                tf_msg.transform.rotation.w = q.w();

                // 5. 发布TF
                broadcaster.sendTransform(tf_msg);
            }

            /**
             * @brief 订阅PointCloud2的回调函数
             * @param cloud_msg 输入的ROS PointCloud2常量指针
             */
            void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
            {
                // ===================== 核心实现 =====================
                // 1. 初始化空的PCL XYZI点云智能指针
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyzi(new pcl::PointCloud<pcl::PointXYZI>());
                
                // 2. 将ROS PointCloud2消息 转换为 PCL PointXYZI点云
                //    自动匹配x/y/z/intensity字段
                pcl::fromROSMsg(*cloud_msg, *cloud_xyzi);
                
                //处理逻辑
                XYZ centroid = get_tf(cloud_xyzi)._xyz;
                publishPCLXYZItoROS(_cloud_filter, _cloud_filter_pub);
                publishDynamicTF(centroid, _tf_broadcaster);
            }

            /**
             * @brief 点云半径离群点移除滤波
             * @param cloud 输入：原始XYZI点云智能指针
             * @return 输出：滤波后的XYZI点云智能指针
             */
            pcl::PointCloud<pcl::PointXYZI>::Ptr radiusFilter(
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
            {
                // 1. 创建输出点云
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZI>());

                // 2. 初始化半径离群点滤波器
                pcl::RadiusOutlierRemoval<pcl::PointXYZI> ror;
                ror.setInputCloud(cloud);  // 设置输入点云

                // =====================滤波核心参数（根据场景调整） =====================
                ror.setRadiusSearch(0.35);        // 搜索半径：单位米，搜索该半径内的邻域点
                ror.setMinNeighborsInRadius(10);  // 最小邻域点数：小于该值的点判定为离群点并删除
                // ========================================================================

                // 3. 执行滤波
                ror.filter(*cloud_filtered);

                // 4. 返回滤波后的点云
                return cloud_filtered;
            }


            /**
             * @brief 欧几里得聚类分割
             * @param cloud 输入XYZI点云
             * @return std::vector 所有有效聚类簇，按点数【从大到小】排序
             */
            std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> euclideanClusterAll(
                pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
            {
                // 定义返回值：存储所有聚类簇的向量
                std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusters;

                // 空点云直接返回空向量
                if (cloud->empty())
                {
                    std::cout << "点云为空，聚类跳过" << std::endl;
                    return clusters;
                }

                // 1. 创建KdTree
                pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());
                tree->setInputCloud(cloud);

                // 2. 存储所有聚类的索引
                std::vector<pcl::PointIndices> cluster_indices;

                // 3. 初始化聚类器
                pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
                ec.setInputCloud(cloud);
                ec.setSearchMethod(tree);

                // ===================== 可调参数 =====================
                ec.setClusterTolerance(0.5);    // 聚类距离阈值(米)
                ec.setMinClusterSize(30);       // 最小聚类点数（过滤微小噪点）
                ec.setMaxClusterSize(999999);   // 最大聚类点数
                // ====================================================

                // 执行聚类
                ec.extract(cluster_indices);

                // 无有效聚类，直接返回
                if (cluster_indices.empty())
                    return clusters;

                // ===================== 核心：按点数从大到小排序 =====================
                std::sort(cluster_indices.begin(), cluster_indices.end(),
                    [](const pcl::PointIndices& a, const pcl::PointIndices& b) {
                        return a.indices.size() > b.indices.size();
                    });

                // ===================== 转换为点云并存储 =====================
                for (const auto& indices : cluster_indices)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>());
                    // 提取当前簇的所有点
                    for (int idx : indices.indices)
                    {
                        cluster->push_back(cloud->points[idx]);
                    }
                    // 设置点云格式
                    cluster->width = cluster->size();
                    cluster->height = 1;
                    cluster->is_dense = true;
                    clusters.push_back(cluster);
                }

                //std::cout << "聚类完成，有效簇数量: " << clusters.size() << std::endl;
                return clusters;
            }


            /**
             * @brief 体素下采样滤波函数（适配PointXYZI点云）
             * @param input_cloud 输入原始点云（智能指针）
             * @param leaf_size 体素网格大小（单位：米，默认0.1m）
             * @return 滤波后的点云（智能指针）
             */
            pcl::PointCloud<pcl::PointXYZI>::Ptr voxelFilter(
                const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
                float leaf_size = 0.2f)
            {
                // 1. 安全校验：输入点云为空，直接返回空指针
                if (!input_cloud || input_cloud->empty())
                {
                    std::cerr << "错误：输入点云为空！" << std::endl;
                    return pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
                }

                // 2. 创建滤波后输出点云
                pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

                // 3. 体素滤波核心代码
                pcl::VoxelGrid<pcl::PointXYZI> vg;
                vg.setInputCloud(input_cloud);   // 设置输入点云
                vg.setLeafSize(leaf_size, leaf_size, leaf_size); // 统一设置XYZ方向体素大小
                vg.filter(*output_cloud);        // 执行滤波

                // 4. 返回滤波后的点云
                return output_cloud;
            }

            /**
             * @brief 计算PointXYZI点云的质心
             * @param input_cloud 输入点云智能指针
             * @return 点云质心（XYZ结构体）
             */
            XYZ computeCloudCentroid(const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud)
            {
                XYZ centroid; // 初始化质心

                // 安全校验：空指针 或 空点云，直接返回默认质心(0,0,0)
                if (!input_cloud || input_cloud->empty())
                {
                    std::cerr << "警告：点云为空，返回默认质心 (0,0,0)！" << std::endl;
                    return centroid;
                }

                // 累加所有点的坐标（用double避免精度丢失）
                double sum_x = 0.0, sum_y = 0.0, sum_z = 0.0;
                size_t point_num = input_cloud->size(); // 总点数

                // 遍历所有点，累加x/y/z
                for (const auto& point : *input_cloud)
                {
                    sum_x += point.x;
                    sum_y += point.y;
                    sum_z += point.z;
                }

                // 计算平均值（质心）
                centroid._x = sum_x / point_num;
                centroid._y = sum_y / point_num;
                centroid._z = sum_z / point_num;

                return centroid;
            }

        };

    }

}

#endif