#ifndef __MAPPING_H_
#define __MAPPING_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include "./../parameter/parameter.h"
#include "./../lidar.h"
#include <ros/callback_queue.h>

namespace Ten
{
class mapping
{
public:
    /**
     * @brief 构造函数：初始化后自动完成点云订阅与发布器注册
     * @param subscribe_topic  输入点云话题名
     * @param publish_topic    输出累积点云话题名
     * @param voxel_size       基础体素下采样分辨率，单位：米
     * @param max_points       累积点云最大点数上限，超限自动压缩
     * @param cluster_interval 欧式聚类间隔帧数，每N帧批量净化一次再并入总地图
     */
    mapping(const std::string& subscribe_topic = "/fast_lio2/cloud_registered",
            const std::string& publish_topic = "/relocation/mapping",
            float voxel_size = _voxeldownsample_threshold_,
            size_t max_points = 100000,
            size_t cluster_interval = 10)
    : accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
    , temp_accumulated_cloud_(new pcl::PointCloud<pcl::PointXYZI>)
    , voxel_leaf_size_(voxel_size)
    , max_accumulated_points_(max_points)
    , cluster_interval_(cluster_interval)
    , frame_counter_(0)
    {
        // 绑定私有回调队列，订阅者完全隔离于全局队列
        nh_.setCallbackQueue(&callback_queue_);

        // 订阅点云话题
        cloud_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>(subscribe_topic, 10, &mapping::pointCloudCallback, this);

        // 注册发布者
        accumulated_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(publish_topic, 10);

        // 预初始化复用的滤波器，避免每次构造析构开销
        voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);

        std::cout << "[mapping] 初始化完成" << std::endl
                  << "  订阅话题: " << subscribe_topic << std::endl
                  << "  发布话题: " << publish_topic << std::endl
                  << "  基础体素分辨率: " << voxel_size << "m" << std::endl
                  << "  最大点数上限: " << max_points << std::endl
                  << "  聚类清理间隔: 每" << cluster_interval << "帧批量净化并入图" << std::endl;
    }

    ~mapping() = default;

    /**
     * @brief 处理一次私有回调队列，需在线程循环中调用
     * @param timeout 队列等待超时时间，单位秒
     */
    void callback(double timeout = 0.03)
    {
        callback_queue_.callAvailable(ros::WallDuration(timeout));
    }

    /**
     * @brief 清空当前累积的点云地图与临时缓存
     */
    void clearMap()
    {
        accumulated_cloud_->clear();
        temp_accumulated_cloud_->clear();
        frame_counter_ = 0;
        std::cout << "[mapping] 累积点云已清空" << std::endl;
    }

private:
    /**
     * @brief 统计离群点滤波：去除单帧点云中的飞点、噪声离群点
     * @param input_cloud  输入原始单帧点云
     * @param mean_k       邻域采样点数量，默认50
     * @param std_dev_mul  标准差倍数阈值，默认1.0
     * @return 去噪后的点云
     */
    inline pcl::PointCloud<pcl::PointXYZI>::Ptr applyStatisticalFilter(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        int mean_k = 50,
        float std_dev_mul = 1.0f)
    {
        if (!input_cloud || input_cloud->empty())
        {
            return input_cloud;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor_filter;
        sor_filter.setInputCloud(input_cloud);
        sor_filter.setMeanK(mean_k);
        sor_filter.setStddevMulThresh(std_dev_mul);
        sor_filter.filter(*filtered_cloud);

        return filtered_cloud;
    }

    /**
     * @brief Z轴高度滤波：保留Z轴在[z_min, z_max]区间内的点，过滤区间外的点
     * @param input_cloud 输入点云
     * @param z_min       Z轴下限，小于该值的点被过滤，默认-100（基本不过滤下限）
     * @param z_max       Z轴上限，大于该值的点被过滤，默认3.0米
     * @return 高度过滤后的点云
     */
    inline pcl::PointCloud<pcl::PointXYZI>::Ptr applyZFilter(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        float z_min = -100.0f,
        float z_max = _max_z_)
    {
        if (!input_cloud || input_cloud->empty())
        {
            return input_cloud;
        }

        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        filtered_cloud->reserve(input_cloud->size());

        for (const auto& point : input_cloud->points)
        {
            if (point.z >= z_min && point.z <= z_max)
            {
                filtered_cloud->push_back(point);
            }
        }

        return filtered_cloud;
    }

    /**
     * @brief 欧式聚类去小簇：删除点数小于阈值的离散聚类，保留大结构点云
     * @param input_cloud        输入点云
     * @param cluster_tolerance  聚类距离容差，单位：米，小于该距离的点判定为同一簇
     * @param min_cluster_size   最小聚类点数，小于该值的簇会被整体删除
     * @return 净化后的点云
     */
    pcl::PointCloud<pcl::PointXYZI>::Ptr removeSmallClusters(
        const pcl::PointCloud<pcl::PointXYZI>::Ptr& input_cloud,
        float cluster_tolerance = _voxeldownsample_threshold_ + _voxeldownsample_threshold_*0.3,
        int min_cluster_size = 25.0 / _voxeldownsample_threshold_)
    {
        if (!input_cloud || input_cloud->empty())
        {
            return input_cloud;
        }

        // 构建KD树搜索器
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(input_cloud);

        // 欧式聚类提取
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec_extractor;
        ec_extractor.setInputCloud(input_cloud);
        ec_extractor.setSearchMethod(tree);
        ec_extractor.setClusterTolerance(cluster_tolerance);
        ec_extractor.setMinClusterSize(min_cluster_size);
        ec_extractor.setMaxClusterSize(input_cloud->size());
        ec_extractor.extract(cluster_indices);

        // 安全保护：无有效聚类时直接返回原输入，避免点云被清空
        if (cluster_indices.empty())
        {
            return input_cloud;
        }

        // 拼接所有符合要求的聚类点
        pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        output_cloud->reserve(input_cloud->size());
        for (const auto& indices : cluster_indices)
        {
            for (int idx : indices.indices)
            {
                output_cloud->push_back(input_cloud->points[idx]);
            }
        }

        return output_cloud;
    }

    /**
     * @brief 点云回调核心逻辑：单帧去噪 → 临时缓存 → 攒够N帧 → 聚类净化 → Z轴滤波 → 并入全局 → 体素压缩 → 发布
     */
    void pointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
    {
        // 1. ROS消息转 PCL PointXYZI 格式
        pcl::PointCloud<pcl::PointXYZI>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZI>);
        pcl::fromROSMsg(*cloud_msg, *current_cloud);

        if (!current_cloud || current_cloud->empty())
        {
            return;
        }

        // 2. 单帧入口去噪：先做统计滤波去除单帧飞点
        pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_current = applyStatisticalFilter(current_cloud);

        // 3. 加入N帧临时缓存，暂不并入全局地图
        *temp_accumulated_cloud_ += *filtered_current;
        frame_counter_++;

        // 4. 攒够指定帧数后，批量净化再并入全局
        if (frame_counter_ >= cluster_interval_)
        {
            // 4.1 对临时缓存的N帧点云做欧式聚类去小簇
            pcl::PointCloud<pcl::PointXYZI>::Ptr cleaned_temp = removeSmallClusters(temp_accumulated_cloud_);

            // 4.2 新增：Z轴高度滤波，过滤超出高度范围的点
            //cleaned_temp = applyZFilter(cleaned_temp);
            // 自定义参数示例：applyZFilter(cleaned_temp, -0.5f, 2.5f);

            // 4.3 净化后的点云并入全局总地图
            *accumulated_cloud_ += *cleaned_temp;

            // 4.4 清空临时缓存，重置计数
            temp_accumulated_cloud_->clear();
            frame_counter_ = 0;

            // 4.5 全局体素下采样（仅合并时执行，减少计算）
            pcl::PointCloud<pcl::PointXYZI>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            voxel_filter_.setInputCloud(accumulated_cloud_);
            voxel_filter_.filter(*downsampled_cloud);
            accumulated_cloud_ = downsampled_cloud;

            // ========== 点数超限压缩逻辑（带滞回区间，避免频繁触发） ==========
            constexpr float trigger_ratio = 1.1f;
            constexpr float target_ratio  = 0.9f;
            constexpr float max_voxel_scale = 3.0f;

            const size_t current_points = accumulated_cloud_->size();
            if (current_points > max_accumulated_points_ * trigger_ratio)
            {
                const size_t target_points = static_cast<size_t>(max_accumulated_points_ * target_ratio);
                pcl::PointCloud<pcl::PointXYZI>::Ptr compressed_cloud(new pcl::PointCloud<pcl::PointXYZI>);

                // 阶段1：优先通过放大体素均匀降采样
                const float point_ratio = static_cast<float>(current_points) / static_cast<float>(target_points);
                const float voxel_scale = std::pow(point_ratio, 1.0f / 3.0f);
                float target_voxel = voxel_leaf_size_ * voxel_scale;
                target_voxel = std::min(target_voxel, voxel_leaf_size_ * max_voxel_scale);

                pcl::VoxelGrid<pcl::PointXYZI> compress_voxel;
                compress_voxel.setInputCloud(accumulated_cloud_);
                compress_voxel.setLeafSize(target_voxel, target_voxel, target_voxel);
                compress_voxel.filter(*compressed_cloud);

                // 阶段2：体素达上限后仍超限，用均匀随机降采样兜底
                if (compressed_cloud->size() > target_points)
                {
                    pcl::PointCloud<pcl::PointXYZI>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZI>);
                    pcl::RandomSample<pcl::PointXYZI> random_sampler;
                    random_sampler.setInputCloud(compressed_cloud);
                    random_sampler.setSample(static_cast<unsigned int>(target_points));
                    random_sampler.filter(*sampled_cloud);
                    compressed_cloud = sampled_cloud;
                }

                accumulated_cloud_ = compressed_cloud;
            }

            // 5. 转为ROS消息并发布（仅合并更新时发布，降低下游开销）
            sensor_msgs::PointCloud2 output_msg;
            pcl::toROSMsg(*accumulated_cloud_, output_msg);
            output_msg.header = cloud_msg->header;
            _Map_GET2_.push(output_msg);
            accumulated_pub_.publish(output_msg);
        }
    }

    ros::NodeHandle nh_;
    ros::CallbackQueue callback_queue_;
    ros::Subscriber cloud_sub_;
    ros::Publisher accumulated_pub_;

    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated_cloud_;       // 全局总地图
    pcl::PointCloud<pcl::PointXYZI>::Ptr temp_accumulated_cloud_; // N帧临时缓存，用于批量聚类
    float voxel_leaf_size_;
    size_t max_accumulated_points_;
    pcl::VoxelGrid<pcl::PointXYZI> voxel_filter_;

    // 定期聚类参数
    size_t cluster_interval_;  // 批量聚类间隔帧数
    size_t frame_counter_;     // 帧计数器
};

} // namespace Ten

#endif // __MAPPING_H_
