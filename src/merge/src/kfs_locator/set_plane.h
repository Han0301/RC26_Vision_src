#ifndef __SET_PLANE_H_
#define __SET_PLANE_H_
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <Eigen/Dense>
#include <cmath>
#include "./../method_math.h"

namespace Ten::kfs_locator {
    constexpr float BOX_SIZE = 0.35;
    constexpr float leaf_size_XY = 0.006f;               // 体素滤波XY尺寸，值越大点云越稀疏
    constexpr float leaf_size_Z  = 0.010f;               // 体素滤波Z尺寸，值越大点云越稀疏
    constexpr float DistanceThreshold = 0.020f;          // 平面拟合距离阈值，值越大拟合范围越大
    constexpr int  MaxIterations = 500;                 // 平面拟合迭代次数，值越大精度越高
    constexpr float ClusterTolerance = 0.012;            // 欧式聚类容差，值越大聚类范围越大
    constexpr int   MinPlaneInliers = 80;               // RANSAC 平面拟合最小内点数，低于此值视为无效


    // 存储平面位姿信息
    struct Plane_Info
    {
        Eigen::Vector3d plane_center;        // 平面3D中心点坐标
        Eigen::Vector3d plane_normal;        // 平面单位法向量

        Plane_Info()
        {
            plane_center = Eigen::Vector3d::Zero();
            plane_normal = Eigen::Vector3d::UnitZ();
        }
    };

    class Ten_set_plane
    {
    public:

        // 点云组合滤波
        bool cloud_filter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pclclouds,
            pcl::PointCloud<pcl::PointXYZ>::Ptr& out_pclclouds
        );

        // 平面拟合与点云提取(只提取顶面)
        bool Plane_fitter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
            Plane_Info& plane_info
        );

        // 计算平面点云质心与初始姿态
        bool compute_Center(
            pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
            Plane_Info& plane_info
        );

        Eigen::Vector3d cal_base_point(const Plane_Info& plane_info);

    private:
        // 体素网格降采样
        void voxel_Downsample(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud
        );

        // 欧式聚类提取主点云
        void euclidean_filter(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
            pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud);

        // RANSAC算法拟合平面
        bool ransac_Plane_Segment(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
            pcl::PointIndices::Ptr& plane_inliers,
            Plane_Info& plane_info
        );

        // 提取平面内点云
        void extract_Plane_Cloud(
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
            const pcl::PointIndices::Ptr& plane_inliers,
            pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
            bool negative = false
        );
    }; // class Ten_set_plane


    inline bool Ten_set_plane::cloud_filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_pclclouds,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& out_pclclouds
    )
    {
        if(input_pclclouds->size() <= 50 || !input_pclclouds) return false;

        // 执行体素降采样
        pcl::PointCloud<pcl::PointXYZ>::Ptr mid_pclclouds(new pcl::PointCloud<pcl::PointXYZ>);
        voxel_Downsample(input_pclclouds, mid_pclclouds);
        euclidean_filter(mid_pclclouds,out_pclclouds);
        // 校验点云数量
        if(out_pclclouds->size() <= 50) return false;
        return true;
    }

    inline void Ten_set_plane::voxel_Downsample(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud
    )
    {
        // 初始化并配置体素滤波器
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(leaf_size_XY, leaf_size_XY, leaf_size_Z);
        vg.filter(*output_cloud);
    }

    inline void Ten_set_plane::euclidean_filter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
    {
        // 执行欧式聚类
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setInputCloud(input_cloud);
        ec.setClusterTolerance(ClusterTolerance);
        ec.extract(cluster_indices);

        // 提取主聚类点云
        if (cluster_indices.empty() || cluster_indices[0].indices.empty())
        {
            *output_cloud = *input_cloud;
            return;
        }
        output_cloud->clear();
        for (int idx : cluster_indices[0].indices)
        {
            output_cloud->push_back(input_cloud->points[idx]);
        }
    }

    inline bool Ten_set_plane::ransac_Plane_Segment(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointIndices::Ptr& plane_inliers,
        Plane_Info& plane_info
    )
    {
        // 初始化平面拟合参数
        pcl::ModelCoefficients::Ptr coeffs(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(DistanceThreshold);
        seg.setMaxIterations(MaxIterations);

        // 执行平面拟合
        seg.setInputCloud(input_cloud);
        seg.segment(*plane_inliers, *coeffs);

        // 计算并赋值平面法向量
        if(!plane_inliers->indices.empty())
        {
            // 校验内点数量，避免少量噪点被误认为平面
            if (plane_inliers->indices.size() < static_cast<size_t>(MinPlaneInliers))
            {
                plane_inliers->indices.clear();
                return false;
            }
            float a = coeffs->values[0];
            float b = coeffs->values[1];
            float c = coeffs->values[2];
            float norm = sqrt(a*a + b*b + c*c);
            plane_info.plane_normal = Eigen::Vector3d(a/norm, b/norm, c/norm);
        }

        return !plane_inliers->indices.empty();
    }

    inline void Ten_set_plane::extract_Plane_Cloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        const pcl::PointIndices::Ptr& plane_inliers,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
        bool negative
    )
    {
        // 初始化并配置点云提取器
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(input_cloud);
        extract.setIndices(plane_inliers);
        extract.setNegative(negative);
        extract.filter(*output_cloud);
    }

    inline bool Ten_set_plane::Plane_fitter(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
        pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud,
        Plane_Info& plane_info
    )
    {
        // 校验点云有效性
        if (input_cloud->empty() || input_cloud->size() < 50) return false;

        // 执行平面拟合
        pcl::PointIndices::Ptr plane_inliers(new pcl::PointIndices);
        if (!ransac_Plane_Segment(input_cloud, plane_inliers, plane_info))
        {
            std::cerr << "未检测到平面！" << std::endl;
            return false;
        }

        // 提取平面点云
        extract_Plane_Cloud(input_cloud, plane_inliers, output_cloud);

        return true;
    }


    inline bool Ten_set_plane::compute_Center(
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
        Plane_Info& plane_info)
    {
        // 计算点云质心
        if (!input_cloud || input_cloud->empty())
        {
            plane_info.plane_center = Eigen::Vector3d::Zero();
            return false;
        }
        Eigen::Vector4f centroid_float = Eigen::Vector4f::Zero();
        pcl::compute3DCentroid(*input_cloud, centroid_float);
        plane_info.plane_center = Eigen::Vector3d(centroid_float[0], centroid_float[1], centroid_float[2]);
        return true;
    }

    inline Eigen::Vector3d Ten_set_plane::cal_base_point(const Plane_Info& plane_info)
    {
        const double offset_distance = BOX_SIZE / 2.0;

        // 1. 获取平面法向量
        Eigen::Vector3d normal = plane_info.plane_normal;
        Eigen::Vector3d plane_center = plane_info.plane_center;

        // 2. 统一法向量方向：确保背离相机（指向物体内部）
        // 相机在原点，平面中心的位置向量指向远离相机的方向
        // 点积 < 0 说明法向量朝向相机，取反
        if (normal.dot(plane_center) < 0.0)
        {
            normal = -normal;
        }

        // 3. 沿法向量（远离相机、指向物体内部）偏移厚度
        Eigen::Vector3d body_center = plane_center + normal * offset_distance;
        return body_center;
    }

}
#endif