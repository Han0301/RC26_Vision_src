#ifndef __RELOCATION_H_
#define __RELOCATION_H_
#include <iostream>
#include <vector>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/correspondence_estimation.h>
#include <teaser/registration.h>
#include <teaser/matcher.h>
#include <ros/ros.h>
#include "method_math.h"
#include "lidar.h"
#include <unistd.h>
#include <sensor_msgs/PointCloud2.h>
#include <Eigen/Dense>
#include <cmath>
#include <pcl/registration/icp.h>

namespace Ten
{

template<typename PointT>
class Ten_relocation
{
    //using PointT = pcl::PointXYZINormal;
    using PointCloudT = pcl::PointCloud<PointT>;
    using NormalCloudT = pcl::PointCloud<pcl::Normal>;
    using FPFHCloudT = pcl::PointCloud<pcl::FPFHSignature33>;
public:
    Ten_relocation(const Ten_relocation& rel) = delete;
    Ten_relocation& operator=(const Ten_relocation& rel) = delete;

    /** 
        @brief 初始化重定位
        @param path: 全局地图点云pcd文件路径
    */
    Ten_relocation(std::string path)
    :global_cloud_(new PointCloudT),
    local_cloud_(new PointCloudT)
    {
        flag_ = readPCD(path, global_cloud_);
        if(flag_ != 0)
        {
            if(global_cloud_->points.size() == 0)
            {
                flag_ = 0;
            }
        }
    }


    /**
     * @brief 获取位姿变换信息，使用前要urcu_memb_register_thread();
     * @return Ten::XYZRPY : 世界2到当前世界坐标系的坐标变换
     */
    Ten::XYZRPY get_transformation()
    {
        // typename PointCloudT::Ptr local_cloud;
        // int lock = 0;
        // for(int i = 0; i < 10;)
        // {
        //     sensor_msgs::PointCloud2 map = Ten::_Map_GET_.read_data();
        //     local_cloud = Ten::sensor_msgs_PointCloud2topcltype<PointCloudT>(map);

        //     if(local_cloud == nullptr || local_cloud->points.size() == 0)
        //     {
        //         lock++;
        //         std::cout<< "local_cloud == nullptr  " << "local_cloud->points.size() == 0" << std::endl;
        //         usleep(100000);//sleep 0.1s
        //         if(lock >= 99)
        //         {
        //             flag_ = 0;
        //             break;
        //         }
        //         continue;
        //     }
        //     if(local_cloud->points.size() < 30000)
        //     {
        //         std::cout<< "local_cloud->points.size()" << local_cloud->points.size() << std::endl;
        //         i++;
        //         usleep(500000);//sleep 1s
        //         continue;
        //     }
        //     break;
        // }

        typename PointCloudT::Ptr local_cloud;
        sensor_msgs::PointCloud2 map = Ten::_Map_GET_.read_data();
        local_cloud = Ten::sensor_msgs_PointCloud2topcltype<PointCloudT>(map);   
        if(local_cloud == nullptr || local_cloud->points.size() <= 20000)
        {
            std::cout<< "local_cloud->points.size()" << local_cloud->points.size() << std::endl;
            return Ten::XYZRPY();
        }

        if(flag_ == 0)
        {
            return Ten::XYZRPY();
        }
        std::cout<< "local_cloud->points.size()" << local_cloud->points.size() << std::endl;
        return get_transformation(local_cloud);
    }

    /**
     * @brief 获取位姿变换信息，使用前要urcu_memb_register_thread();
     * @param local_cloud： 当前点云
     * @return Ten::XYZRPY : 世界2到当前世界坐标系的坐标变换
     */
    Ten::XYZRPY get_transformation(typename PointCloudT::Ptr local_cloud)
    {
        if(local_cloud->points.size() == 0 || flag_ == 0)
        {
            return Ten::XYZRPY();
        }
        local_cloud_ = local_cloud;

        
        //icp精配准
        // typename PointCloudT::Ptr local_cloud_raw(new PointCloudT);
        // pcl::transformPointCloud(*local_cloud, *local_cloud_raw, transform_matrix_raw);
        // Eigen::Matrix4d transform_matrix_icp = icpRegistration(local_cloud_raw, global_cloud_downsampled);
        // Eigen::Matrix4d transform_matrix_mix = transform_matrix_icp * transform_matrix_raw;
        //求逆矩阵
        //Eigen::Matrix4d inverse_transform = transform_matrix_mix.inverse();


        Eigen::Matrix4d transform_matrix_raw = process_teaser(global_cloud_, local_cloud_, 0.3);
        typename PointCloudT::Ptr local_cloud_raw(new PointCloudT);
        pcl::transformPointCloud(*local_cloud, *local_cloud_raw, transform_matrix_raw);
        // Eigen::Matrix4d transform_matrix_raw_2 = process_teaser(global_cloud_, local_cloud_raw, 0.3);
        // Eigen::Matrix4d transform_matrix_mix = transform_matrix_raw_2 * transform_matrix_raw;
        Eigen::Matrix4d transform_matrix_raw_2 = process_icp(global_cloud_, local_cloud_raw, 0.5);
        Eigen::Matrix4d transform_matrix_mix = transform_matrix_raw_2 * transform_matrix_raw;
        Eigen::Matrix4d inverse_transform = transform_matrix_mix.inverse();
        return Ten::transform_matrixtoXYZRPY(inverse_transform);
    }


private:



    /**
     * @brief teaser配准
     * @param input_cloud_target: 全局点云
     * @param input_cloud_source: 当前点云
     * @return Eigen::Matrix4d: 当前到全局的变化矩阵
     */
    Eigen::Matrix4d process_teaser(const typename PointCloudT::Ptr& input_cloud_target, typename PointCloudT::Ptr& input_cloud_source, float size)
    {
        // ===================== 3. 点云预处理（下采样） =====================
        typename PointCloudT::Ptr global_cloud_downsampled(new PointCloudT);
        typename PointCloudT::Ptr local_cloud_downsampled(new PointCloudT);
        voxelDownSample(input_cloud_target, global_cloud_downsampled, size); // 体素0.05m，可根据点云密度调整
        voxelDownSample(input_cloud_source, local_cloud_downsampled, size);

        // ===================== 4. 提取FPFH特征 =====================
        FPFHCloudT::Ptr global_fpfh(new FPFHCloudT);
        FPFHCloudT::Ptr local_fpfh(new FPFHCloudT);
        extractFPFHFeatures(global_cloud_downsampled, global_fpfh);
        extractFPFHFeatures(local_cloud_downsampled, local_fpfh);

        // ===================== 5. 特征匹配生成对应点对 =====================
        pcl::Correspondences correspondences;
        matchFeatures(local_cloud_downsampled, global_cloud_downsampled, local_fpfh, global_fpfh, correspondences);

        //转换对应点对为TEASER++所需格式（Eigen矩阵）
        Eigen::Matrix<double, 3, Eigen::Dynamic> src_points(3, correspondences.size());
        Eigen::Matrix<double, 3, Eigen::Dynamic> tgt_points(3, correspondences.size());

        if(correspondences.size() <= 50)
        {
            return Eigen::Matrix4d::Identity();
        }

        for (size_t i = 0; i < correspondences.size(); ++i) {
            const auto& corr = correspondences[i];
            // 源点（局部点云）
            src_points(0, i) = local_cloud_downsampled->points[corr.index_query].x;
            src_points(1, i) = local_cloud_downsampled->points[corr.index_query].y;
            src_points(2, i) = local_cloud_downsampled->points[corr.index_query].z;
            // 目标点（全局点云）
            tgt_points(0, i) = global_cloud_downsampled->points[corr.index_match].x;
            tgt_points(1, i) = global_cloud_downsampled->points[corr.index_match].y;
            tgt_points(2, i) = global_cloud_downsampled->points[corr.index_match].z;
        }

        // 7. 配置TEASER++配准求解器参数
        teaser::RobustRegistrationSolver::Params params;
        params.noise_bound = size;          // 噪声边界（与添加的噪声一致）
        params.cbar2 = 1;                          // GNC-TLS的正则化参数
        params.estimate_scaling = false;           // 不估计尺度（SE(3)变换无尺度）
        params.rotation_max_iterations = 200;      // 旋转估计最大迭代次数
        params.rotation_gnc_factor = 1.4;          // GNC-TLS的衰减因子
        params.rotation_estimation_algorithm = teaser::RobustRegistrationSolver::ROTATION_ESTIMATION_ALGORITHM::GNC_TLS; // 旋转估计算法选择GNC-TLS
        params.rotation_cost_threshold = 0.005;    // 旋转估计的成本阈值

        // 初始化TEASER++求解器
        teaser::RobustRegistrationSolver solver(params);
        solver.solve(src_points, tgt_points);

        // ===================== 7. 获取配准结果 =====================
        auto solution = solver.getSolution();
        Eigen::Matrix4d transform_matrix_raw = Eigen::Matrix4d::Identity();
        transform_matrix_raw.block<3, 3>(0, 0) = solution.rotation; // 旋转矩阵
        transform_matrix_raw.block<3, 1>(0, 3) = solution.translation; // 平移向量

        std::cout << "\n===================== TEASER++ 配准结果 =====================" << std::endl;
        std::cout << "旋转矩阵:\n" << solution.rotation << std::endl;
        std::cout << "平移向量 (x,y,z): " << solution.translation.transpose() << std::endl;
        std::cout << "变换矩阵:\n" << transform_matrix_raw << std::endl;

        return transform_matrix_raw;
    }



    /**
     * @brief 读取PCD点云文件
     * @param file_path PCD文件路径
     * @param cloud 输出点云
     * @return 读取成功返回true
     */
    bool readPCD(const std::string& file_path, typename PointCloudT::Ptr& cloud) {
        if (pcl::io::loadPCDFile<PointT>(file_path, *cloud) == -1) {
            //PCL_ERROR("无法读取PCD文件: %s\n", file_path.c_str());
            return false;
        }
        //std::cout << "读取点云 " << file_path << "，点数量: " << cloud->size() << std::endl;
        return true;
    }
    /**
     * @brief 点云下采样（体素滤波），减少计算量
     * @param input_cloud 输入点云
     * @param output_cloud 输出下采样点云
     * @param leaf_size 体素大小（单位：m）
     */
    void voxelDownSample(const typename PointCloudT::Ptr& input_cloud, typename PointCloudT::Ptr& output_cloud, float leaf_size = 0.1) 
    {
        pcl::VoxelGrid<PointT> vg;
        vg.setInputCloud(input_cloud);
        vg.setLeafSize(leaf_size, leaf_size, leaf_size);
        vg.filter(*output_cloud);
        std::cout << "下采样后点数量: " << output_cloud->size() << std::endl;
    }

    /**
     * @brief 提取FPFH特征（用于生成点云对应关系）
     * @param cloud 输入点云
     * @param fpfh_features 输出FPFH特征
     */
    void extractFPFHFeatures(const typename PointCloudT::Ptr& cloud, FPFHCloudT::Ptr& fpfh_features) {
        // 1. 计算法向量
        NormalCloudT::Ptr normals(new NormalCloudT);
        pcl::NormalEstimation<PointT, pcl::Normal> ne;
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
        ne.setInputCloud(cloud);
        ne.setSearchMethod(tree);
        ne.setKSearch(20); // 近邻数，可根据点云密度调整
        //ne.setRadiusSearch(5);
        ne.compute(*normals);

        // 2. 计算FPFH特征
        pcl::FPFHEstimation<PointT, pcl::Normal, pcl::FPFHSignature33> fpfh;
        fpfh.setInputCloud(cloud);
        fpfh.setInputNormals(normals);
        fpfh.setSearchMethod(tree);
        fpfh.setKSearch(50); // 特征计算近邻数
        fpfh.compute(*fpfh_features);
        std::cout << "FPFH特征提取完成，特征维度: " << fpfh_features->size() << std::endl;
    }

    /**
     * @brief 特征匹配生成点对（全局→局部）
     * @param src_cloud 源点云（局部点云）
     * @param tgt_cloud 目标点云（全局点云）
     * @param src_fpfh 源点云FPFH特征
     * @param tgt_fpfh 目标点云FPFH特征
     * @param correspondences 输出对应点对（src_idx → tgt_idx）
     */
    void matchFeatures(const typename PointCloudT::Ptr& src_cloud, const typename PointCloudT::Ptr& tgt_cloud,
    const FPFHCloudT::Ptr& src_fpfh, const FPFHCloudT::Ptr& tgt_fpfh, pcl::Correspondences& correspondences) 
    {
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> ce;
        ce.setInputSource(src_fpfh);
        ce.setInputTarget(tgt_fpfh);
        ce.determineCorrespondences(correspondences, 7); // 距离阈值，可调整
        std::cout << "特征匹配完成，生成对应点对数量: " << correspondences.size() << std::endl;
    }

    /**
     * @brief 简化版ICP配准函数（仅核心参数）
     * @param source_cloud      输入：源点云（需要配准的点云）
     * @param target_cloud      输入：目标点云（参考点云）
     * @param transformation    输出：配准得到的4x4刚体变换矩阵（旋转+平移）
     * @return bool             配准是否成功（收敛则返回true）
     */
    Eigen::Matrix4d process_icp( const typename PointCloudT::Ptr& target_cloud, const typename PointCloudT::Ptr& source_cloud, float size)
    {
        // 1. 输入合法性检查：源/目标点云不能为空
        if (source_cloud->empty() || target_cloud->empty()) {
            std::cerr << "[ERROR] 源点云或目标点云为空！" << std::endl;
            return Eigen::Matrix4d::Identity();
        }
        Eigen::Matrix4d transformation = Eigen::Matrix4d::Identity();


        // ===================== 3. 点云预处理（下采样） =====================
        typename PointCloudT::Ptr global_cloud_downsampled(new PointCloudT);
        typename PointCloudT::Ptr local_cloud_downsampled(new PointCloudT);
        voxelDownSample(target_cloud, global_cloud_downsampled, size); // 体素0.05m，可根据点云密度调整
        voxelDownSample(source_cloud, local_cloud_downsampled, size);

        // 2. 初始化ICP核心对象
        pcl::IterativeClosestPoint<PointT, PointT> icp;

        // 3. 设置ICP输入点云
        icp.setInputSource(local_cloud_downsampled);  // 待配准的源点云
        icp.setInputTarget(global_cloud_downsampled);  // 参考的目标点云

        // 4. 设置ICP默认核心参数（经典默认值，适配多数场景）
        icp.setMaximumIterations(100);                // 最大迭代次数
        icp.setTransformationEpsilon(1e-8);          // 收敛阈值（迭代停止条件）
        icp.setMaxCorrespondenceDistance(0.05);      // 最大对应点距离阈值（单位：m）
        icp.setEuclideanFitnessEpsilon(1e-6);        // 欧式适应度阈值（拟合精度）

        // 5. 执行ICP配准（无需保存配准后点云，仅执行配准计算）
        PointCloudT temp_cloud;  // 临时点云，仅用于执行align
        icp.align(temp_cloud);

        // 6. 检查ICP收敛状态
        if (!icp.hasConverged()) {
            std::cerr << "[WARNING] ICP配准未收敛！" << std::endl;
            return Eigen::Matrix4d::Identity();
        }

        // 7. 获取配准后的刚体变换矩阵（赋值给输出参数）
        //transformation = icp.getFinalTransformation().template cast<double>();
        Eigen::Matrix4f trans_float = icp.getFinalTransformation();
        transformation = trans_float.cast<double>();

        // 8. 输出基础配准信息（可选，可根据需求删除）
        std::cout << "[INFO] ICP配准完成：" << std::endl;
        std::cout << "  - 拟合分数（越小越好）：" << icp.getFitnessScore() << std::endl;
        std::cout << "  - 变换矩阵：\n" << transformation << std::endl;

        return transformation;
    }
    

typename PointCloudT::Ptr global_cloud_;
typename PointCloudT::Ptr local_cloud_;
int flag_ = 1;
};


}

















#endif
