#ifndef __PNP_FUNC_H_
#define __PNP_FUNC_H_    // 头文件保护宏，防止重复包含

#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>

#include <pcl/ModelCoefficients.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/icp.h>
#include "pnp_param.h"
#include "../camera.h"
#include "../openvino.h"

namespace Ten
{
namespace KFS
{

// PnP算法输出结果结构体
struct kfsPnpOutput
{
  bool valid = false;                                   // 结果有效标志，true为有效
  std::string status;                                   // 状态信息，记录处理结果

  cv::Rect roi;                                          // 目标ROI矩形区域
  cv::Mat kfs_Mask;                                      // 目标掩码图像

  Eigen::Vector3f center = Eigen::Vector3f::Zero();      // 目标中心坐标
  Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();  // 目标旋转四元数

  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudRaw;         // 原始点云数据
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered;    // 滤波后点云数据
  std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> planeClouds;  // 平面点云集合
  std::vector<pcl::ModelCoefficients::Ptr> planeCoeffs; // 平面模型系数集合

  // 构造函数，初始化点云智能指针
  kfsPnpOutput()
      : cloudRaw(new pcl::PointCloud<pcl::PointXYZ>),
        cloudFiltered(new pcl::PointCloud<pcl::PointXYZ>)
  {
  }
};

// 平面数据结构体
struct planeData
{
  Eigen::Vector3f normal;
  Eigen::Vector3f bodyCenter;
  int pointsCount;
  size_t cloudIdx;
  float faceArea;
};

// PnP位姿求解器类，实现完整位姿解算流程
class kfsPnpSolver
{
public:
  // 构造函数
  explicit kfsPnpSolver(const kfsPnpConfig& cfg, const rs2_intrinsics& color_intr)
      : cfg_(cfg),
        inliers(new pcl::PointIndices),
        coeff(new pcl::ModelCoefficients),
        plane(new pcl::PointCloud<pcl::PointXYZ>),
        remain(new pcl::PointCloud<pcl::PointXYZ>)
        //detector("/home/h/下载/卷轴检测_han2/best","cpu",0,0.5,0.5)
  {
    color_intr_ = color_intr;
    // 覆盖相机内参参数
    cfg_.fx = color_intr.fx;
    cfg_.fy = color_intr.fy;
    cfg_.cx = color_intr.ppx;
    cfg_.cy = color_intr.ppy;
  }

  const kfsPnpConfig& config() const
  {
    return cfg_;
  }

  /*
   * @brief 执行完整位姿解算流程
   * @param colorBgr 输入彩色图像
   * @param depthU16 输入深度图像
   * @return 解算结果结构体
   */
  kfsPnpOutput process(
    const cv::Mat& colorBgr, 
    const cv::Mat& depth_frame
)
  {
    // 初始化输出对象
    kfsPnpOutput out;

    // 判断输入图像是否为空 || 验证智能指针本身非空 || 验证 RealSense 深度帧对象有效
    if (colorBgr.empty() || depth_frame.empty() )
    {
      out.status = "colorBgr.empty() || depth_frame.empty()";
      return out;
    }

    // // 提取红色目标ROI区域
    out.roi = getRedRoi(colorBgr, out.kfs_Mask);

    // out.roi = test_yolo2(colorBgr);

    // // 判断ROI是否有效
    if (out.roi.area() <= 0)
    {
      out.status = "no red roi";
      return out;
    }
    
    // ROI区域转换为三维点云
    convertRoiToCloud(depth_frame, out.roi, out.kfs_Mask, out.cloudRaw);

    // 判断原始点云是否为空
    if (out.cloudRaw->empty() || out.cloudRaw->size() > 80000)
    {
      out.status = "out.cloudRaw->empty() || out.cloudRaw->size() > 80000";
      return out;
    }

    // set_Pcl_Cloud(depth_frame,color_intr,out.cloudRaw);
    std::cout << "out.cloudRaw.size(): " << out.cloudRaw->size() << std::endl;

    // 点云预处理（降采样 + 欧式聚类）
    voxel_Downsample(out.cloudRaw, out.cloudFiltered);
    euclidean_filter(out.cloudFiltered,out.cloudFiltered);
    std::cout << "out.cloudFiltered.size(): " << out.cloudFiltered->size() << std::endl;

    // 判断滤波后点云是否为空
    if (!out.cloudFiltered || out.cloudFiltered->empty())
    {
      out.status = "empty filtered cloud";
      return out;
    }

    // 提取多个平面特征
    extractMultiPlanes(out.cloudFiltered, &out.planeCoeffs, &out.planeClouds);

    // 判断平面提取是否成功
    if (out.planeCoeffs.empty())
    {
      out.status = "plane extract failed";
      return out;
    }

    // 平面解算目标位姿
    bool plane_ok = plane_filter(out.planeCoeffs, out.planeClouds, plane_info);
    if (plane_ok)
    {
      solvePoseFromPlanes(out.planeCoeffs, out.planeClouds, plane_info, &out.center, &out.orientation);
    }
    else
    {
      out.status = "plane filter failed";
      return out;
    }

    // 位姿初始化完成后执行ICP精配准
    if (poseInitialized_)
    {
      Eigen::Affine3f T_refined = refineWithICP(out.cloudFiltered, out.center, out.orientation);
      out.center = T_refined.translation();
      out.orientation = Eigen::Quaternionf(T_refined.linear());

      // 保存上一帧点云和位姿
      if (!prevCloud_)
      {
        prevCloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
      }
      *prevCloud_ = *out.cloudFiltered;
      prevPose_ = T_refined;
    }

    // 位姿滑动窗口平滑
    smoothPose(&out.center, &out.orientation);

    // 设置结果有效标志
    out.valid = true;
    out.status = "ok";
    return out;
  }

  // 设置roi接口
  void set_roi(const cv::Rect& roi, kfsPnpOutput& out)
  {
    if (roi.area() <= 0)
    {
        out.roi = cv::Rect(); // 赋值为空矩形，保持干净
        return;
    }
    out.roi = roi;
  }

private:

  // cv::Rect test_yolo2(const cv::Mat &img)
  // {
  //     // 调用worker函数
  //     cv::Mat image = img.clone();
  //     std::vector<Ten::Detection> results = detector.worker(image);

  //     if(results.size() == 0)
  //     {
  //       return cv::Rect();
  //     }

  //     // 遍历框，计算其中面积最大的
  //     // Ten::Detection best;
  //     // double max_square = 0;
  //     // for (int i = 0; i < results.size(); i++)
  //     // {
  //     //     double square = results[i].w_ * results[i].h_;
  //     //     if (square > max_square)
  //     //     {
  //     //         best = results[i];
  //     //         max_square = square;
  //     //     }
  //     // }
  //     std::sort(results.begin(), results.end(),
  //               [](const Ten::Detection &det1, const Ten::Detection &det2) -> bool
  //               {
  //                   double s1 = det1.w_ * det1.h_;
  //                   double s2 = det2.w_ * det2.h_;
  //                   return s1 > s2;
  //               });
  //     Ten::Detection best = results[0];

  //     // 归一化
  //     float x1 = best.cx_ - best.w_ / 2;
  //     float x2 = best.cx_ + best.w_ / 2;
  //     float y1 = best.cy_ - best.h_ / 2;
  //     float y2 = best.cy_ + best.h_ / 2;

  //     return cv::Rect(cv::Point2i(x1, y1), cv::Point2i(x2, y2));
  // }


  // 提取红色目标ROI区域
  cv::Rect getRedRoi(const cv::Mat& src, cv::Mat& outMask) const
  {
    // 图像转换为LAB颜色空间
    cv::Mat lab;
    cv::cvtColor(src, lab, cv::COLOR_BGR2Lab);

    // 颜色阈值提取红色区域
    // cv::inRange(lab,
    //             cv::Scalar(cfg_.red.labLMin, cfg_.red.labAMin, cfg_.red.labBMin),
    //             cv::Scalar(cfg_.red.labLMax, cfg_.red.labAMax, cfg_.red.labBMax),
    //             outMask);

    cv::inRange(lab,
                cv::Scalar(cfg_.blue.labLMin, cfg_.blue.labAMin, cfg_.blue.labBMin),
                cv::Scalar(cfg_.blue.labLMax, cfg_.blue.labAMax, cfg_.blue.labBMax),
                outMask);
    // 创建形态学卷积核
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

    // 开运算去除噪点
    cv::morphologyEx(outMask, outMask, cv::MORPH_OPEN, kernel);

    // 闭运算填充空洞
    cv::morphologyEx(outMask, outMask, cv::MORPH_CLOSE, kernel);

    // // 计算图像中心点坐标
    // int cx = src.cols / 2;
    // int cy = src.rows / 2;

    // // 获取中心点的 Lab 像素值 (OpenCV 中 Lab 存储为 0~255)
    // cv::Vec3b center_lab = lab.at<cv::Vec3b>(cy, cx);
    // int L = center_lab[0];  // L通道
    // int a = center_lab[1];  // a通道
    // int b = center_lab[2];  // b通道
    // std::cout << "L: " << L << ",a: " << a << ",b: " << b << std::endl;

    // 提取图像轮廓
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(outMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // 判断轮廓是否为空
    if (contours.empty())
    {
      return cv::Rect();
    }

    // 计算图像最大允许面积
    const double imageArea = static_cast<double>(src.cols * src.rows);
    const double maxAllowedArea = cfg_.roiMaxAreaRatio * imageArea;

    // 遍历轮廓筛选最优目标
    double bestArea = 0.0;
    int bestIdx = -1;
    for (size_t i = 0; i < contours.size(); ++i)
    {
      const double area = cv::contourArea(contours[i]);
      if (area < cfg_.roiMinArea || area > maxAllowedArea)
      {
        continue;
      }
      if (area > bestArea)
      {
        bestArea = area;
        bestIdx = static_cast<int>(i);
      }
    }

    // 判断是否找到有效轮廓
    if (bestIdx < 0)
    {
      return cv::Rect();
    }

    // 计算最优轮廓外接矩形并扩展
    cv::Rect rect = cv::boundingRect(contours[static_cast<size_t>(bestIdx)]);
    rect.x = std::max(0, rect.x);
    rect.y = std::max(0, rect.y);
    rect.width = std::min(src.cols - rect.x, rect.width);
    rect.height = std::min(src.rows - rect.y, rect.height);
    return rect;
  }

  // ROI区域转换为三维点云
  void convertRoiToCloud(const cv::Mat& depth_frame,
                                const cv::Rect& roi,
                                const cv::Mat& target_mask,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) const
  {
    // 清空并预分配点云内存
    cloud->clear();
    int w = depth_frame.cols;
    int h = depth_frame.rows;

    // 遍历ROI区域像素
    const int start_v = std::max(0, roi.y - cfg_.roi_padding);
    const int end_v   = std::min(h, roi.y + roi.height + cfg_.roi_padding);
    const int start_u = std::max(0, roi.x - cfg_.roi_padding);
    const int end_u   = std::min(w, roi.x + roi.width + cfg_.roi_padding);

    for (int v = start_v; v < end_v; v++)
    {
      for (int u = start_u; u < end_u; u++)
      {
        // 跳过非方块区域
        // if (target_mask.at<uint8_t>(v, u) == 0) continue;

        // 获取深度值
        float z = depth_frame.ptr<uint16_t>(v)[u] * 0.001f;
        int z_mm = int(z * 1000);

        // 过滤无效深度
        if (z <= 0 || z_mm < cfg_.CloudDepth_min || z_mm > cfg_.CloudDepth_max)
            continue;

        // 反投影
        float pixel[2] = {(float)u, (float)v};
        float point3d[3] = {0};
        rs2_deproject_pixel_to_point(point3d, &color_intr_, pixel, z);
        
        pcl::PointXYZ p;
        p.x = point3d[0];
        p.y = point3d[1];
        p.z = point3d[2];
        cloud->push_back(p);
      }
    }
  }

  /**
   * @brief 根据深度图像， 内参， 直接转成pcl点云
   * @param depth_frame       原生深度帧
   * @param color_intr        彩色相机内参
   * @param pcl_cloud         输出的pcl_cloud点云
  */
  void set_Pcl_Cloud(
    const cv::Mat& depth_frame,
    const rs2_intrinsics& color_intr,
    pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud
  )
  {
    pcl_cloud->clear();
    int w = depth_frame.cols;
    int h = depth_frame.rows;

    for (int v = 0; v < h; v++) {
      for (int u = 0; u < w; u++) {
        // 原生精准深度（0.1mm精度，斜视角无误差）
        float z = depth_frame.ptr<uint16_t>(v)[u] * 0.001f;
        int z_mm = int(z * 1000);

        // 过滤无效深度
        if (z <= 0 || z_mm < cfg_.CloudDepth_min || z_mm > cfg_.CloudDepth_max)
          continue;

        // 反投影（align后用彩色内参，坐标系100%对齐）
        float pixel[2] = {(float)u, (float)v};
        float point3d[3] = {0};
        rs2_deproject_pixel_to_point(point3d, &color_intr, pixel, z);
        
        pcl::PointXYZ p;
        p.x = point3d[0];
        p.y = point3d[1];
        p.z = point3d[2];
        pcl_cloud->push_back(p);
        }
    }
  }

  // 点云预处理: 降采样
  void voxel_Downsample(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud
  )
  {
      // 判断点云是否为空
      if (input_cloud->empty())
      {
          return;
      }

      // 初始化并配置体素滤波器
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud(input_cloud);
      vg.setLeafSize(cfg_.voxelLeaf, cfg_.voxelLeaf, cfg_.voxelLeaf);
      vg.filter(*output_cloud);
  }

  void euclidean_filter(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud,
      pcl::PointCloud<pcl::PointXYZ>::Ptr& output_cloud)
  {
      // 执行欧式聚类
      std::vector<pcl::PointIndices> cluster_indices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setInputCloud(input_cloud);
      ec.setClusterTolerance(cfg_.ClusterTolerance);
      ec.extract(cluster_indices);

      // 提取主聚类点云
      if (cluster_indices.empty())
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

  void extractMultiPlanes(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudIn,
      std::vector<pcl::ModelCoefficients::Ptr>* planeCoeffs,
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>* planeClouds)
  {
    // 清空输出容器
    planeCoeffs->clear();
    planeClouds->clear();

    // 复制输入点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudWork(new pcl::PointCloud<pcl::PointXYZ>(*cloudIn));

    // 初始化RANSAC平面分割器
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(cfg_.ransacDist);
    seg.setMaxIterations(cfg_.ransacIter);

    // 循环提取平面
    size_t firstPlaneSize = 0;
    for (int i = 0; i < cfg_.maxPlanes; ++i)
    {
      if (static_cast<int>(cloudWork->size()) < cfg_.minPlanePoints)
      {
        break;
      }

      inliers->indices.clear();
      coeff->values.clear();

      // 执行平面分割
      seg.setInputCloud(cloudWork);
      seg.segment(*inliers, *coeff);
      if (inliers->indices.empty() || coeff->values.size() < 4)
      {
        break;
      }

      // 记录第一平面大小，过滤过小辅助平面
      if (i == 0)
      {
        firstPlaneSize = inliers->indices.size();
      }
      else if (inliers->indices.size() < firstPlaneSize * cfg_.minSecondPlaneRatio)
      {
        break;
      }

      // 提取平面内点
      extract.setInputCloud(cloudWork);
      extract.setIndices(inliers);

      // 保存当前平面点云
      extract.setNegative(false);
      plane->clear();
      extract.filter(*plane);

      // 法向量归一化，过滤重复平面
      Eigen::Vector3f curr(coeff->values[0], coeff->values[1], coeff->values[2]);
      curr.normalize();
      bool duplicate = false;
      for (const auto& c : *planeCoeffs)
      {
        Eigen::Vector3f ex(c->values[0], c->values[1], c->values[2]);
        ex.normalize();
        if (std::abs(curr.dot(ex)) > cfg_.duplicateNormalDot)
        {
          duplicate = true;
          break;
        }
      }

      // 保存有效平面
      if (!duplicate)
      {
        // 深拷贝系数
        pcl::ModelCoefficients::Ptr coeff_copy(new pcl::ModelCoefficients(*coeff));
        planeCoeffs->push_back(coeff_copy);

        // 深拷贝平面点云
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane_copy(new pcl::PointCloud<pcl::PointXYZ>(*plane));
        planeClouds->push_back(plane_copy);
      }

      // 移除已提取平面，保留剩余点云
      extract.setNegative(true);
      remain->clear();
      extract.filter(*remain);
      cloudWork.swap(remain);
    }
  }

  // 计算平面长轴方向
  Eigen::Vector3f computeLongAxis(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
      const Eigen::Vector3f& zAxis) const
  {
    // 计算平面正交基向量
    Eigen::Vector3f u = zAxis.cross(Eigen::Vector3f::UnitY());
    if (u.norm() < 0.1f)
    {
      u = zAxis.cross(Eigen::Vector3f::UnitX());
    }
    u.normalize();
    const Eigen::Vector3f v = zAxis.cross(u).normalized();

    // 计算点云质心
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);
    const Eigen::Vector3f origin = centroid.head<3>();

    // 点云投影到二维平面
    std::vector<cv::Point2f> projected;
    projected.reserve(cloud->size());
    for (const auto& p : cloud->points)
    {
      const Eigen::Vector3f diff(p.x - origin.x(), p.y - origin.y(), p.z - origin.z());
      projected.emplace_back(diff.dot(u), diff.dot(v));
    }

    // 判断投影点是否为空
    if (projected.empty())
    {
      return u;
    }

    // 计算最小外接矩形，获取长轴方向
    const cv::RotatedRect rect = cv::minAreaRect(projected);
    cv::Point2f verts[4];
    rect.points(verts);
    const cv::Point2f e1 = verts[1] - verts[0];
    const cv::Point2f e2 = verts[2] - verts[1];
    Eigen::Vector3f dir1 = (e1.x * u + e1.y * v).normalized();
    Eigen::Vector3f dir2 = (e2.x * u + e2.y * v).normalized();

    // 返回最长轴方向
    return (cv::norm(e1) > cv::norm(e2)) ? dir1 : dir2;
  }

  // 多平面筛选， 修改并填充coeffs, clouds, planes
  bool plane_filter(
      std::vector<pcl::ModelCoefficients::Ptr>& coeffs,
      std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds,
      std::vector<planeData>& planes
  )
  {
    planes.clear();
    std::vector<pcl::ModelCoefficients::Ptr> new_coeffs;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> new_clouds;

    // 遍历平面，计算平面参数
    std::vector<int> valid_indices;         // 记录【合格平面的原始索引】
    for (size_t i = 0; i < coeffs.size(); ++i)
    {
      if (!clouds[i] || clouds[i]->empty())
      {
        continue;
      }

      // 法向量计算与归一化
      Eigen::Vector3f n(coeffs[i]->values[0], coeffs[i]->values[1], coeffs[i]->values[2]);
      n.normalize();

      // 计算点云质心，调整法向量方向
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*clouds[i], centroid);
      const Eigen::Vector3f viewVec = -centroid.head<3>();
      if (n.dot(viewVec) < 0.0f)
      {
        n = -n;
      }

      // 计算平面正交基
      Eigen::Vector3f u = n.cross(Eigen::Vector3f::UnitY());
      if (u.norm() < 0.1f)
      {
        u = n.cross(Eigen::Vector3f::UnitX());
      }
      u.normalize();
      const Eigen::Vector3f v = n.cross(u).normalized();

      // 点云投影到二维平面
      const Eigen::Vector3f origin = centroid.head<3>();
      std::vector<cv::Point2f> projected;
      projected.reserve(clouds[i]->size());
      for (const auto& p : clouds[i]->points)
      {
        const Eigen::Vector3f diff(p.x - origin.x(), p.y - origin.y(), p.z - origin.z());
        projected.emplace_back(diff.dot(u), diff.dot(v));
      }

      // 判断投影点是否为空
      if (projected.empty())
      {
        continue;
      }

      // 计算目标中心和面积
      const cv::RotatedRect rect = cv::minAreaRect(projected);
      const Eigen::Vector3f faceCenter = origin + rect.center.x * u + rect.center.y * v;
      const Eigen::Vector3f bodyCenter = faceCenter - n * static_cast<float>(cfg_.objectSize * 0.5);
      const float faceArea = rect.size.width * rect.size.height;
      // std::cout << "faceArea: " << faceArea << std::endl;
      std::cout << "width: " << rect.size.width << ",height: " << rect.size.height << std::endl;
 
       // 保存平面数据
      if (std::abs(rect.size.width - cfg_.objectSize) < cfg_.size_min_bias
          && std::abs(rect.size.height - cfg_.objectSize) < cfg_.size_min_bias)
      {
        planes.push_back({n, bodyCenter, static_cast<int>(clouds[i]->size()), planes.size(), faceArea});
        valid_indices.push_back(i);  // 把合格的索引存下来
        // std::cout << "planes: push_back" << std::endl;
      }
    }

    for (int idx : valid_indices)
    {
      new_coeffs.push_back(coeffs[idx]);
      new_clouds.push_back(clouds[idx]);
    }
    // 用合格平面替换原始容器 → 不合格的彻底删除
    coeffs.swap(new_coeffs);
    clouds.swap(new_clouds);

    // 判断平面数据是否为空
    if (planes.empty())
    {
      return false;
    }
    else{return true;}
  }

  // 平面融合解算目标位姿
  void solvePoseFromPlanes(
      const std::vector<pcl::ModelCoefficients::Ptr>& coeffs,
      const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& clouds,
      std::vector<planeData> planes,
      Eigen::Vector3f* center,
      Eigen::Quaternionf* orientation)
  {
    // 首帧质量门控与平面匹配
    if (!poseInitialized_)
    {
      // 选择最优主跟踪平面
      Eigen::Vector3f camZ(0, 0, 1);
      size_t bestIdx = 0;
      float bestAbsDot = -1.0f;
      for (size_t i = 0; i < planes.size(); ++i)
      {
        float absDot = std::abs(planes[i].normal.dot(camZ));
        if (absDot > bestAbsDot)
        {
          bestAbsDot = absDot;
          bestIdx = i;
        }
      }

      // 交换最优平面到首位
      if (bestIdx != 0)
      {
        std::swap(planes[0], planes[bestIdx]);
      }

      // 首帧质量校验
      if (planes[0].pointsCount >= cfg_.minFirstFramePoints &&
          planes[0].faceArea >= cfg_.minFirstFaceArea)
      {
        poseInitialized_ = true;
      }
    }
    else
    {
      // 帧间平面匹配
      std::vector<int> assignment(prevPlaneNormals_.size(), -1);
      std::vector<bool> used(planes.size(), false);

      // 遍历匹配平面
      for (size_t prevI = 0; prevI < prevPlaneNormals_.size(); ++prevI)
      {
        float bestMatch = -2.0f;
        int bestJ = -1;
        for (size_t j = 0; j < planes.size(); ++j)
        {
          if (used[j])
          {
            continue;
          }
          float dot = planes[j].normal.dot(prevPlaneNormals_[prevI]);
          if (dot > bestMatch)
          {
            bestMatch = dot;
            bestJ = static_cast<int>(j);
          }
        }

        // 保存有效匹配
        if (bestJ >= 0 && bestMatch > 0.5f)
        {
          assignment[prevI] = bestJ;
          used[bestJ] = true;
        }
      }

      // 重新排序平面
      std::vector<planeData> reordered;
      for (size_t i = 0; i < assignment.size(); ++i)
      {
        if (assignment[i] >= 0)
        {
          reordered.push_back(planes[static_cast<size_t>(assignment[i])]);
        }
      }
      for (size_t j = 0; j < planes.size(); ++j)
      {
        if (!used[j])
        {
          reordered.push_back(planes[j]);
        }
      }

      // 更新平面数据
      if (!reordered.empty())
      {
        planes = std::move(reordered);
      }
    }

    // 保存当前帧平面法向量
    prevPlaneNormals_.clear();
    for (const auto& pd : planes)
    {
      prevPlaneNormals_.push_back(pd.normal);
    }

    // 加权融合目标中心
    Eigen::Vector3f fused = Eigen::Vector3f::Zero();
    float totalW = 0.0f;
    for (const auto& p : planes)
    {
      const float w = static_cast<float>(std::max(1, p.pointsCount));
      fused += p.bodyCenter * w;
      totalW += w;
    }
    *center = (totalW > 0.0f) ? (fused / totalW) : planes.front().bodyCenter;

    // 计算坐标系轴
    Eigen::Vector3f yAxis = planes.front().normal;
    Eigen::Vector3f xAxis;

    // 多平面时计算X轴
    if (planes.size() >= 2)
    {
      xAxis = planes[1].normal - planes[1].normal.dot(yAxis) * yAxis;
      if (xAxis.norm() < 1e-3f)
      {
        xAxis = computeLongAxis(clouds[planes[0].cloudIdx], yAxis);
      }
    }
    else
    {
      xAxis = computeLongAxis(clouds[planes[0].cloudIdx], yAxis);
    }

    // X轴防翻转处理
    if (poseInitialized_ && refXAxis_.norm() > 0.1f)
    {
      float dotX = xAxis.dot(refXAxis_);
      if (dotX < 0.0f)
      {
        xAxis = -xAxis;
        dotX = -dotX;
      }
      if (std::abs(dotX) < 0.7f)
      {
        Eigen::Vector3f refXProj = refXAxis_ - (refXAxis_.dot(yAxis)) * yAxis;
        if (refXProj.norm() > 0.01f)
        {
          xAxis = refXProj.normalized();
        }
      }
    }

    // 坐标系归一化
    xAxis = xAxis - xAxis.dot(yAxis) * yAxis;
    xAxis.normalize();
    const Eigen::Vector3f zAxis = xAxis.cross(yAxis).normalized();

    // 保存参考坐标系
    if (poseInitialized_)
    {
      refZAxis_ = yAxis;
      refXAxis_ = xAxis;
    }

    // 构建旋转矩阵并转换为四元数
    Eigen::Matrix3f rot;
    rot.col(0) = xAxis;
    rot.col(1) = yAxis;
    rot.col(2) = zAxis;
    *orientation = Eigen::Quaternionf(rot);
    orientation->normalize();
  }

  // ICP精配准优化位姿
  Eigen::Affine3f refineWithICP(
      const pcl::PointCloud<pcl::PointXYZ>::Ptr& currentCloud,
      const Eigen::Vector3f& initCenter,
      const Eigen::Quaternionf& initOrientation)
  {
    // 初始化初始位姿矩阵
    Eigen::Affine3f T_init = Eigen::Affine3f::Identity();
    T_init.translation() = initCenter;
    T_init.linear() = initOrientation.toRotationMatrix();

    // 无历史点云，直接返回初始位姿
    if (!prevCloud_ || prevCloud_->empty())
    {
      return T_init;
    }

    // 计算帧间运动
    Eigen::Affine3f T_motion = T_init * prevPose_.inverse();
    float trans_motion = T_motion.translation().norm();
    float rot_motion = Eigen::AngleAxisf(T_motion.linear()).angle();

    // 运动过小，跳过ICP
    if (trans_motion < 0.002f && rot_motion < 0.005f)
    {
      return T_init;
    }

    // ICP专用降采样
    pcl::VoxelGrid<pcl::PointXYZ> icpVoxel;
    icpVoxel.setLeafSize(0.02f, 0.02f, 0.02f);

    // 初始化采样点云
    pcl::PointCloud<pcl::PointXYZ>::Ptr srcSampled(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr tgtSampled(new pcl::PointCloud<pcl::PointXYZ>);

    // 历史点云变换
    pcl::PointCloud<pcl::PointXYZ>::Ptr alignedPrev(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*prevCloud_, *alignedPrev, T_motion);

    // 降采样处理
    icpVoxel.setInputCloud(alignedPrev);
    icpVoxel.filter(*srcSampled);
    icpVoxel.setInputCloud(currentCloud);
    icpVoxel.filter(*tgtSampled);

    // 点云数量不足，返回初始位姿
    if (srcSampled->size() < 20 || tgtSampled->size() < 20)
    {
      return T_init;
    }

    // 初始化ICP算法
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(srcSampled);
    icp.setInputTarget(tgtSampled);
    icp.setMaxCorrespondenceDistance(0.05);
    icp.setMaximumIterations(15);
    icp.setTransformationEpsilon(1e-4);
    icp.setEuclideanFitnessEpsilon(1e-4);

    // 执行ICP配准
    pcl::PointCloud<pcl::PointXYZ> aligned;
    icp.align(aligned);

    // 返回优化后位姿
    if (icp.hasConverged())
    {
      Eigen::Affine3f T_corr;
      T_corr.matrix() = icp.getFinalTransformation();
      return T_corr * T_init;
    }
    return T_init;
  }

  // 滑动窗口位姿平滑
  void smoothPose(Eigen::Vector3f* center, Eigen::Quaternionf* orientation)
  {
    // 获取平滑窗口大小
    const int window = std::max(1, cfg_.smoothWindow);

    // 保存当前中心坐标
    centerWindow_.push_back(*center);

    // 四元数符号统一
    if (!orientationWindow_.empty() && orientation->dot(orientationWindow_.back()) < 0.0f)
    {
      orientation->coeffs() = -orientation->coeffs();
    }
    orientationWindow_.push_back(*orientation);

    // 维护窗口大小
    if (static_cast<int>(centerWindow_.size()) > window)
    {
      centerWindow_.erase(centerWindow_.begin());
    }
    if (static_cast<int>(orientationWindow_.size()) > window)
    {
      orientationWindow_.erase(orientationWindow_.begin());
    }

    // 计算平均中心坐标
    Eigen::Vector3f avgCenter = Eigen::Vector3f::Zero();
    for (const auto& c : centerWindow_)
    {
      avgCenter += c;
    }
    avgCenter /= static_cast<float>(centerWindow_.size());
    *center = avgCenter;

    // 计算平均四元数
    Eigen::Vector4f avgCoeff = Eigen::Vector4f::Zero();
    for (const auto& q : orientationWindow_)
    {
      avgCoeff += q.coeffs();
    }
    Eigen::Quaternionf avgQ(avgCoeff);
    avgQ.normalize();
    *orientation = avgQ;
  }

private:
  kfsPnpConfig cfg_;                                          // 算法配置参数
  rs2_intrinsics color_intr_;
  std::vector<Eigen::Vector3f> centerWindow_;                // 位姿平滑窗口
  std::vector<Eigen::Quaternionf> orientationWindow_;         // 旋转平滑窗口

  pcl::ExtractIndices<pcl::PointXYZ> extract;               // 索引提取器
  pcl::PointIndices::Ptr inliers;                           // 平面内点索引
  pcl::ModelCoefficients::Ptr coeff;                        // 平面系数
  pcl::PointCloud<pcl::PointXYZ>::Ptr plane;                // 临时平面点云
  pcl::PointCloud<pcl::PointXYZ>::Ptr remain;               // 剩余点云

  std::vector<planeData> plane_info;

  bool poseInitialized_ = false;                              // 首帧初始化标志
  Eigen::Vector3f refZAxis_ = Eigen::Vector3f::Zero();       // 参考Z轴
  Eigen::Vector3f refXAxis_ = Eigen::Vector3f::Zero();       // 参考X轴
  std::vector<Eigen::Vector3f> prevPlaneNormals_;            // 上一帧平面法向量
  pcl::PointCloud<pcl::PointXYZ>::Ptr prevCloud_;            // 上一帧点云
  Eigen::Affine3f prevPose_ = Eigen::Affine3f::Identity();   // 上一帧位姿

  //Ten::Ten_yolo detector;
};

} // namespace KFS
} // namespace Ten
#endif // __KFS_PCL_PNP_FUNC_H_