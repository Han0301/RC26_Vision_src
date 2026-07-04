#ifndef __APRILTAG_POSE_MODULE_H_
#define __APRILTAG_POSE_MODULE_H_

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

extern "C" {
#include <apriltag/apriltag.h>
#include <apriltag/apriltag_pose.h>

#include <apriltag/tagStandard41h12.h>
}

/*
  author: zy

  AprilTag 位姿估计模块 (apriltag_pose_module.h)

  数据结构:
    CameraIntrinsics  — 相机内参 (fx,fy,cx,cy) + 畸变系数 + 内参矩阵 K
    TagPoseResult      — 单标签位姿 (id, err, R_3x3, t_3x1)
    FusedPoseResult    — 融合后位姿 (valid, id, err, R, t)
    FusionConfig       — 融合参数 (主/辅标签ID, 辅在主系中的位置, fusion_bias)
    FusionDebugInfo    — 调试输出 (是否检测到主/辅标签, 实测基线长度)

  类与接口:

  1. AprilTagPoseCore — AprilTag C 库封装
     构造: tag_size, quad_decimate, quad_sigma, nthreads, refine_edges
         ├─ 创建 tagStandard41h12 标签族
         ├─ 创建检测器, 设置降采样/高斯平滑/线程/边缘细化
         └─ 创建 CLAHE (1.2, 8x8) 自适应直方图均衡
     Detect(gray, intrinsics):
         BGR→灰度 → CLAHE增强 → apriltag_detector_detect() → estimate_tag_pose()
         逐标签: 复制 R/t/err/id → 释放 pose 内存 → 返回 vector<TagPoseResult>

  2. AprilTagDualTagFusion — 双标签融合位姿解算
     SelectBestTags(tags, &primary, &hasP, &aux, &hasA):
         遍历tags, 按 primary_id/aux_id 分别选 err 最小的样本
     SolvePrimaryPose(primary, aux):
         同时检测到两个标签时:
         ├─ baseline_vec = primary.t - aux.t  (id24→id7)
         ├─ 中心 = 两标签平移中点
         ├─ RotationFromBaseline():
         │   ├─ X轴 = normalize(baseline_vec)  [强约束]
         │   ├─ Z轴提示 = 双法向平均  [防翻转]
         │   ├─ Y轴 = Z × X,  Z = X × Y  [正交化]
         │   └─ 回退方案: 用主标签对应轴替换零向量
         └─ translation = 中点 + R * fusion_bias  [叠加融合系偏置]
         否则返回 invalid
*/

namespace apriltag_pose_module {

// 相机内参与畸变参数，用于位姿投影与可视化绘制。
struct CameraIntrinsics {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
  cv::Mat dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);

  /**
   * @brief 检查内参是否有效
   * @return 当焦距 fx/fy 均有效时返回 true
   */
  bool IsValid() const {
    return fx > 1e-9 && fy > 1e-9;
  }
};

// 单个检测到的标签位姿估计结果。
struct TagPoseResult {
  int id = -1;
  double err = 0.0; //这里的err是apriltag库输出的重投影误差，单位是像素。它反映了标签边缘点在图像中的重投影与实际检测到的位置之间的差异。较小的err通常表示更准确的位姿估计。
  cv::Matx33d rotation = cv::Matx33d::eye(); // 标签旋转矩阵，定义了标签坐标系相对于相机坐标系的旋转关系。
  cv::Vec3d translation = cv::Vec3d(0.0, 0.0, 0.0);// 标签坐标系定义为：以标签中心为原点，x 轴沿标签边缘方向，z 轴垂直于标签平面指向相机。
};

// 双标签融合后的主标签位姿结果。
struct FusedPoseResult {
  bool valid = false; 
  int id = -1;
  double err = 0.0; //这里的err是融合后的重投影误差，单位是像素。
  cv::Matx33d rotation = cv::Matx33d::eye(); // 融合后的标签旋转矩阵。
  cv::Vec3d translation = cv::Vec3d(0.0, 0.0, 0.0); // 融合后的标签平移向量。
};

// 双标签融合运行参数配置。
struct FusionConfig {
  int primary_id = 7;
  int aux_id = 24;
  cv::Vec3d aux_in_primary = cv::Vec3d(-0.052, 0.0, 0.0); // 辅标签在主标签坐标系中的位置，默认基线长度为52mm。
  cv::Vec3d fusion_bias = cv::Vec3d(0.0, -0.1, 0.0); // 融合坐标系下的偏置（单位：米），默认在 -Y 方向 0.1m，应用于融合后的平移向量。
};

// 融合步骤的可选调试输出。
struct FusionDebugInfo {
  bool has_primary = false; //是否检测到主标签
  bool has_aux = false; //是否检测到辅标签
  bool used_primary_fallback = false; //是否使用了主标签的回退方案
  double measured_baseline = 0.0; //测量到的基线长度
};


// AprilTag 检测封装：图像预处理 + 标签位姿估计。
class AprilTagPoseCore {
 public:

  /**
   * @brief 构造 AprilTag 位姿估计核心对象
   * @param tag_size 标签边长（米）
   * @param quad_decimate 检测降采样系数
   * @param quad_sigma 检测前高斯平滑参数
   * @param nthreads 检测线程数
   * @param refine_edges 是否启用边缘细化
   */
  AprilTagPoseCore(double tag_size,
                   double quad_decimate,
                   double quad_sigma,
                   int nthreads,
                   bool refine_edges)
      : tag_size_(tag_size) {
    family_ = tagStandard41h12_create(); // 创建标签族对象，支持 41h12 标签。

    detector_ = apriltag_detector_create(); // 创建 AprilTag 检测器对象。
    apriltag_detector_add_family(detector_, family_); // 将标签族添加到检测器中。

    detector_->quad_decimate = quad_decimate; // 设置检测降采样系数，较大值可加速检测但可能降低精度。
    detector_->quad_sigma = quad_sigma; // 设置检测前高斯平滑参数，较大值可抑制噪声但可能模糊边缘。
    detector_->nthreads = nthreads; // 设置检测线程数，建议根据 CPU 核心数调整。
    detector_->debug = 0; // 禁用内部调试输出。
    detector_->refine_edges = refine_edges ? 1 : 0; // 是否启用边缘细化，启用后可提高亚像素级精度但增加计算量。

    clahe_ = cv::createCLAHE(1.2, cv::Size(8, 8));  // 创建 CLAHE 对象，用于自适应直方图均衡化，增强局部对比度，改善检测鲁棒性。
  }

  const std::string& FamilyName() const 
  {
    static const std::string kFamily("tagStandard41h12"); // 标签族名称，固定为 "tagStandard41h12"。
    return kFamily; // 返回标签族名称的常量引用。
  }

  ~AprilTagPoseCore() 
  {
    if (detector_ != nullptr) 
    {
      apriltag_detector_destroy(detector_); // 销毁检测器对象，释放相关资源。
      detector_ = nullptr; // 避免悬空指针。
    }
    if (family_ != nullptr) 
    {
      tagStandard41h12_destroy(family_); // 销毁标签族对象，释放相关资源。
      family_ = nullptr; // 避免悬空指针。
    }
  }

  /**
   * @brief 检测并估计图像中每个 AprilTag 的位姿
   * @param gray 输入图像（灰度或 BGR）
   * @param k 相机内参与畸变参数
   * @return 标签位姿结果列表
   */
  std::vector<TagPoseResult> Detect(const cv::Mat& gray, const CameraIntrinsics& k) 
  {
    std::vector<TagPoseResult> results;// 存储检测结果的向量。
    if (gray.empty() || !k.IsValid() || detector_ == nullptr) 
    {
      return results;// 输入无效时返回空结果。
    }

    cv::Mat mono;// 用于存储单通道灰度图的临时变量。
    if (gray.type() == CV_8UC1) 
    {
      mono = gray;// 已经是灰度图，无需转换。
    } else {
      cv::cvtColor(gray, mono, cv::COLOR_BGR2GRAY);// 将彩色图转换为灰度图。
    }

    cv::Mat processed;// 用于存储 CLAHE 处理后图像的临时变量。
    clahe_->apply(mono, processed); // 对灰度图应用 CLAHE 增强局部对比度，提高检测鲁棒性。

    image_u8_t* im = image_u8_create_stride(
        static_cast<unsigned int>(processed.cols), // 创建 AprilTag 库使用的图像结构，指定宽度、高度和行跨度。
        static_cast<unsigned int>(processed.rows), // 图像高度。
        static_cast<unsigned int>(processed.step)); // 图像行跨度（字节数），通常为 cols * channels。
    if (im == nullptr) 
    {
      return results; // 内存分配失败时返回空结果。
    }

    for (int r = 0; r < processed.rows; ++r) 
    {
      std::memcpy(im->buf + r * im->stride, processed.ptr<uint8_t>(r), static_cast<size_t>(processed.cols)); // 将处理后的图像数据复制到 AprilTag 库使用的图像结构中，逐行复制以适应不同的行跨度。
    }

    zarray_t* detections = apriltag_detector_detect(detector_, im); // 调用 AprilTag 库的检测函数，传入图像结构，返回检测到的标签列表。
    const int count = zarray_size(detections); // 获取检测到的标签数量。
    results.reserve(count); // 预先分配结果向量的内存，以避免后续的动态扩容，提高性能。

    for (int i = 0; i < count; ++i) 
    {
      apriltag_detection_t* det = nullptr; // 定义指针用于存储当前检测到的标签信息。  
      zarray_get(detections, i, &det); // 从检测结果列表中获取第 i 个标签的检测信息，存储在 det 指针指向的结构中。

      apriltag_detection_info_t info; // 定义结构体用于存储位姿估计所需的信息，包括检测结果、标签大小和相机内参。
      info.det = det; // 将当前检测到的标签信息传入位姿估计结构体。
      info.tagsize = tag_size_; // 设置标签的实际边长（米），用于位姿估计。
      info.fx = k.fx; // 设置相机焦距 fx，用于位姿估计中的投影计算。
      info.fy = k.fy; // 设置相机焦距 fy，用于位姿估计中的投影计算。
      info.cx = k.cx; // 设置相机主点 cx，用于位姿估计中的投影计算。
      info.cy = k.cy; // 设置相机主点 cy，用于位姿估计中的投影计算。

      apriltag_pose_t pose; // 定义结构体用于存储位姿估计的输出结果，包括旋转矩阵、平移向量和重投影误差。
      const double err = estimate_tag_pose(&info, &pose); // 调用 AprilTag 库的位姿估计函数，传入位姿估计所需的信息结构体，输出位姿结果和重投影误差。

      TagPoseResult result; // 定义结构体用于存储当前标签的位姿估计结果，包括标签 ID、重投影误差、旋转矩阵和平移向量。
      result.id = det->id; // 将检测到的标签 ID 存储在结果结构体中。
      result.err = err; // 将位姿估计的重投影误差存储在结果结构体中，单位为像素。
      result.translation = cv::Vec3d(pose.t->data[0], pose.t->data[1], pose.t->data[2]); // 将位姿估计的平移向量存储在结果结构体中，单位为米。
      result.rotation = cv::Matx33d(
          pose.R->data[0], pose.R->data[1], pose.R->data[2], // 将位姿估计的旋转矩阵存储在结果结构体中，单位为无量纲。
          pose.R->data[3], pose.R->data[4], pose.R->data[5], // 旋转矩阵按行优先存储，依次填充到 cv::Matx33d 结构中。
          pose.R->data[6], pose.R->data[7], pose.R->data[8]); 
      results.push_back(result); // 将当前标签的位姿估计结果添加到结果向量中。

      std::free(pose.R); // 释放位姿估计结果中的旋转矩阵内存，避免内存泄漏。
      std::free(pose.t); 
    }

    apriltag_detections_destroy(detections); // 销毁检测结果列表，释放相关资源。
    image_u8_destroy(im); // 销毁 AprilTag 库使用的图像结构，释放相关资源。
    return results;
  }

 private:

  apriltag_family_t* family_ = nullptr; 
  apriltag_detector_t* detector_ = nullptr; 
  cv::Ptr<cv::CLAHE> clahe_; 
  double tag_size_ = 0.055;
};

// 将 id7/id6（或配置的主辅标签）融合为稳定的主标签位姿。
class AprilTagDualTagFusion {
 public:

  explicit AprilTagDualTagFusion(const FusionConfig& config)
      : config_(config), expected_baseline_(cv::norm(config_.aux_in_primary)) {}

  /**
   * @brief 从检测结果中选取主/辅标签的最优样本
   * @param tags 当前帧全部标签结果
   * @param primary_tag 输出主标签
   * @param has_primary 输出是否检测到主标签
   * @param aux_tag 输出辅标签
   * @param has_aux 输出是否检测到辅标签
   */
  void SelectBestTags(const std::vector<TagPoseResult>& tags,
                      TagPoseResult* primary_tag,
                      bool* has_primary,
                      TagPoseResult* aux_tag,
                      bool* has_aux) const {
    *has_primary = false;
    *has_aux = false;

    for (const auto& tag : tags) {
      if (tag.id == config_.primary_id) { *primary_tag = tag; *has_primary = true; }
      if (tag.id == config_.aux_id)   { *aux_tag = tag;   *has_aux = true;     }
    }
  }

  /**
   * @brief 融合主/辅标签位姿，求解主标签稳定位姿
   * @param primary_tag 主标签结果
   * @param has_primary 是否有主标签
   * @param aux_tag 辅标签结果
   * @param has_aux 是否有辅标签
   * @param debug_info 可选调试输出
   * @return 融合位姿结果
   */
  FusedPoseResult SolvePrimaryPose(const TagPoseResult& primary_tag,
                                   bool has_primary,
                                   const TagPoseResult& aux_tag,
                                   bool has_aux,
                                   FusionDebugInfo* debug_info) const {
    FusedPoseResult fused;
    fused.id = config_.primary_id;

    if (debug_info != nullptr) 
    {
      debug_info->has_primary = has_primary; // 输出是否检测到主标签。
      debug_info->has_aux = has_aux;
      debug_info->used_primary_fallback = false; // 当前实现中不提供单标签回退方案，因此始终为 false。
      debug_info->measured_baseline = 0.0; // 当前实现中仅在同时检测到主辅标签时计算基线长度，否则保持为 0。
    }

    if (has_primary && has_aux) 
    {
      const cv::Vec3d baseline_vec = primary_tag.translation - aux_tag.translation;  // id24 -> id7
      const double measured_baseline = cv::norm(baseline_vec); // 计算测量到的基线长度，即主标签和平标签之间的距离。
      if (debug_info != nullptr) 
      {
        debug_info->measured_baseline = measured_baseline; // 输出测量到的基线长度，单位为米。
      }

      // 同时看到两个标签时：中心取中点，x 轴强约束为 id24 -> id7，再叠加融合系偏置。
      fused.rotation = RotationFromBaseline(primary_tag.rotation, aux_tag.rotation, baseline_vec);
      fused.translation = 0.5 * (primary_tag.translation + aux_tag.translation)
                          + fused.rotation * config_.fusion_bias;
      fused.err = 0.5 * (primary_tag.err + aux_tag.err); // 将主辅标签的重投影误差平均值作为融合后的误差，提供一个综合的质量指标。
      fused.valid = true;
      return fused;
    }

    // 必须同时检测到主辅标签；不再提供单标签偏移补偿。
    return fused;
  }

  int PrimaryId() const
  {
    return config_.primary_id; // 返回配置的主标签 ID。
  }

  int AuxId() const
  {
    return config_.aux_id; // 返回配置的辅标签 ID。
  }

  void SetFusionBias(const cv::Vec3d& bias) { config_.fusion_bias = bias; }

  double ExpectedBaseline() const 
  {
    return expected_baseline_; // 返回配置的预期基线长度，单位为米。目前实现中未直接使用该值，但可以作为后续改进的参考，例如在单标签检测时提供基线长度的补偿。
  }

 private:
  /**
   * @brief 归一化向量，失败时返回回退向量
   * @param v 待归一化向量
   * @param fallback 回退向量
   * @return 归一化结果或回退向量
   */
  static cv::Vec3d NormalizeOrFallback(const cv::Vec3d& v, const cv::Vec3d& fallback) 
  {
    const double n = cv::norm(v);
    if (n > 1e-9) 
    {
      return v * (1.0 / n);
    }
    return fallback;
  }


  static cv::Vec3d ColAsVec3(const cv::Matx33d& r, int col) 
  {
    return cv::Vec3d(r(0, col), r(1, col), r(2, col));
  }

  /**
   * @brief 依据基线方向与法向提示构造融合旋转矩阵
   * @param r_primary 主标签旋转矩阵
   * @param r_aux 辅标签旋转矩阵
   * @param baseline_vec 辅标签指向主标签的基线向量
   * @return 融合后的右手系旋转矩阵
   */
  // 以基线方向为 x 轴构造右手系旋转矩阵：x=baseline(id24->id7)。
  cv::Matx33d RotationFromBaseline(const cv::Matx33d& r_primary,
                                   const cv::Matx33d& r_aux,
                                   const cv::Vec3d& baseline_vec) const {
    const cv::Vec3d fallback_x = ColAsVec3(r_primary, 0);
    const cv::Vec3d fallback_y = ColAsVec3(r_primary, 1);
    const cv::Vec3d fallback_z = ColAsVec3(r_primary, 2);

    cv::Vec3d x_axis = NormalizeOrFallback(baseline_vec, fallback_x);

    // 优先使用双标签平均法向，避免 y/z 在视角变化时无约束翻转。
    const cv::Vec3d normal_hint = NormalizeOrFallback(ColAsVec3(r_primary, 2) + ColAsVec3(r_aux, 2), fallback_z);
    cv::Vec3d y_axis = cv::Vec3d(normal_hint[1] * x_axis[2] - normal_hint[2] * x_axis[1],
                                 normal_hint[2] * x_axis[0] - normal_hint[0] * x_axis[2],
                                 normal_hint[0] * x_axis[1] - normal_hint[1] * x_axis[0]);
    y_axis = NormalizeOrFallback(y_axis, fallback_y);

    cv::Vec3d z_axis = cv::Vec3d(x_axis[1] * y_axis[2] - x_axis[2] * y_axis[1],
                                 x_axis[2] * y_axis[0] - x_axis[0] * y_axis[2],
                                 x_axis[0] * y_axis[1] - x_axis[1] * y_axis[0]);
    z_axis = NormalizeOrFallback(z_axis, fallback_z);

    // 重新正交化，降低数值误差。
    y_axis = cv::Vec3d(z_axis[1] * x_axis[2] - z_axis[2] * x_axis[1],
                       z_axis[2] * x_axis[0] - z_axis[0] * x_axis[2],
                       z_axis[0] * x_axis[1] - z_axis[1] * x_axis[0]);
    y_axis = NormalizeOrFallback(y_axis, fallback_y); 

    return cv::Matx33d(
        x_axis[0], y_axis[0], z_axis[0],
        x_axis[1], y_axis[1], z_axis[1],
        x_axis[2], y_axis[2], z_axis[2]); // 构造融合后的旋转矩阵，确保以基线方向为 x 轴，并通过法向提示稳定 y/z 轴的方向，最终形成一个正交的右手系旋转矩阵。
  }

  FusionConfig config_;
  double expected_baseline_ = 0.0;
};

}  // namespace apriltag_pose_module

// 前向声明，避免在头文件中引入 ros/ros.h
namespace ros { class NodeHandle; }

// 世界坐标系位姿（由 apriltag.cpp 实现）
struct WorldPose { double x = 0.0; double y = 0.0; double z = 0.0; double yaw = 0.0; };

namespace Ten {
namespace apriltag_pose_module {
extern WorldPose processOneFrame(const cv::Mat&, ros::NodeHandle* = nullptr);
extern void serial_send();
}  // namespace apriltag_pose_module
}  // namespace Ten

#endif
