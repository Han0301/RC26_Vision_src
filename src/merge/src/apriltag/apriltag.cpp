#include "../serial.h"   // 串口通信
#include "../camera.h"   // RealSense 相机封装

#include <algorithm>   // std::min, std::max
#include <atomic>   // std::atomic
#include <cmath>   // std::sqrt, std::abs, std::atan2, std::acos, M_PI
#include <cstdio>   // printf, std::snprintf
#include <deque>   // std::deque (滑动窗口)
#include <string>   // std::string
#include <vector>   // std::vector

#include <opencv2/opencv.hpp>   // cv::Mat, cv::Matx33d, cv::Vec3d, cv::Rodrigues, cv::drawFrameAxes

#include <ros/ros.h>   // ros::NodeHandle, ros::Publisher, ros::Rate
#include <std_msgs/Float64.h>   // std_msgs::Float64 消息类型

#include <urcu/urcu-memb.h>   // URCU 用户态 RCU 内存屏障

#include "apriltag_pose_module.h"  // AprilTag 位姿估计模块接口
#include "./../parameter/parameter.h"

/*
  author：zy

  全链路处理流程 (单帧 BGR → 世界位姿 → 串口发送):

  1. 相机采集
     Ten_camera::camera_read() 读取一帧 BGR 彩色图像 (640×480, 60fps)

  2. AprilTag 检测 + 位姿估计 (apriltag_pose_module.h)
     core_.Detect(image, intrinsics)
     ├─ BGR → 灰度 → CLAHE 自适应直方图均衡 (增强对比度)
     ├─ AprilTag C 库检测 (tagStandard41h12 族, 41h12 编码标签)
     └─ estimate_tag_pose(): 单标签 PnP 位姿估计, 返回 R,t,重投影误差 err

  3. 位姿融合 (根据模式选择)
     ├─ 单标签模式: 选目标ID(error最小) → 帧间跳变统计(滑动窗口 std)
     └─ 双标签模式: SelectBestTags(选主id7+辅id24 error最小的一对)
          → SolvePrimaryPose(基线向量=X轴, 双法向平均=Z轴, 右手系正交)

  4. 相机系 → 世界系 坐标变换
     kCamToWorld 旋转矩阵: cam+z→world+x, cam+x→world-y, cam+y→world-z
     ├─ worldT = kCamToWorld * tag.translation + world_bias
     ├─ worldR = kCamToWorld * tag.rotation
     └─ yaw = atan2(R(1,0), R(0,0)) + π/2 → EMA 平滑 (α=0.35)

  5. 位姿质量监控
     滑动窗口 (100帧) 计算世界 x/y/z/yaw 帧间跳变标准差 → ROS Float64 发布
     话题: apriltag/std_world_x, std_world_y, std_world_z, std_world_yaw

  6. 可视化 (OpenCV)
     drawFrameAxes: 绘制 RGB 坐标轴 + 标签文本 + 跳变/std 数值

  7. 串口发送 (独立线程, 30Hz)
     serial_send(): 读取→处理→NaN检测→serial.serial_send(arr, 6, 16字节)
     数据格式: float[4] = {x, y, z, yaw}, 设备号 6

  输入:  BGR 彩色图像 (cv::Mat, 640×480)
  输出:  WorldPose {x(m), y(m), z(m), yaw(rad)} → 串口 + ROS话题
*/

namespace Ten {
std::atomic<bool> g_apriltag_enabled{true};  // AprilTag 模块使能标志(初始关闭)
}

namespace Ten {
namespace apriltag_pose_module {

// ── 相机 → 世界旋转矩阵: cam+z→world+x, cam+x→world-y, cam+y→world-z ──
static const cv::Matx33d kCamToWorld(   // 3×3 旋转矩阵, 将相机系变换到世界系
    0.0, 0.0, 1.0,   // 相机 +z 轴 → 世界 +x 轴
   -1.0, 0.0, 0.0,   // 相机 +x 轴 → 世界 -y 轴
    0.0,-1.0, 0.0);   // 相机 +y 轴 → 世界 -z 轴

// ========== AprilTag 配置参数 (#define, 修改后需重新编译) ==========

// // ── 相机内参 ──
// #define APRILTAG_FX 603.70866605   // 焦距 fx (像素)
// #define APRILTAG_FY 602.75542134   // 焦距 fy (像素)
// #define APRILTAG_CX 328.76609402   // 主点 cx (像素)
// #define APRILTAG_CY 232.37643764   // 主点 cy (像素)
// #define APRILTAG_K1 0.09239455   // 径向畸变 k1
// #define APRILTAG_K2 -0.16531614   // 径向畸变 k2
// #define APRILTAG_P1 0.00322926   // 切向畸变 p1
// #define APRILTAG_P2 0.00042834   // 切向畸变 p2
// #define APRILTAG_K3 0.0   // 径向畸变 k3

// // ── 相机分辨率 ──
// #define APRILTAG_CAMERA_WIDTH  640   // 彩色图宽度 (像素)
// #define APRILTAG_CAMERA_HEIGHT 480   // 彩色图高度 (像素)
// #define APRILTAG_CAMERA_FPS    60   // 相机帧率

// // ── AprilTag 检测参数 ──
// #define APRILTAG_TAG_SIZE      0.0155   // 标签边长 (米)
// #define APRILTAG_TAG_SPACING   0.006   // 双标签间距 (米)
// #define APRILTAG_QUAD_DECIMATE 1.0   // 检测降采样系数
// #define APRILTAG_QUAD_SIGMA    0.0   // 检测前高斯平滑
// #define APRILTAG_NTHREADS      1   // 检测线程数
// #define APRILTAG_REFINE_EDGES  true   // 是否启用亚像素边缘细化

// // ── 融合参数 ──
// #define APRILTAG_PRIMARY_ID      7   // 主标签 ID
// #define APRILTAG_AUX_ID          24   // 辅标签 ID
// #define APRILTAG_SINGLE_TAG_MODE false   // 是否启用单标签模式
// #define APRILTAG_SINGLE_TAG_ID   24   // 单标签模式下的标签 ID
// #define APRILTAG_FUSION_BIAS_X   0.0   // 融合系偏置 X (米)
// #define APRILTAG_FUSION_BIAS_Y  -0.1   // 融合系偏置 Y (米)
// #define APRILTAG_FUSION_BIAS_Z   0.0   // 融合系偏置 Z (米)

// // ── 世界系偏置 (米) ──
// #define APRILTAG_WORLD_BIAS_X 0.0   // 世界系 X 偏置
// #define APRILTAG_WORLD_BIAS_Y (-(0.08995 - 0.0175 - 0.015))   // 世界系 Y 偏置 ≈ 0.05745
// #define APRILTAG_WORLD_BIAS_Z (-(0.07470 - 0.0125))   // 世界系 Z 偏置 ≈ 0.0622

// // ── 滤波参数 ──
// #define APRILTAG_JUMP_WINDOW   100   // 跳变统计滑动窗口大小 (帧)
// #define APRILTAG_YAW_EMA_ALPHA 0.35   // Yaw EMA 平滑系数

// // ── 可视化参数 ──
// #define APRILTAG_ENABLE_SAW_WINDOW false   // 是否启用 OpenCV 可视化窗口
// #define APRILTAG_DRAW_FONT_SCALE   0.5   // 绘制字体缩放比例
// #define APRILTAG_DRAW_THICKNESS    1   // 绘制线条粗细
// #define APRILTAG_SAW_WINDOW_NAME   "saw"   // 可视化窗口名称


class AprilTagProcessor {
 public:
  /** @brief 构造函数: 初始化检测核心、相机内参、融合器 */
  explicit AprilTagProcessor()
      : intrinsics_([]() {   // lambda: 构造相机内参
          ::apriltag_pose_module::CameraIntrinsics k;
          k.fx = APRILTAG_FX; k.fy = APRILTAG_FY;
          k.cx = APRILTAG_CX; k.cy = APRILTAG_CY;
          k.camera_matrix = (cv::Mat_<double>(3, 3) << APRILTAG_FX, 0.0, APRILTAG_CX,
                                                        0.0, APRILTAG_FY, APRILTAG_CY,
                                                        0.0, 0.0, 1.0);  // 3×3 内参矩阵 K
          k.dist_coeffs = (cv::Mat_<double>(1, 5) << APRILTAG_K1, APRILTAG_K2,
                              APRILTAG_P1, APRILTAG_P2, APRILTAG_K3);   // 1×5 畸变系数
          return k;
        }()),
        core_(APRILTAG_TAG_SIZE, APRILTAG_QUAD_DECIMATE, APRILTAG_QUAD_SIGMA,
              APRILTAG_NTHREADS, APRILTAG_REFINE_EDGES),   // 初始化 AprilTag 检测核心
        fusion_([]() {   // lambda: 构造融合配置
          ::apriltag_pose_module::FusionConfig f;
          f.primary_id = APRILTAG_PRIMARY_ID;
          f.aux_id = APRILTAG_AUX_ID;
          f.aux_in_primary = cv::Vec3d(-(APRILTAG_TAG_SIZE + APRILTAG_TAG_SPACING), 0.0, 0.0);  // 基线方向
          f.fusion_bias = cv::Vec3d(APRILTAG_FUSION_BIAS_X, APRILTAG_FUSION_BIAS_Y, APRILTAG_FUSION_BIAS_Z);
          return f;
        }()) {
    if (APRILTAG_ENABLE_SAW_WINDOW) {
      cv::namedWindow(APRILTAG_SAW_WINDOW_NAME, cv::WINDOW_NORMAL);   // 创建可调大小窗口
      printf("[AprilTag] OpenCV saw window enabled: %s\n", APRILTAG_SAW_WINDOW_NAME);
    }
    printf("[AprilTag] Single-tag mode: %s, tag_id=%d\n",
           APRILTAG_SINGLE_TAG_MODE ? "ON" : "OFF",
           APRILTAG_SINGLE_TAG_ID);
  }

  ~AprilTagProcessor() {
    if (APRILTAG_ENABLE_SAW_WINDOW) {
      cv::destroyWindow(APRILTAG_SAW_WINDOW_NAME);   // 销毁可视化窗口
    }
  }

  /** @brief 主入口: 处理单帧 BGR 图像, 返回世界位姿
   *  @param imageBgr 输入 BGR 彩色图像 (640×480)
   *  @param nh       可选 ROS NodeHandle, 传入则发布 std_world_* 话题
   *  @return 世界坐标系位姿
   */
  WorldPose processOneFrame(const cv::Mat& imageBgr, ros::NodeHandle* nh = nullptr) {
    initRosPublishers(nh);                                  // 惰性初始化 ROS 发布者
    if (imageBgr.empty()) return WorldPose{};                // 空帧返回零位姿

    auto tags = core_.Detect(imageBgr, intrinsics_);         // AprilTag 检测 + PnP 位姿估计
    auto fused = resolvePose(tags);                          // 单/双标签 → 融合位姿
    drawDebugOverlay(imageBgr, fused);                       // 可视化 (坐标轴 + 跳变文本)
    return worldOutput(fused);                               // 坐标变换 + 跳变统计 + ROS 发布
  }

 private:
  // ==================== Pipeline 步骤 ====================

  /** @brief 惰性初始化 ROS 标准差发布者 */
  void initRosPublishers(ros::NodeHandle* nh) {
    if (nh == nullptr || !pub_std_world_x_.getTopic().empty()) return;   // 已初始化或无需初始化
    pub_std_world_x_   = nh->advertise<std_msgs::Float64>("apriltag/std_world_x",   10);
    pub_std_world_y_   = nh->advertise<std_msgs::Float64>("apriltag/std_world_y",   10);
    pub_std_world_z_   = nh->advertise<std_msgs::Float64>("apriltag/std_world_z",   10);
    pub_std_world_yaw_ = nh->advertise<std_msgs::Float64>("apriltag/std_world_yaw", 10);
  }

  /** @brief 单/双标签位姿解算: 输入检测结果, 输出融合后的 (或选优后的) 位姿 */
  ::apriltag_pose_module::FusedPoseResult resolvePose(
      const std::vector<::apriltag_pose_module::TagPoseResult>& tags) {

    if (APRILTAG_SINGLE_TAG_MODE) {   // ── 单标签模式 ──
      const ::apriltag_pose_module::TagPoseResult* best = nullptr;
      double best_err = std::numeric_limits<double>::max();
      for (const auto& t : tags) {   // 遍历选误差最小的目标 ID 标签
        if (t.id == APRILTAG_SINGLE_TAG_ID && t.err < best_err) { best = &t; best_err = t.err; }
      }

      if (best == nullptr) {   // 未找到目标标签
        last_status_text_ = std::string("tag id=") + std::to_string(APRILTAG_SINGLE_TAG_ID) + " not found";
        return ::apriltag_pose_module::FusedPoseResult{};   // invalid
      }

      // 计算相机系帧间跳变
      if (has_prev_pose_) {
        last_jump_pos_mm_ = cv::norm(best->translation - prev_translation_) * 1000.0;  // 米→毫米
        last_jump_x_mm_ = std::abs(best->translation[0] - prev_translation_[0]) * 1000.0;
        last_jump_y_mm_ = std::abs(best->translation[1] - prev_translation_[1]) * 1000.0;
        last_jump_z_mm_ = std::abs(best->translation[2] - prev_translation_[2]) * 1000.0;

        cv::Matx33d R_diff = best->rotation * prev_rotation_.t();   // 相对旋转: R_cur * R_prevᵀ
        double tr = R_diff(0,0) + R_diff(1,1) + R_diff(2,2);   // 旋转矩阵的迹
        double cos_theta = std::max(-1.0, std::min(1.0, (tr - 1.0) * 0.5));  // cosθ = (tr(R)-1)/2
        (void)cos_theta;   // rot_jump_deg 暂不使用
      }

      if (has_prev_pose_) {   // 更新跳变滑动窗口
        jump_x_buf_.push_back(last_jump_x_mm_);
        jump_y_buf_.push_back(last_jump_y_mm_);
        jump_z_buf_.push_back(last_jump_z_mm_);
        while (jump_x_buf_.size() > APRILTAG_JUMP_WINDOW) jump_x_buf_.pop_front();
        while (jump_y_buf_.size() > APRILTAG_JUMP_WINDOW) jump_y_buf_.pop_front();
        while (jump_z_buf_.size() > APRILTAG_JUMP_WINDOW) jump_z_buf_.pop_front();
      }

      prev_translation_ = best->translation;   // 保存当前帧供下一帧跳变计算
      prev_rotation_ = best->rotation;
      has_prev_pose_ = true;

      ::apriltag_pose_module::FusedPoseResult fused;   // 单标签包装为融合结果
      fused.valid = true;
      fused.id = best->id;
      fused.err = best->err;
      fused.translation = best->translation;
      fused.rotation = best->rotation; 
      return fused;
    }

    // ── 双标签融合模式 ──
    ::apriltag_pose_module::TagPoseResult primaryTag, auxTag;
    bool hasPrimary = false, hasAux = false;
    fusion_.SelectBestTags(tags, &primaryTag, &hasPrimary, &auxTag, &hasAux);

    if (!hasPrimary || !hasAux) {
      last_status_text_ = "need both id7 and id24";
      char buf[128];
      std::snprintf(buf, sizeof(buf), "Both tags required (id=%d and id=%d). has_primary=%d has_aux=%d",
                    fusion_.PrimaryId(), fusion_.AuxId(), hasPrimary ? 1 : 0, hasAux ? 1 : 0); // 输出检测状态
      printf("[AprilTag] %s\n", buf);
      return ::apriltag_pose_module::FusedPoseResult{};   // 双标签模式，必须同时检测到主辅标签，否则返回无效结果。
    }

    auto fused = fusion_.SolvePrimaryPose(primaryTag, hasPrimary, auxTag, hasAux, nullptr); // 融合主辅标签位姿，求解稳定位姿
    if (!fused.valid) printf("[AprilTag] Failed to solve fused primary pose.\n"); // 检测到主辅标签但融合失败的情况（理论上不应该发生）
    return fused;
  }

  /** @brief 统一可视化: 绘制坐标轴 + 状态文本 */
  void drawDebugOverlay(const cv::Mat& imageBgr,
                        const ::apriltag_pose_module::FusedPoseResult& fused) {
    if (!APRILTAG_ENABLE_SAW_WINDOW) return;   // 未启用可视化

    cv::Mat saw = imageBgr.clone();   // 克隆避免污染原图

    if (!fused.valid) {   // 位姿无效 → 显示错误提示
      cv::putText(saw, last_status_text_, cv::Point(20, 40),
                  cv::FONT_HERSHEY_SIMPLEX, APRILTAG_DRAW_FONT_SCALE,
                  cv::Scalar(0, 0, 255), APRILTAG_DRAW_THICKNESS);
    } else {
      // 绘制 RGB 坐标轴 + 标签文本
      const cv::Scalar color = APRILTAG_SINGLE_TAG_MODE ? cv::Scalar(0, 255, 0)   // 单标签: 绿色
                                                        : cv::Scalar(255, 0, 255); // 双标签: 紫色
      const std::string label = APRILTAG_SINGLE_TAG_MODE ? "single" : "center";
      drawFusedPose(saw, fused, color, label);

      if (APRILTAG_SINGLE_TAG_MODE) {   // 单标签模式额外显示跳变/std 文本
        char buf[200];
        std::snprintf(buf, sizeof(buf),
                      "jump: tot=%.4f x=%.4f y=%.4f z=%.4f mm | "
                      "std x=%.4f y=%.4f z=%.4f",
                      last_jump_pos_mm_, last_jump_x_mm_, last_jump_y_mm_, last_jump_z_mm_,
                      rollingStd(jump_x_buf_), rollingStd(jump_y_buf_), rollingStd(jump_z_buf_));
        cv::putText(saw, buf, cv::Point(20, saw.rows - 20),   // 图像底部
                    cv::FONT_HERSHEY_SIMPLEX, APRILTAG_DRAW_FONT_SCALE,
                    cv::Scalar(0, 255, 255), APRILTAG_DRAW_THICKNESS); 
      }
    }
    cv::imshow(APRILTAG_SAW_WINDOW_NAME, saw);
    cv::waitKey(1);   // 刷新窗口
  }

  /** @brief 在世界系位姿上绘制 RGB 坐标轴 + 标签原点文本 */
  void drawFusedPose(cv::Mat& image,
                     const ::apriltag_pose_module::FusedPoseResult& fused,
                     const cv::Scalar& color,
                     const std::string& label) {
    cv::Vec3d rvec;
    cv::Rodrigues(fused.rotation, rvec);   // 旋转矩阵→旋转向量
    const cv::Vec3d tvec = fused.translation;

    cv::drawFrameAxes(image, intrinsics_.camera_matrix,
                      intrinsics_.dist_coeffs, rvec, tvec,
                      0.036, APRILTAG_DRAW_THICKNESS + 1);   // 绘制 RGB 坐标轴 (长度 3.6cm)

    std::vector<cv::Point3f> originPts(1, cv::Point3f(0.0f, 0.0f, 0.0f));  // 标签原点
    std::vector<cv::Point2f> originUv;   // 投影像素坐标
    cv::projectPoints(originPts, rvec, tvec,
                      intrinsics_.camera_matrix, intrinsics_.dist_coeffs,
                      originUv);   // 3D→2D 投影
    if (!originUv.empty()) {
      const cv::Point pt(static_cast<int>(originUv[0].x),
                         static_cast<int>(originUv[0].y));   // 原点像素坐标
      char buf[160];
      std::snprintf(buf, sizeof(buf), "%s id=%d t=[%.3f %.3f %.3f]",
                    label.c_str(), fused.id, tvec[0], tvec[1], tvec[2]);
      cv::putText(image, buf, pt + cv::Point(6, 18),   // 原点偏移处绘制
                  cv::FONT_HERSHEY_SIMPLEX, APRILTAG_DRAW_FONT_SCALE,
                  color, APRILTAG_DRAW_THICKNESS);
    }
  }

  /** @brief 相机系→世界系变换 + 跳变统计 + ROS 标准差发布 */
  WorldPose worldOutput(const ::apriltag_pose_module::FusedPoseResult& tag) {
    if (!tag.valid) return WorldPose{};   // invalid → 全零位姿

    cv::Matx33d worldR = kCamToWorld * tag.rotation;   // 旋转: 相机系→世界系
    cv::Vec3d   worldT = kCamToWorld * tag.translation;   // 平移: 相机系→世界系

    latest_world_x_ = worldT[0] + APRILTAG_WORLD_BIAS_X;   // 叠加世界系偏置
    latest_world_y_ = worldT[1] + APRILTAG_WORLD_BIAS_Y;
    latest_world_z_ = worldT[2] + APRILTAG_WORLD_BIAS_Z;
    const double raw_yaw = std::atan2(worldR(1, 0), worldR(0, 0)) + M_PI_2;  // 提取 yaw + 90° 偏移
    if (!yaw_initialized_) { smoothed_yaw_ = raw_yaw; yaw_initialized_ = true; }
    else {
      smoothed_yaw_ = APRILTAG_YAW_EMA_ALPHA * raw_yaw
                      + (1.0 - APRILTAG_YAW_EMA_ALPHA) * smoothed_yaw_;  // EMA: α·raw + (1-α)·prev
    }
    latest_world_yaw_ = smoothed_yaw_; // 使用平滑后的 yaw 输出

    printf("[AprilTag] world t=[%.3f %.3f %.3f] err=%.8e\n",
           worldT[0], worldT[1], worldT[2], tag.err);

    if (has_prev_world_) {   // 有历史世界位姿时计算世界系跳变
      const double jump_wx = std::abs(latest_world_x_ - prev_world_x_) * 1000.0;  // 米→毫米
      const double jump_wy = std::abs(latest_world_y_ - prev_world_y_) * 1000.0;
      const double jump_wz = std::abs(latest_world_z_ - prev_world_z_) * 1000.0;
      double jump_wyaw = latest_world_yaw_ - prev_world_yaw_;
      while (jump_wyaw >  M_PI) jump_wyaw -= 2.0 * M_PI;   // 归一化到 [-π, π]
      while (jump_wyaw < -M_PI) jump_wyaw += 2.0 * M_PI;
      jump_wyaw = std::abs(jump_wyaw) * 180.0 / M_PI;   // 弧度→度

      world_jump_x_buf_.push_back(jump_wx);   // 推入滑动窗口
      world_jump_y_buf_.push_back(jump_wy);
      world_jump_z_buf_.push_back(jump_wz);
      world_jump_yaw_buf_.push_back(jump_wyaw);
      while (world_jump_x_buf_.size() > APRILTAG_JUMP_WINDOW) world_jump_x_buf_.pop_front();
      while (world_jump_y_buf_.size() > APRILTAG_JUMP_WINDOW) world_jump_y_buf_.pop_front();
      while (world_jump_z_buf_.size() > APRILTAG_JUMP_WINDOW) world_jump_z_buf_.pop_front();
      while (world_jump_yaw_buf_.size() > APRILTAG_JUMP_WINDOW) world_jump_yaw_buf_.pop_front();

      if (!pub_std_world_x_.getTopic().empty()) {   // ROS 话题已初始化→发布标准差
        std_msgs::Float64 msg;
        msg.data = rollingStd(world_jump_x_buf_);   pub_std_world_x_.publish(msg);
        msg.data = rollingStd(world_jump_y_buf_);   pub_std_world_y_.publish(msg);
        msg.data = rollingStd(world_jump_z_buf_);   pub_std_world_z_.publish(msg);
        msg.data = rollingStd(world_jump_yaw_buf_); pub_std_world_yaw_.publish(msg);
      }
    }
    prev_world_x_ = latest_world_x_;   // 保存本帧供下一帧跳变计算
    prev_world_y_ = latest_world_y_;
    prev_world_z_ = latest_world_z_;
    prev_world_yaw_ = latest_world_yaw_;
    has_prev_world_ = true;

    return WorldPose{latest_world_x_, latest_world_y_, latest_world_z_, latest_world_yaw_};
  }

  // ==================== 工具函数 ====================

  /** @brief 运行时调整融合偏置 */
  void setFusionBias(double x, double y, double z) { fusion_.SetFusionBias(cv::Vec3d(x, y, z)); }

  /** @brief 滑动窗口标准差: Var = E[X²] - (E[X])² */
  static double rollingStd(const std::deque<double>& buf) {
    const size_t n = buf.size();
    if (n < 2) return 0.0;   // 少于2个样本返回0
    double sum = 0.0, sum_sq = 0.0;
    for (double v : buf) { sum += v; sum_sq += v * v; }
    double mean = sum / static_cast<double>(n);   // 均值
    double var = sum_sq / static_cast<double>(n) - mean * mean;   // Var = E[X²] - µ²
    return std::sqrt(std::max(0.0, var));   // Std = √Var (防浮点负零)
  }

  // ==================== 成员变量 ====================

  // ── 不可变组件 ──
  ::apriltag_pose_module::CameraIntrinsics intrinsics_;   // 相机内参
  ::apriltag_pose_module::AprilTagPoseCore core_;   // AprilTag 检测核心
  ::apriltag_pose_module::AprilTagDualTagFusion fusion_;   // 双标签融合器

  // ── 相机系跳变状态 ──
  bool has_prev_pose_ = false;
  cv::Vec3d prev_translation_ = cv::Vec3d(0, 0, 0);   // 前一帧标签平移
  cv::Matx33d prev_rotation_ = cv::Matx33d::eye();   // 前一帧标签旋转
  std::deque<double> jump_x_buf_;   // 相机系 X 跳变滑动窗口 (毫米)
  std::deque<double> jump_y_buf_;
  std::deque<double> jump_z_buf_;
  double last_jump_pos_mm_ = 0.0;   // 最新帧位置跳变总量 (毫米)
  double last_jump_x_mm_ = 0.0;     // 最新帧 X 跳变 (毫米)
  double last_jump_y_mm_ = 0.0;
  double last_jump_z_mm_ = 0.0;

  // ── 世界系位姿状态 ──
  double latest_world_x_ = 0.0;   // 最新世界 X (米)
  double latest_world_y_ = 0.0;
  double latest_world_z_ = 0.0;
  double latest_world_yaw_ = 0.0;   // 最新世界 Yaw (弧度)
  double smoothed_yaw_ = 0.0;   // EMA 平滑后 Yaw
  bool yaw_initialized_ = false;

  // ── 世界系跳变状态 ──
  bool has_prev_world_ = false;
  double prev_world_x_ = 0.0, prev_world_y_ = 0.0;
  double prev_world_z_ = 0.0, prev_world_yaw_ = 0.0;
  std::deque<double> world_jump_x_buf_;   // 世界 X 跳变滑动窗口 (毫米)
  std::deque<double> world_jump_y_buf_;
  std::deque<double> world_jump_z_buf_;
  std::deque<double> world_jump_yaw_buf_;   // 世界 Yaw 跳变滑动窗口 (度)

  // ── ROS 发布者 ──
  ros::Publisher pub_std_world_x_;
  ros::Publisher pub_std_world_y_;
  ros::Publisher pub_std_world_z_;
  ros::Publisher pub_std_world_yaw_;

  // ── 可视化状态 ──
  std::string last_status_text_;   // 上一帧状态文本 (无标签时展示)
};


/** @brief 外部调用入口: 处理单帧 BGR, 返回世界位姿 */
//WorldPose processOneFrame(const cv::Mat& color, ros::NodeHandle* nh) {
//    static AprilTagProcessor instance;   // 全局单例
//    return instance.processOneFrame(color, nh);
//}


// ========== AprilTag 串口发送 (独立线程, 60Hz) ==========

/** @brief 串口发送线程主函数: 读相机→位姿估计→NaN检测→串口发送 */
void serial_send() {
    std::cout << "serial_send() " << std::endl;
    urcu_memb_register_thread();   // 注册 URCU 线程
    Ten_serial& serial = Ten_serial::GetInstance();   // 串口单例
    Ten_camera& cam = Ten_camera::GetInstance();   // 相机单例, 分辨率与内参标定对齐
    std::cout << "Ten_camera::GetInstance() " << std::endl;
    cam.reset_camera(APRILTAG_CAMERA_WIDTH, APRILTAG_CAMERA_HEIGHT,APRILTAG_CAMERA_FPS);
    AprilTagProcessor instance;   // AprilTag 处理器实例
    float arr[4] = {0};   // 发送数据: [x, y, z, yaw]

    ros::Rate sl(APRILTAG_CAMERA_FPS);   // 60Hz 发送频率
    while (_APRILTAG_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag()) {
         // 检查线程池运行标志
        //std::cout << "_APRILTAG_FLAG_ " << std::endl;
        //if (!g_apriltag_enabled.load()) { sl.sleep(); continue; }   // 模块未使能则休眠

        cv::Mat color = cam.camera_read();   // 读取彩色图像 (阻塞)
        if (color.empty()) { 
          std::cout << "color.empty() " << std::endl;
          sl.sleep(); 
          continue; }   // 空帧跳过

        WorldPose pose = instance.processOneFrame(color, nullptr);   // 执行位姿估计

        arr[0] = static_cast<float>(pose.x);
        arr[1] = static_cast<float>(pose.y);
        arr[2] = static_cast<float>(pose.z);
        arr[3] = static_cast<float>(pose.yaw);

        bool nan_flag = false;
        for (int i = 0; i < 4; i++)
            if (std::isnan(arr[i])) { nan_flag = true; break; }   // NaN 检测
        if (nan_flag) { sl.sleep(); continue; }   // NaN 则丢弃本帧

        serial.serial_send(arr, 6, sizeof(arr));   // 串口发送 (设备6, 16字节)
        // printf("[APRILTAG] x=%.3f y=%.3f z=%.3f yaw=%.3f rad\n",
        //        arr[0], arr[1], arr[2], arr[3]);
        //std::cout << arr[0] << " " << arr[1] << " " << arr[2] << " " << arr[3];

        sl.sleep();   // 按帧率休眠
    }
    urcu_memb_unregister_thread();   // 注销 URCU 线程
}

}  // namespace apriltag_pose_module
}  // namespace Ten
