#ifndef __PNP_PARAM_H_
#define __PNP_PARAM_H_
#include <algorithm>
#include <cmath>
#include <numeric>
#include <string>
#include <vector>
namespace Ten
{
namespace KFS
{

// PnP算法配置参数结构体
struct kfsPnpConfig
{
  size_t colorWidth = 640;                  // 彩色图像宽度，匹配相机分辨率
  size_t colorHeight = 480;                 // 彩色图像高度，匹配相机分辨率
  size_t depthWidth = 640;                  // 深度图像宽度，匹配相机分辨率
  size_t depthHeight = 480;                 // 深度图像高度，匹配相机分辨率
  size_t fps = 30;                          // 帧率，调整数据处理速度

  double fx = 553.7294;                 // 相机x方向焦距，内参校准值
  double fy = 553.7891;                 // 相机y方向焦距，内参校准值
  double cx = 317.2345;                 // 相机x方向光心，内参校准值
  double cy = 239.7654;                 // 相机y方向光心，内参校准值

  double objectSize = 0.35;                 // 目标物理尺寸，单位米，调整位姿尺度

  double roiMinArea = 100.0;                // 最小ROI面积，过滤小噪声区域
  double roiMaxAreaRatio = 0.4;            // 最大ROI占比，过滤过大无效区域

  struct red
  {
    int labLMin = 30;                          // LAB颜色L通道最小值
    int labLMax = 255;                        // LAB颜色L通道最大值
    int labAMin = 135;                        // LAB颜色A通道最小值
    int labAMax = 255;                        // LAB颜色A通道最大值
    int labBMin = 130;                          // LAB颜色B通道最小值
    int labBMax = 180;                        // LAB颜色B通道最大值
  };

  struct blue
  {
    int labLMin = 0;                          // LAB颜色L通道最小值
    int labLMax = 255;                        // LAB颜色L通道最大值
    int labAMin = 120;                        // LAB颜色A通道最小值
    int labAMax = 180;                        // LAB颜色A通道最大值
    int labBMin = 50;                          // LAB颜色B通道最小值
    int labBMax = 120;                        // LAB颜色B通道最大值
  };
  kfsPnpConfig::red red;
  kfsPnpConfig::blue blue;

  int roi_padding = 0;
  int CloudDepth_min = 300;
  int CloudDepth_max = 2000;
  float voxelLeaf = 0.008f;                  // 体素滤波大小，值越大点云越稀疏
  
  float ClusterTolerance = 0.016;

  float size_min_bias = 0.06;

  double ransacDist = 0.012;                 // RANSAC平面距离阈值，调整平面拟合精度
  int ransacIter = 500;                     // RANSAC迭代次数，值越高拟合越准
  int minPlanePoints = 50;                  // 最小平面点数，过滤小平面
  double minSecondPlaneRatio = 0.3;         // 第二平面比例，保证辅助平面有效性
  double duplicateNormalDot = 0.8;          // 法向量重复阈值，过滤相似平面
  int maxPlanes = 3;                        // 最大平面数量，控制计算复杂度

  int minFirstFramePoints = 200;            // 首帧最小点数，保证首帧稳定性
  double minFirstFaceArea = 0.121;           // 首帧最小面积，单位平方米，保证目标尺寸
  int smoothWindow = 2;                     // 平滑窗口帧数，值越大姿态越平滑
};
} // namespace KFS
} // namespace Ten
#endif