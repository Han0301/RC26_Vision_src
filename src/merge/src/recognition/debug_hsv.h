#ifndef __DEBUG_HSV_H_
#define __DEBUG_HSV_H_
#include <ros/ros.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <algorithm>
#include <unordered_map>
#include <vector>
#include <array>
#include <numeric>
#include <unordered_set>
#include <mutex>
#include <cmath>
#include <cfloat>
#include <climits>
#include <iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <cfloat>
#include "./../method_math.h"
#include "zbuffer_simplify.h"

namespace Ten{

class Ten_debug_hsv{
public:

/**
 * @brief 可视化并保存单张图片的HSV直方图（输入vector长度3：H/S/V三通道各1个直方图）
 * @param hsv_hist 输入直方图容器：
 *        - 索引0: H通道直方图 
 *        - 索引1: S通道直方图 
 *        - 索引2: V通道直方图
 * @param save_path 保存路径（如："single_img_hsv_hist.jpg"）
 * @param h_bin_num H通道分箱数（默认18，适配0-179范围）
 * @param s_bin_num S通道分箱数（默认32，适配0-255范围）
 * @param v_bin_num V通道分箱数（默认32，适配0-255范围）
 * @param canvas_height 画布高度（默认300像素，影响直方图显示高度）
 */
void save_hsv_hist_visualization(const std::vector<cv::Mat>& hsv_hist, 
                                 const std::string& save_path,
                                 int h_bin_num = 1,  // H通道独立bin数
                                 int s_bin_num = 1,  // S通道独立bin数
                                 int v_bin_num = 1,  // V通道独立bin数
                                 int canvas_height = 480);


// 功能函数 set_hsv_hist: 输入roi_image，生成hsv_list(一张图片的3个通道直方图)
std::vector<cv::Mat> set_hsv_hist(cv::Mat roi_image);




};

extern Ten::Ten_debug_hsv _DEBUG_HSV_;
}
#endif 