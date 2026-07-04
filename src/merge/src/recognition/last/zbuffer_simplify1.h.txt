#ifndef __Ten_zbuffer_simplify_H_
#define __Ten_zbuffer_simplify_H_
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


namespace Ten{

#define L_ 1.2f         // 台阶长度
#define H_ 0.2f         // 台阶高度
#define ly1_ 0.425f     // 台阶到方块的间距
#define lx1_ 0.425f     // 台阶到方块的间距
#define lh_ 0.35f       // 方块的长度
#define X_ 2.58f        // 初始位置到梅花林1号位置边角的x轴距离
#define Y_ 3.395f       // 初始位置到梅花林1号位置边角的y轴距离

struct surface_2d_point {        
    int idx;                       // 对应方块索引
    cv::Point2f left_up;           // 左上2D点
    cv::Point2f right_up;          // 右上2D点
    cv::Point2f right_down;        // 右下2D点
    cv::Point2f left_down;         // 左下2D点
    float surface_depth;           // 该表面的深度值
};

struct box{
    int idx;                             // 表示位置的下标索引
    cv::Mat roi_image;                   // 裁剪出来的roi图片
    int cls = 0;                             // 识别类别
    float confidence = 0.0f;                // 自信度
    int zbuffer_flag = 0;                    // zbuffer是否处理的标志位， 0 表示未处理， 1 表示已处理， -1 表示异常
    int exist_flag = -1;                      // 是否筛空的标志位， 0 表示空， 1 表示有方块， -1 表示未处理
    int points_count;                    // 当前帧裁剪出来的roi_image中有效像素的点的数量，用于与历史的图像比较来更新最优
};

// 初始化方块和台阶的3d点，2d点的 结构体
struct init_3d_box{
    // 1 3D点集合
    std::vector<cv::Point3f> W_object_points_;        // 方块3D点,在world下
    std::vector<cv::Point3f> W_plum_points_;          // 台阶3D点,在world下
    std::vector<cv::Point3f> LS_object_points_;       // lidar_static 方块3D点,在lidar下，static静态用于初始化
    std::vector<cv::Point3f> LS_plum_blossom_points_; // lidar_static 台阶3D点,在lidar下，static静态用于初始化
    std::vector<cv::Point3f> LM_object_points_;       // lidar_move 方块3D点,在lidar下，move动态
    std::vector<cv::Point3f> LM_plum_points_;         // lidar_move 台阶3D点,在lidar下，move动态
    std::vector<cv::Point3f> C_object_points_;        // 方块3D点,在camera下
    std::vector<cv::Point3f> C_plum_points_;          // 台阶3D点,在camera下
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_LS_plum_object_points_;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_LM_plum_object_points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_C_plum_object_points_;

    // 2 2D像素点集合,未处理
    std::vector<cv::Point2f> object_plum_points;
    std::vector<cv::Point2f> object_2d_points;
    std::vector<cv::Point2f> plum_2d_points;

    std::vector<std::vector<surface_2d_point>> object_2d_points_;
    std::vector<std::vector<surface_2d_point>> plum_2d_points_;

    std::vector<box> box_lists_;
    // 3. x,y,z方向的 偏移量
    float offset_x_ = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;

    // 无参构造函数
    init_3d_box()
    :pcl_LS_plum_object_points_(new pcl::PointCloud<pcl::PointXYZ>()),
    //pcl_LM_plum_object_points_(new pcl::PointCloud<pcl::PointXYZ>()),
    pcl_C_plum_object_points_(new pcl::PointCloud<pcl::PointXYZ>())

    {
        object_plum_points.resize(96*2);
        object_2d_points.resize(96);
        plum_2d_points.resize(96);
        C_object_points_.resize(96);
        C_plum_points_.resize(96);

        object_2d_points_.resize(3);
        plum_2d_points_.resize(3);

        box_lists_.resize(12);
        for(int i = 0; i < 12; i++)
        {
            box_lists_[i].idx = i + 1;
            box_lists_[i].roi_image = cv::Mat::zeros(160, 160, CV_8UC3);
        }

        float arr_[12] {0.4, 0.2, 0.4, 0.2, 0.4, 0.6, 0.4, 0.6, 0.4, 0.2, 0.4, 0.2};
        for(int j = 0; j < 4; j++) {
            for(int i = 0; i < 3; i++) {
                // 方块8个3D点
                W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_,       Y_ - i*L_ - ly1_,       arr_[i*3+j]+lh_));                
                W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_,       Y_ - i*L_ - ly1_ - lh_, arr_[i*3+j]+lh_));                
                W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_,       Y_ - i*L_ - ly1_ - lh_, arr_[i*3+j]));
                W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_,       Y_ - i*L_ - ly1_,       arr_[i*3+j]));
                W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ - i*L_ - ly1_,       arr_[i*3+j]+lh_));
                W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ - i*L_ - ly1_ - lh_, arr_[i*3+j]+lh_));
                W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ - i*L_ - ly1_ - lh_, arr_[i*3+j]));
                W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ - i*L_ - ly1_,       arr_[i*3+j]));
                // 台阶8个3D点
                W_plum_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ - i*L_,      arr_[i*3+j] ));
                W_plum_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ - i*L_- L_,  arr_[i*3+j] ));
                W_plum_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ - i*L_- L_,  0));
                W_plum_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ - i*L_,      0));
                W_plum_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_,      arr_[i*3+j] ));
                W_plum_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_ - L_, arr_[i*3+j]));
                W_plum_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_ - L_, 0 ));
                W_plum_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_,      0));
            }
        }    
            for(int i = 0; i < 96; i++){
                //减雷达高度
                LS_object_points_.push_back(cv::Point3f(W_object_points_[i].x, W_object_points_[i].y, W_object_points_[i].z - 0.717));
                LS_plum_blossom_points_.push_back(cv::Point3f(W_plum_points_[i].x, W_plum_points_[i].y, W_plum_points_[i].z  - 0.717));
            }     
            for(int i = 0; i < 96; i++){
                pcl::PointXYZ tmp_object;
                tmp_object.x = LS_object_points_[i].x;
                tmp_object.y = LS_object_points_[i].y;
                tmp_object.z = LS_object_points_[i].z;
                pcl_LS_plum_object_points_->points.push_back(tmp_object);
                //pcl_LM_plum_object_points_->points.push_back(tmp_object);
                pcl_C_plum_object_points_->points.push_back(tmp_object);   
            }
            for(int i = 0; i < 96; i++){
                pcl::PointXYZ tmp_plum;
                tmp_plum.x = LS_plum_blossom_points_[i].x;
                tmp_plum.y = LS_plum_blossom_points_[i].y;
                tmp_plum.z = LS_plum_blossom_points_[i].z;
                pcl_LS_plum_object_points_->points.push_back(tmp_plum);
                //pcl_LM_plum_object_points_->points.push_back(tmp_plum);  
                pcl_C_plum_object_points_->points.push_back(tmp_plum);       
            }
            for(int i = 0; i < 96; i++){
                // 先给一个初始值
                LM_object_points_.push_back(cv::Point3f(W_object_points_[i].x, W_object_points_[i].y, W_object_points_[i].z));
                LM_plum_points_.push_back(cv::Point3f(W_plum_points_[i].x, W_plum_points_[i].y, W_plum_points_[i].z ));
                C_object_points_.push_back(cv::Point3f(W_object_points_[i].x, W_object_points_[i].y, W_object_points_[i].z));
                LM_plum_points_.push_back(cv::Point3f(W_plum_points_[i].x, W_plum_points_[i].y, W_plum_points_[i].z ));
            }

        if (W_object_points_.empty()) {
            ROS_WARN("object_points_ is empty!");
        }
        if (W_plum_points_.empty()) {
            ROS_WARN("plum_blossom_points_ is empty!");
        }

    }

    void split_2d_points()
    {
        // std::vector<cv::Point2f> object_plum_points;
        // std::vector<cv::Point2f> object_2d_points;
        // std::vector<cv::Point2f> plum_2d_points;
        // std::vector<cv::Point3f> C_object_points_;        // 方块3D点,在camera下
        // std::vector<cv::Point3f> C_plum_points_;          // 台阶3D点,在camera下
        // pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_C_plum_object_points_;
        for(size_t i = 0; i < 96; i++)
        {
            object_2d_points[i] = object_plum_points[i];
            //std::cout<< "object_2d_points["<<i<<"]: "<< object_2d_points[i] << std::endl;
        }
        for(size_t i = 96; i < 96*2; i++)
        {
            plum_2d_points[i-96] = object_plum_points[i];
        }

        cv::Point3f tmp;
        for(size_t i = 0; i < 96; i++)
        {
            tmp.x = pcl_C_plum_object_points_->points[i].x;
            tmp.y = pcl_C_plum_object_points_->points[i].y;
            tmp.z = pcl_C_plum_object_points_->points[i].z;

            C_object_points_[i] = tmp;
        }

        for(size_t i = 96; i < 96*2; i++)
        {
            tmp.x = pcl_C_plum_object_points_->points[i].x;
            tmp.y = pcl_C_plum_object_points_->points[i].y;
            tmp.z = pcl_C_plum_object_points_->points[i].z;

            C_plum_points_[i-96] = tmp;
        }

    }

};

class Ten_zbuffer_simplify{
public:
/**
 * @brief 由用户自己设置存在方块的数组, 方便调试
 * @param exist_boxes_ 输入 int， 12 数组， 1 表示存在， 0 表示不存在， -1 表示异常
 */
void set_exist_boxes(int exist_boxes[12])
{
    std::lock_guard<std::mutex> lock(mtx_);
    for(int i = 0; i < 12; i++){exist_boxes_[i] = exist_boxes[i];}
};
/**
 * @brief 由用户自己设置感兴趣的方块的数组（zbuffer矩阵将仅更新 有方块 且 感兴趣的 位置处的方块深度信息）, 方便调试
 * @param exist_boxes_ 输入 int， 12 数组， 1 表示存在， 0 表示不存在， -1 表示异常
 */
void set_interested_boxes(int interested_boxes[12])
{
    std::lock_guard<std::mutex> lock(mtx_);
    for(int i = 0; i < 12; i++){interested_boxes_[i] = interested_boxes[i];}
};

/**
 * @brief 根据转换后的相机坐标系下点和像素坐标系下点，来给object_2d_points_，plum_2d_points_赋值，为 set_box_lists作数据准备
 * @param C_object_points
 * @param C_plum_points
 * @param object_2d_points
 * @param plum_2d_points
 * @param object_2d_points_
 * @param plum_2d_points_
 */
void set_box_lists_init(
    const std::vector<cv::Point3f>& C_object_points,
    const std::vector<cv::Point3f>& C_plum_points,
    const std::vector<cv::Point2f>& object_2d_points,
    const std::vector<cv::Point2f>& plum_2d_points,
    std::vector<std::vector<surface_2d_point>>& object_2d_points_,
    std::vector<std::vector<surface_2d_point>>& plum_2d_points_
);

/**
 * @brief 通过更新zbuffer矩阵来更新 box_lists
 * @param image 输入的图像
 * @param object_2d_points_lists 方块的在图像中的2d点
 * @param plum_2d_points_lists   台阶的在图像中的2d点
 * @param box_lists 方块的列表，std::vector<box>
 */
void set_box_lists_(
    const cv::Mat& image,     
    const std::vector<std::vector<surface_2d_point>>& object_2d_points_lists,
    const std::vector<std::vector<surface_2d_point>>& plum_2d_points_lists,
    std::vector<box>& box_lists);

/**
 * @brief 拼接调试图像
 * @param box_lists  方块的列表，std::vector<box>
 * @param debug_best_roi_image 必须为 cv::Mat::zeros(480, 640, CV_8UC3) !!!
 */
void set_debug_roi_image(
    std::vector<Ten::box>box_lists,
    cv::Mat& debug_best_roi_image);

/**
 * @brief 通过图像来 判断 是否有方块
 * @param box_lists 方块的列表，std::vector<box>
 */
void set_HSV_exist_boxes_(std::vector<box>& box_lists);
/**
 * @brief 设置标准 hsv 值
 * @param box_lists 方块的列表，std::vector<box>
 */
void set_standard_hsv_(std::vector<box>box_lists);
/**
 * @brief 由用户z自己输入标准的hsv 值
 * @param stand_hsv 输入int 型容器，size为3，表示3个通道的标准hsv值
 */
void set_stand_hsv_(std::vector<int>stand_hsv){
    standard_hsv_ = stand_hsv;
};
private:
int exist_boxes_[12] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};             // 由用户自己设置存在方块的数组
int interested_boxes_[12] = {1,1,1,1,1,1,1,1,1,1,1,1};        // 由用户自己设置感兴趣的方块的数组
std::vector<int> standard_hsv_ = {0,0,0};

mutable std::mutex mtx_;

// 功能函数
int cal_single_standard_hsv(std::vector<int>single_channel_result, int min_total_single = INT_MAX){
    int result_channel = -1;
    for(int i = 0;i < single_channel_result.size(); ++i){
        int candidate = single_channel_result[i];
        int total_diff = 0;
        // 计算每一个值与其他所有值的总差值
        for(int j = 0;j < single_channel_result.size(); ++j){      
            total_diff += std::abs(candidate - single_channel_result[j]);
        }
        if(total_diff < min_total_single){
            min_total_single = total_diff;
            result_channel = candidate;
        }
    }
    if (result_channel == -1){
        ROS_WARN("cal_single_standard_hsv: result_channel = -1 !");
    }
    return result_channel;
};

std::unordered_map<int, std::tuple<int, int, int>> set_idx_hsv_map_(std::vector<box>& box_lists){
    std::unordered_map<int, std::tuple<int, int, int>>idx_hsv_map_;
    for (const auto& box: box_lists){
        cv::Mat hsv_image;
        cv::cvtColor(box.roi_image, hsv_image, cv::COLOR_RGB2HSV);
        //  跳过像素数≤600的idx
        if (box.points_count <= 600)continue;
        //  填充HSV直方图
        cv::Mat h_hist = cv::Mat::zeros(1, 180, CV_32S);     // HSV直方图
        cv::Mat s_hist = cv::Mat::zeros(1, 255, CV_32S);
        cv::Mat v_hist = cv::Mat::zeros(1, 255, CV_32S);
        // for (const auto& pt : box.points) {
        //     cv::Vec3b hsv = hsv_image.at<cv::Vec3b>(pt.y, pt.x);
        //     h_hist.at<int>(0, hsv[0])++;
        //     s_hist.at<int>(0, hsv[1])++;
        //     v_hist.at<int>(0, hsv[2])++;
        // }
        for(int i = 0; i < hsv_image.cols; i ++){
            for(int j = 0;j < hsv_image.rows;j++){
                cv::Vec3b hsv = hsv_image.at<cv::Vec3b>(i,j);
                if(hsv[2] == 0)continue;
                h_hist.at<int>(0, hsv[0])++;
                s_hist.at<int>(0, hsv[1])++;
                v_hist.at<int>(0, hsv[2])++;
            }
        }
        // 2.6 计算众数
        cv::Point h_max_loc, s_max_loc, v_max_loc;
        cv::minMaxLoc(h_hist, nullptr, nullptr, nullptr, &h_max_loc);
        cv::minMaxLoc(s_hist, nullptr, nullptr, nullptr, &s_max_loc);
        cv::minMaxLoc(v_hist, nullptr, nullptr, nullptr, &v_max_loc);
        // 2.7 填充idx_hsv_map，构建下标和对应的hsv众数
        int h_mode = h_max_loc.x;
        int s_mode = s_max_loc.x;
        int v_mode = v_max_loc.x;
        idx_hsv_map_[box.idx] = {h_mode, s_mode, v_mode};
    }
    return idx_hsv_map_;
}

inline float calc_camera_distance (const cv::Point3f& p) {
    return sqrt(powf(p.x, 2) + powf(p.y, 2) + powf(p.z, 2));
};
};


    extern Ten::Ten_zbuffer_simplify _ZBUFFER_SIMPLIFY_;
    extern Ten::init_3d_box _INIT_3D_BOX_;
}       // namespace Ten
#endif 