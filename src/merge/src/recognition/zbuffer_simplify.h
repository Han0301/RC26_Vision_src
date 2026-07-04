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

namespace Ten{

#define L_ 1.2f         // 台阶长度
#define H_ 0.2f         // 台阶高度
#define ly1_ 0.425f     // 台阶到方块的间距
#define lx1_ 0.425f     // 台阶到方块的间距
#define lh_ 0.35f       // 方块的长度
#define X_ 2.58f        // 初始位置到梅花林1号位置边角的x轴距离
#define Y_ 3.395f       // 初始位置到梅花林1号位置边角的y轴距离
#define LIDAR_HEIGHT_ 0.717     // 雷达的高度 

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
};

struct HistValue {
    int position;  // 直方图的位置（对应H/S/V的取值）
    int count;     // 该位置的像素数量
    float degree_of_promacy;        // 首位度
};

// 计算筛空的指标
struct score{
    int idx;
    std::vector<HistValue> top_n_h;
    std::vector<HistValue> top_n_s;
    std::vector<HistValue> top_n_v;
    float hsv_score;
    int valid_count = 0;                      // 有效像素数量
    bool is_managed = true;     
};


// 初始化方块和台阶的3d点，2d点的 结构体
struct init_3d_box{
    // 1 3D点集合
    std::vector<cv::Point3f> W_object_plum_points_;        // 方块3D点和台阶3D点,在world下
    std::vector<cv::Point3f> LM_object_plum_points_;       // lidar_move 方块3D点和台阶3D点， 在lidar下，move动态
    std::vector<cv::Point3f> C_object_plum_points_;        //  方块3D点和台阶3D点,在camera下
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_LM_plum_object_points_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_C_plum_object_points_;

    // 2 转化成的像素坐标系下， 2D像素点
    std::vector<cv::Point2f> object_plum_2d_points_;

    std::vector<box> box_lists_;
    std::vector<score> score_lists_;
    // 3. x,y,z方向的 偏移量
    float offset_x_ = 0.0f;
    float offset_y = 0.0f;
    float offset_z = 0.0f;

    // 无参构造函数
    init_3d_box()
    :pcl_LM_plum_object_points_(new pcl::PointCloud<pcl::PointXYZ>()),
    pcl_C_plum_object_points_(new pcl::PointCloud<pcl::PointXYZ>())

    {
        object_plum_2d_points_.resize(96*2);
        W_object_plum_points_.resize(96*2);
        LM_object_plum_points_.resize(96*2);
        C_object_plum_points_.resize(96*2);

        // 初始化 box_lists_ 和 score_lists
        box_lists_.resize(12);
        score_lists_.resize(12);

        for(int i = 0; i < 12; i++)
        {
            box_lists_[i].idx = i + 1;
            box_lists_[i].roi_image = cv::Mat::zeros(160, 160, CV_8UC3);

            score_lists_[i].idx = i + 1;
        }

        // 初始化 W_object_plum_points_
        float arr_[12] {0.4, 0.2, 0.4, 0.2, 0.4, 0.6, 0.4, 0.6, 0.4, 0.2, 0.4, 0.2};
        for(int j = 0; j < 4; j++) {
            for(int i = 0; i < 3; i++) {
                // 方块8个3D点
                W_object_plum_points_[(j * 3 + i) * 8 + 0] = cv::Point3f(X_ + j*L_ + lx1_,       Y_ - i*L_ - ly1_,       arr_[i*3+j]+lh_);                
                W_object_plum_points_[(j * 3 + i) * 8 + 1] = cv::Point3f(X_ + j*L_ + lx1_,       Y_ - i*L_ - ly1_ - lh_, arr_[i*3+j]+lh_);                
                W_object_plum_points_[(j * 3 + i) * 8 + 2] = cv::Point3f(X_ + j*L_ + lx1_,       Y_ - i*L_ - ly1_ - lh_, arr_[i*3+j]);
                W_object_plum_points_[(j * 3 + i) * 8 + 3] = cv::Point3f(X_ + j*L_ + lx1_,       Y_ - i*L_ - ly1_,       arr_[i*3+j]);
                W_object_plum_points_[(j * 3 + i) * 8 + 4] = cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ - i*L_ - ly1_,       arr_[i*3+j]+lh_);
                W_object_plum_points_[(j * 3 + i) * 8 + 5] = cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ - i*L_ - ly1_ - lh_, arr_[i*3+j]+lh_);
                W_object_plum_points_[(j * 3 + i) * 8 + 6] = cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ - i*L_ - ly1_ - lh_, arr_[i*3+j]);
                W_object_plum_points_[(j * 3 + i) * 8 + 7] = cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ - i*L_ - ly1_,       arr_[i*3+j]);
            }
        } 
        for(int j = 0; j < 4; j++) {
            for(int i = 0; i < 3; i++) {
                // 台阶8个3D点
                W_object_plum_points_[96 + (j * 3 + i) * 8 + 0] = cv::Point3f(X_ + j*L_,      Y_ - i*L_,      arr_[i*3+j]);
                W_object_plum_points_[96 + (j * 3 + i) * 8 + 1] = cv::Point3f(X_ + j*L_,      Y_ - i*L_- L_,  arr_[i*3+j]);
                W_object_plum_points_[96 + (j * 3 + i) * 8 + 2] = cv::Point3f(X_ + j*L_,      Y_ - i*L_- L_,  0);
                W_object_plum_points_[96 + (j * 3 + i) * 8 + 3] = cv::Point3f(X_ + j*L_,      Y_ - i*L_,      0);
                W_object_plum_points_[96 + (j * 3 + i) * 8 + 4] = cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_,      arr_[i*3+j]);
                W_object_plum_points_[96 + (j * 3 + i) * 8 + 5] = cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_ - L_, arr_[i*3+j]);
                W_object_plum_points_[96 + (j * 3 + i) * 8 + 6] = cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_ - L_, 0);
                W_object_plum_points_[96 + (j * 3 + i) * 8 + 7] = cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_,      0);
            }
        }    

        // 初始化 LM_object_plum_points_，C_object_plum_points_
        for(int i = 0; i < 96 * 2; i++){
            //减雷达高度
            LM_object_plum_points_[i] = cv::Point3f(W_object_plum_points_[i].x, W_object_plum_points_[i].y, W_object_plum_points_[i].z - LIDAR_HEIGHT_);

            C_object_plum_points_[i]  = cv::Point3f(W_object_plum_points_[i].x, W_object_plum_points_[i].y, W_object_plum_points_[i].z);
        }    

        // 初始化 pcl_LM_plum_object_points_，pcl_C_plum_object_points_
        for(int i = 0; i < 96 * 2; i++){
            pcl::PointXYZ tmp;
            tmp.x = LM_object_plum_points_[i].x;
            tmp.y = LM_object_plum_points_[i].y;
            tmp.z = LM_object_plum_points_[i].z;
            pcl_LM_plum_object_points_->points.push_back(tmp);
            pcl_C_plum_object_points_->points.push_back(tmp);  
        }
    }
    // 用于转 pcl_C_plum_object_points_ 到 C_object_plum_points_
    void pcl_to_C()
    {
        cv::Point3f tmp;
        for(size_t i = 0; i < 96*2; i++)
        {
            tmp.x = pcl_C_plum_object_points_->points[i].x;
            tmp.y = pcl_C_plum_object_points_->points[i].y;
            tmp.z = pcl_C_plum_object_points_->points[i].z;

            C_object_plum_points_[i] = tmp;
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
 * @param interested_boxes 输入 int， 12 数组， 1 表示存在， 0 表示不存在， -1 表示异常
 */
void set_interested_boxes(int interested_boxes[12])
{
    std::lock_guard<std::mutex> lock(mtx_);
    for(int i = 0; i < 12; i++){interested_boxes_[i] = interested_boxes[i];}
};

/**
 * @brief 通过更新zbuffer矩阵来更新 box_lists
 * @param image 输入的图像
 * @param C_object_plum_points 相机坐标系下，方块和台阶的3D点
 * @param object_plum_2d_points  像素坐标系下，方块和台阶的2d点
 * @param box_lists 方块的列表，std::vector<box>
 */
void set_box_lists_(
    const cv::Mat& image,     
    const std::vector<cv::Point3f>& C_object_plum_points,
    const std::vector<cv::Point2f>& object_plum_2d_points,
    std::vector<box>& box_lists);

/**
 * @brief 拼接调试图像
 * @param box_lists  方块的列表，std::vector<box>
 * @param score_lists   得分列表 std::vector<score>
 * @param debug_best_roi_image 必须为 cv::Mat::zeros(480, 640, CV_8UC3) !!!
 */
void set_debug_roi_image(
    std::vector<Ten::box>box_lists,
    std::vector<Ten::score> score_lists,
    cv::Mat& debug_best_roi_image);

/**
 * @brief 直接在原图像中绘制框
 * @param image 输入图像
 * @param object_plum_2d_points_ 输入的 方块 和台阶的 2d 点对
 * @return cv::Mat 调试图像
 * 
*/
cv::Mat update_debug_image(
    cv::Mat image,
    const std::vector<cv::Point2f>& object_plum_2d_points_
);

/**
 * @brief 由用户自己输入标准的hsv 值
 * @param stand_hsv 输入int 型容器，size为3，表示3个通道的标准hsv值
 */
void set_stand_hsv_(std::vector<int>stand_hsv){
    standard_hsv_ = stand_hsv;
};

/**
 * @brief 获取 standard_hsv_
*/
std::vector<int> get_standard_hsv_()
{
    return standard_hsv_;
}

/**
 * @brief 设置 hsv 前几最大众数
 * @param box_lists  方块的列表，std::vector<box>
 * @param score_lists   得分列表 std::vector<score>
 * @param top_n         设置的前几最大众数
 * @param min_manage_points 最小像素处理的阈值
*/
void set_hsv_top_n(std::vector<box>& box_lists, std::vector<score>& score_lists,int top_n,int min_manage_points = 600);
/**
 * @brief 根据 hsv 前几最大众数 来计算 标准值
 * @param box_lists  方块的列表，std::vector<box>
 * @param score_lists   得分列表 std::vector<score>
 * @param top_n 
 * @param min_manage_points 最小像素处理的阈值
*/
void set_hsv_topn_stand(std::vector<box>& box_lists, std::vector<score>& score_lists,int top_n,int min_manage_points = 600);
/**
 * @brief 根据 hsv 前几最大众数 来计算 得分
 * @param box_lists  方块的列表，std::vector<box>
 * @param score_lists   得分列表 std::vector<score>
 * @param top_n 
 * @param min_manage_points 最小像素处理的阈值
*/
void set_hsv_topn_score(std::vector<box>& box_lists, std::vector<score>& score_lists,int top_n,int min_manage_points = 600);


private:
int exist_boxes_[12] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};             // 由用户自己设置存在方块的数组
int interested_boxes_[12] = {1,1,1,1,1,1,1,1,1,1,1,1};        // 由用户自己设置感兴趣的方块的数组
std::vector<int> standard_hsv_ = {0,0,0};

mutable std::mutex mtx_;

float cal_distance (const cv::Point3f& p) {
    return sqrt(powf(p.x, 2) + powf(p.y, 2) + powf(p.z, 2));
}

std::vector<HistValue> getTopNValues(const cv::Mat& hist, int top_n) {
    std::vector<HistValue> hist_values;
    
    // 1. 提取直方图所有非零值（零值无统计意义）
    for (int col = 0; col < hist.cols; col++) {
        int count = hist.at<int>(0, col);
        if (count > 0) { // 只保留有像素的位置
            hist_values.push_back({col, count});
        }
    }
    
    // 2. 按数量降序排序（数量多的在前）
    std::sort(hist_values.begin(), hist_values.end(), 
        [](const HistValue& a, const HistValue& b) {
            return a.count > b.count;
        });
    
    // 3. 截取前N个（如果总数不足N，取全部）
    if (hist_values.size() > top_n) {
        hist_values.resize(top_n);
    }
    
    // 4. 填充 首位度
    float first_count = static_cast<float>(hist_values[0].count);
    for (int i = 0; i < top_n; i ++)
    {   
        float now_count = static_cast<float>(hist_values[i].count);
        hist_values[i].degree_of_promacy = now_count / first_count;
    }
    return hist_values;
}
};


    extern Ten::Ten_zbuffer_simplify _ZBUFFER_SIMPLIFY_;
    extern Ten::init_3d_box _INIT_3D_BOX_;
}       // namespace Ten
#endif 