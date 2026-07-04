#ifndef __ZBUFFEER_H_
#define __ZBUFFEER_H_
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
#include "methon_math.cpp"
// 命名空间封装
namespace Ten
{
// ===================== 内部结构体定义 =====================
struct surface_2d_point {        
    int idx;                       // 对应方块索引
    cv::Point2f left_up;           // 左上2D点
    cv::Point2f right_up;          // 右上2D点
    cv::Point2f right_down;        // 右下2D点
    cv::Point2f left_down;         // 左下2D点
    float surface_depth;           // 该表面的深度值
};

// ===================== 核心类声明+成员定义 =====================
class Ten_zbuffer
{
public:
    // 构造函数：初始化相机参数、3D点、方块列表
    Ten_zbuffer();

    // 析构函数：默认（无动态资源需释放）
    ~Ten_zbuffer() = default;

    // -------------------------- 核心函数声明 --------------------------
    /**
     * @brief 提取方块HSV并过滤空方块：统计有效像素的HSV众数，计算得分并返回高分局索引
     * @param zbuffer 深度缓冲矩阵
     * @param image 原始图像
     * @param processed_idxs 已处理的索引集合（引用传递，修改会同步到外部）
     * @return 得分≥95的方块索引列表
     */
    std::vector<int> extract_square_hsv_filterEmptyBoxes(
        const cv::Mat& zbuffer, 
        const cv::Mat& image,
        std::unordered_set<int>& processed_idxs
    );

    /**
     * @brief 深度缓冲遮挡处理：生成方块/台阶的深度缓冲，处理遮挡关系
     * @param box_2d_points_lists 方块各表面2D点列表
     * @param plum_box_2d_points_lists 台阶各表面2D点列表
     * @param camera_y 相机y坐标
     * @param camera_x 相机x坐标
     * @param camera_z 相机z坐标
     * @param yaw 相机偏航角
     * @param image 原始图像（用于获取尺寸）
     * @return 处理后的方块深度缓冲矩阵
     */
    cv::Mat zbuffer_occlusion(
        std::vector<std::vector<surface_2d_point>> box_2d_points_lists,
        std::vector<std::vector<surface_2d_point>> plum_box_2d_points_lists,
        const float camera_y,
        const float camera_x,
        const float camera_z,
        const double yaw,
        const cv::Mat& image
    );

    /**
     * @brief 绘制深度缓冲：在图像上绘制深度区域、方块索引
     * @param image 待绘制的图像（引用传递，直接修改）
     * @param zbuffer 深度缓冲矩阵
     * @param allow_idx_lists 有方块的序号
     * @param front_2d 前表面2D点列表
     * @param side_2d 侧表面2D点列表
     * @param up_2d 上表面2D点列表 
     */
    void draw_zbuffer(
        cv::Mat& image, 
        const cv::Mat& zbuffer, 
        const std::vector<int> allow_idx_lists,
        const std::vector<surface_2d_point>& front_2d,
        const std::vector<surface_2d_point>& side_2d,
        const std::vector<surface_2d_point>& up_2d
    );

    /** 
     * @brief 输入位置，图像，返回序号，最优裁剪图像对
    * @param msg
    * @param image
    */ 
    std::unordered_map<int, cv::Mat> manage_odom_zbuffer_roi(const nav_msgs::Odometry msg, cv::Mat & image);

    /** 
     * @brief 输入位置，图像，返回序号，最优裁剪图像对的数量
    * @param msg
    * @param image
    */ 
    size_t getBestRoiImage_size(const nav_msgs::Odometry msg,cv::Mat & image);

    /**
     * @brief 生成最优裁剪的roi图像字典
     */ 
    void roi_best_zbuffer(
    cv::Mat& image,
    const cv::Mat& zbuffer,
    const std::vector<int> allow_idx_lists);

    // 发布关于最优裁剪图像的debug图像
    cv::Mat roi_best_zbuffer_debug(const std::unordered_map<int, cv::Mat>& best_roi_image,const std::vector<int> yolo_result);

    float calc_camera_distance (cv::Point3f& p);

    // -------------------------- 辅助接口（获取内部数据） --------------------------

    cv::Mat get_rvec(std::string type){
        if (type == "WL"){
            return WL_rvec;
        }else if (type == "LC"){
            return LC_rvec;
        };
    }
    cv::Mat get_tvec(std::string type){
        if (type == "WL"){
            return WL_tvec;
        }else if (type == "LC"){
            return LC_tvec;
        };
    }

    const std::unordered_map<int, cv::Mat>& getBestRoiImage() const {
        return best_roi_image;
    }
    const cv::Mat& getdebugRoiImage() const {
        return debug_image;
    }

    // 更新r,t矩阵（旋转/平移向量）
    void updateTf(const cv::Mat& rvec, const cv::Mat& tvec, const std::string& chang_type) {
    std::lock_guard<std::mutex> lock(mtx_tf_);
        if(chang_type == "WL"){
            rvec.copyTo(WL_rvec);
            tvec.copyTo(WL_tvec);
            ROS_INFO("updateTf: Updated world_to_robot (WL) rvec/tvec");
        }else if(chang_type == "LC"){
            rvec.copyTo(LC_rvec);
            tvec.copyTo(LC_tvec);
            ROS_INFO("updateTf: Updated world_to_robot (LC) rvec/tvec");
        }else{};
    }


private:
    // ===================== 私有成员变量 =====================
    // 1. 相机参数
    cv::Mat K_;               // 内参矩阵
    cv::Mat distCoeffs_;      // 畸变系数

    cv::Mat WL_rvec;        // world_to_lidar
    cv::Mat WL_tvec;        // world_to_lidar
    
    // cv::Mat LC_rvec =  (cv::Mat_<float>(3, 3) << 
    //     0.99997239,0.00650835, -0.00358751,
    //     -0.00365338,  0.01013816, -0.99994193,
    //     -0.0064716,   0.99992743,  0.01016166
    //     );        // lidar_to_camera 矩阵
    // cv::Mat LC_rvec_vec = (cv::Mat_<float>(3, 1) << 1.209940, -1.213038, 1.211830);     // 向量
    // cv::Mat LC_tvec = (cv::Mat_<float>(3, 1) << 0.008304,1.260707, 0.033896);        // lidar_to_camera 向量
    cv::Mat LC_rvec =  (cv::Mat_<float>(3, 3) << 
        -0.0530959,  -0.998589,  -1.9891e-05,
        -0.0403182,  0.00216366,  -0.999185,
        0.997775,  -0.0530518,  -0.0403762
        );        // lidar_to_camera 矩阵
    cv::Mat LC_rvec_vec = (cv::Mat_<float>(3, 1) << 1.2126, -1.2788, 1.2281);     // 向量
    cv::Mat LC_tvec = (cv::Mat_<float>(3, 1) << 0.0214, 0.3877, 0.4997);        // lidar_to_camera 向量

    // 2. 常量参数（替代原宏定义）
    static constexpr float L_ = 1.2f;
    static constexpr float H_ = 0.2f;
    static constexpr float ly1_ = 0.425f;
    static constexpr float lx1_ = 0.425f;
    static constexpr float lh_ = 0.35f;
    static constexpr float X_ = 2.58f;
    static constexpr float Y_ = 3.395f;  //3.395
    float arr_[12] = {0.4, 0.2, 0.4, 0.2, 0.4, 0.6, 0.4, 0.6, 0.4, 0.2, 0.4, 0.2};

    // 3. 3D点集合
    std::vector<cv::Point3f> W_object_points_;       // 方块3D点,在world下
    std::vector<cv::Point3f> W_plum_blossom_points_; // 台阶3D点,在world下
    std::vector<cv::Point3f> LS_object_points_;       // lidar_static 方块3D点,在lidar下，static静态用于初始化
    pcl::PointCloud<pcl::PointXYZ> pcl_LS_plum_object_points_;
    std::vector<cv::Point3f> LS_plum_blossom_points_; // lidar_static 台阶3D点,在lidar下，static静态用于初始化
    pcl::PointCloud<pcl::PointXYZ> pcl_LM_plum_object_points_;
    std::vector<cv::Point3f> LM_object_points_;       // lidar_move 方块3D点,在lidar下，move动态
    std::vector<cv::Point3f> LM_plum_blossom_points_; // lidar_move 台阶3D点,在lidar下，move动态
    std::vector<cv::Point3f> C_object_points_;       // 方块3D点,在camera下
    std::vector<cv::Point3f> C_plum_blossom_points_; // 台阶3D点,在camera下
    pcl::PointCloud<pcl::PointXYZ> pcl_C_plum_object_points_;
    Eigen::Matrix4f transform_matrix;
    // 2D像素点集合,未处理
    std::vector<cv::Point2f> object_2d_points;
    std::vector<cv::Point2f> plum_blossom_2d_points;
    // 2D像素点集合，含有面的点和深度信息，喂给zbuffer_occlsion
    std::vector<surface_2d_point> object_2d_points_lists;
    std::vector<surface_2d_point> plum_blossom_2d_points_lists;


    // 5. 深度相关数据
    std::unordered_map<int, std::vector<float>> idx_to_depth_;  // 索引→深度映射
    cv::Mat last_zbuffer_;                       // 上一帧深度缓冲（用于防抖）

    // 6. 状态变量
    float last_camera_x_ = INFINITY;             // 上一帧相机x
    float last_camera_y_ = INFINITY;             // 上一帧相机y
    double last_yaw_ = INFINITY;                 // 上一帧相机偏航角
    bool is_move_ = true;                        // 相机是否移动

    // 7. 线程安全锁（保护共享数据）
    std::mutex mtx_last_zbuffer_;    // 保护last_zbuffer_
    std::mutex mtx_tf_;              // 保护rvec_/tvec_

    // 8. ROI缓存
    std::unordered_map<int, cv::Mat> best_roi_image;    // 存储最优ROI图像
    std::unordered_map<int, int> best_roi_count;        // 存储每个idx的最大点集数量
    cv::Mat debug_image;
    // ===================== 私有辅助函数声明 =====================
    /**
     * @brief 初始化3D点和方块/台阶列表（构造函数调用）
     */
    void initObjectPoints();
};

// ===================== 类成员函数实现 =====================
// 构造函数：初始化相机参数 + 3D点
Ten_zbuffer::Ten_zbuffer() {
    // 1. 初始化相机内参
    K_ = (cv::Mat_<double>(3,3) <<
        1380.4350, 0, 974.0183,
        0,  1385.0788, 541.4301,
        0, 0, 1);
    // 2. 畸变系数（零畸变）
    distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);

    // 4. 初始化3D点和方块/台阶列表，初始化LS,LM,C等坐标系下的坐标初始值
    initObjectPoints();
}

// 私有辅助函数：初始化3D点和方块/台阶的世界点坐标，初始化LS,LM,C等坐标系下的坐标初始值
void Ten_zbuffer::initObjectPoints() {
    // 1. 生成方块3D点
    // for(int i = 0; i < 4; i++) {
    //     for(int j = 0; j < 3; j++) {
    //         // 方块8个3D点
    //         W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_,       Y_ + i*L_ + ly1_,       arr_[i*3+j]+lh_));                
    //         W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ + i*L_ + ly1_,       arr_[i*3+j]+lh_));                
    //         W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ + i*L_ + ly1_,       arr_[i*3+j]));
    //         W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_,       Y_ + i*L_ + ly1_,       arr_[i*3+j]));
    //         W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_,       Y_ + i*L_ + ly1_ + lh_, arr_[i*3+j]+lh_));
    //         W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ + i*L_ + ly1_ + lh_, arr_[i*3+j]+lh_));
    //         W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_ + lh_, Y_ + i*L_ + ly1_ + lh_, arr_[i*3+j]));
    //         W_object_points_.push_back(cv::Point3f(X_ + j*L_ + lx1_,       Y_ + i*L_ + ly1_ + lh_, arr_[i*3+j]));

    //         // 台阶8个3D点
    //         W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ + i*L_,      arr_[i*3+j] ));
    //         W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ + i*L_,      arr_[i*3+j] ));
    //         W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ + i*L_,      0));
    //         W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ + i*L_,      0));
    //         W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ + i*L_ + L_, arr_[i*3+j] ));
    //         W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ + i*L_ + L_, arr_[i*3+j]));
    //         W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ + i*L_ + L_, 0 ));
    //         W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ + i*L_ + L_, 0));
    //     }
    // }    
    // transform_matrix << 
    // 0.99997219,  0.00650825, -0.00358794, -0.01800176,  // 第0行
    // -0.00365325,  0.01013824, -0.99994184,  1.22010031,  // 第1行
    // -0.00647204,  0.99992734,  0.01016173,  0.02747127,  // 第2行
    // 0.0         ,  0.0        ,  0.0        ,  1.0;       // 第3行（齐次项）

    transform_matrix << 
    -0.0530959,  -0.998589,  -1.9891e-05,  0.0214078,  // 第0行
    -0.0403182,  0.00216366,  -0.999185,  0.387689,  // 第1行
    0.997775,  -0.0530518,  -0.0403762,  0.499693,  // 第2行
    0.0         ,  0.0        ,  0.0        ,  1.0;       // 第3行（齐次项）

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
            W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ - i*L_,      arr_[i*3+j] ));
            W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ - i*L_- L_,  arr_[i*3+j] ));
            W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ - i*L_- L_,  0));
            W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_,      Y_ - i*L_,      0));
            W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_,      arr_[i*3+j] ));
            W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_ - L_, arr_[i*3+j]));
            W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_ - L_, 0 ));
            W_plum_blossom_points_.push_back(cv::Point3f(X_ + j*L_ + L_, Y_ - i*L_,      0));
        }
    }    
        for(int i = 0; i < 96; i++)
        {
            LS_object_points_.push_back(cv::Point3f(W_object_points_[i].x, W_object_points_[i].y, W_object_points_[i].z - 0.717));
            LS_plum_blossom_points_.push_back(cv::Point3f(W_plum_blossom_points_[i].x, W_plum_blossom_points_[i].y, W_plum_blossom_points_[i].z  - 0.717));

        }     
        for(int i = 0; i < 96; i++)
        {
            pcl::PointXYZ tmp_object;
            tmp_object.x = LS_object_points_[i].x;
            tmp_object.y = LS_object_points_[i].y;
            tmp_object.z = LS_object_points_[i].z;
            pcl_LS_plum_object_points_.points.push_back(tmp_object);
            pcl_LM_plum_object_points_.points.push_back(tmp_object);
            pcl_C_plum_object_points_.points.push_back(tmp_object);   
        }
        for(int i = 0; i < 96; i++){
            pcl::PointXYZ tmp_plum;
            tmp_plum.x = LS_plum_blossom_points_[i].x;
            tmp_plum.y = LS_plum_blossom_points_[i].y;
            tmp_plum.z = LS_plum_blossom_points_[i].z;
            pcl_LS_plum_object_points_.points.push_back(tmp_plum);
            pcl_LM_plum_object_points_.points.push_back(tmp_plum);  
            pcl_C_plum_object_points_.points.push_back(tmp_plum);       
        }
        for(int i = 0; i < 96; i++)
        {
            // 先给一个初始值
            LM_object_points_.push_back(cv::Point3f(W_object_points_[i].x, W_object_points_[i].y, W_object_points_[i].z));
            LM_plum_blossom_points_.push_back(cv::Point3f(W_plum_blossom_points_[i].x, W_plum_blossom_points_[i].y, W_plum_blossom_points_[i].z ));
            C_object_points_.push_back(cv::Point3f(W_object_points_[i].x, W_object_points_[i].y, W_object_points_[i].z));
            C_plum_blossom_points_.push_back(cv::Point3f(W_plum_blossom_points_[i].x, W_plum_blossom_points_[i].y, W_plum_blossom_points_[i].z ));
        }

    if (W_object_points_.empty()) {
        ROS_WARN("object_points_ is empty!");
    }
    if (W_plum_blossom_points_.empty()) {
        ROS_WARN("plum_blossom_points_ is empty!");
    }
    // ROS_INFO("object_points_ size: %zu, plum_blossom_points_ size: %zu", 
    //      W_object_points_.size(), W_plum_blossom_points_.size());
}

std::unordered_map<int, cv::Mat> Ten_zbuffer::manage_odom_zbuffer_roi(const nav_msgs::Odometry msg,cv::Mat & image)
{
    auto start1 = std::chrono::high_resolution_clock::now();
    // 1. 解析姿态角（从四元数到欧拉角）
    tf2::Quaternion tf_quat(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    );
    tf2::Matrix3x3 mat(tf_quat);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);  
    double x =  msg.pose.pose.position.x + 0.0037;
    double y =  msg.pose.pose.position.y + 0.0012;
    double z =  msg.pose.pose.position.z + 0.0026;

    XYZ xyz;
    xyz._x = x;
    xyz._y = y;
    //xyz._z = z - 0.049303;
    xyz._z = z;
    RPY rpy;
    rpy._roll = roll + 0.007;
    rpy._pitch = pitch - 0.01;
    rpy._yaw = yaw - 0.0003;

    Eigen::Matrix4f T = worldtocurrent(xyz, rpy);
    pcl::transformPointCloud(pcl_LS_plum_object_points_, pcl_LM_plum_object_points_, T);
    pcl::transformPointCloud(pcl_LM_plum_object_points_, pcl_C_plum_object_points_, transform_matrix);

    for(int i = 0; i < 96; i++)
    {
        cv::Point3f LM_obj;
        cv::Point3f LM_blo;
        cv::Point3f C_obj;
        cv::Point3f C_blo;

        LM_obj.x = pcl_LM_plum_object_points_[i].x;
        LM_obj.y = pcl_LM_plum_object_points_[i].y;
        LM_obj.z = pcl_LM_plum_object_points_[i].z;
        C_obj.x = pcl_C_plum_object_points_[i].x;
        C_obj.y = pcl_C_plum_object_points_[i].y;
        C_obj.z = pcl_C_plum_object_points_[i].z;
        LM_blo.x = pcl_LM_plum_object_points_[i+96].x;
        LM_blo.y = pcl_LM_plum_object_points_[i+96].y;
        LM_blo.z = pcl_LM_plum_object_points_[i+96].z;
        C_blo.x = pcl_C_plum_object_points_[i+96].x;
        C_blo.y = pcl_C_plum_object_points_[i+96].y;
        C_blo.z = pcl_C_plum_object_points_[i+96].z;
        // if (i % 8 == 0){
        //     std::cout << "idx: " <<  i / 8  + 1<< std::endl;
        // }
        // std::cout << "LS.x: " << pcl_LS_plum_object_points_.points[i + 96].x << "  LS.y: " <<  pcl_LS_plum_object_points_.points[i + 96].y << "  LS.z: " <<  pcl_LS_plum_object_points_.points[i + 96].z << std::endl;
        // std::cout << "LM.x: " << LM_blo.x << "  LM.y: " << LM_blo.y << "  LM.z: " << LM_blo.z << std::endl;
        // std::cout << "C.x: " << C_blo.x << " C.y: " << C_blo.y << "  C.z: " << C_blo.z << std::endl;
        LM_object_points_[i] = LM_obj;
        LM_plum_blossom_points_[i] = LM_blo;
        C_object_points_[i] = C_obj;
        C_plum_blossom_points_[i] = C_blo;
    }

    {
        std::lock_guard<std::mutex> lock(mtx_tf_);
        cv::projectPoints(LM_object_points_, LC_rvec_vec, LC_tvec, K_, distCoeffs_, object_2d_points);
        cv::projectPoints(LM_plum_blossom_points_, LC_rvec_vec, LC_tvec, K_, distCoeffs_, plum_blossom_2d_points);
    }

    // 解析2d点,填充 box_2d_points_lists 和 plum_2d_points_lists
    std::vector<surface_2d_point> object_2d_front_points_;
    std::vector<surface_2d_point> object_2d_side_points_;
    std::vector<surface_2d_point> object_2d_up_points_;
    std::vector<surface_2d_point> plum_2d_front_points_;
    std::vector<surface_2d_point> plum_2d_side_points_;
    std::vector<surface_2d_point> plum_2d_up_points_;
    std::vector<std::vector<surface_2d_point>> object_2d_points_;
    std::vector<std::vector<surface_2d_point>> plum_2d_points_;
    for (int i = 0;i < 96;i +=8 ){
        int idx = i / 8 + 1;
        float object_front_depth = (calc_camera_distance(C_object_points_[i]) + calc_camera_distance(C_object_points_[i + 1]) + calc_camera_distance(C_object_points_[i + 2]) + calc_camera_distance(C_object_points_[i + 3])) / 4.0f;
        float object_left_depth = (calc_camera_distance(C_object_points_[i + 4]) + calc_camera_distance(C_object_points_[i]) + calc_camera_distance(C_object_points_[i + 3]) + calc_camera_distance(C_object_points_[i + 7])) / 4.0f; 
        float object_right_depth = (calc_camera_distance(C_object_points_[i + 1]) + calc_camera_distance(C_object_points_[i + 5]) + calc_camera_distance(C_object_points_[i + 6]) + calc_camera_distance(C_object_points_[i + 2])) / 4.0f; 
        float object_up_depth = (calc_camera_distance(C_object_points_[i + 4]) + calc_camera_distance(C_object_points_[i + 5]) + calc_camera_distance(C_object_points_[i + 1]) + calc_camera_distance(C_object_points_[i])) / 4.0f;
        object_2d_front_points_.push_back({idx,object_2d_points[i],object_2d_points[i + 1], object_2d_points[i + 2], object_2d_points[i + 3],object_front_depth});
        object_2d_up_points_.push_back({idx,object_2d_points[i + 4],object_2d_points[i + 5], object_2d_points[i + 1], object_2d_points[i],object_up_depth});
        if (object_left_depth > object_right_depth) {
            object_2d_side_points_.push_back({idx,object_2d_points[i + 1],object_2d_points[i + 5], object_2d_points[i + 6], object_2d_points[i + 2],object_right_depth});
        }else{object_2d_side_points_.push_back({idx,object_2d_points[i + 4],object_2d_points[i], object_2d_points[i + 3], object_2d_points[i + 7],object_left_depth});}

        float plum_front_depth = (calc_camera_distance(C_plum_blossom_points_[i]) + calc_camera_distance(C_plum_blossom_points_[i + 1]) + calc_camera_distance(C_plum_blossom_points_[i + 2])+ calc_camera_distance(C_plum_blossom_points_[i + 3])) / 4.0f;
        float plum_left_depth = (calc_camera_distance(C_plum_blossom_points_[i + 4]) + calc_camera_distance(C_plum_blossom_points_[i]) + calc_camera_distance(C_plum_blossom_points_[i + 3]) + calc_camera_distance(C_plum_blossom_points_[i + 7])) / 4.0f; 
        float plum_right_depth = (calc_camera_distance(C_plum_blossom_points_[i + 1]) + calc_camera_distance(C_plum_blossom_points_[i + 5]) + calc_camera_distance(C_plum_blossom_points_[i + 6]) + calc_camera_distance(C_plum_blossom_points_[i + 2])) / 4.0f; 
        float plum_up_depth = (calc_camera_distance(C_plum_blossom_points_[i + 4]) + calc_camera_distance(C_plum_blossom_points_[i + 5]) + calc_camera_distance(C_plum_blossom_points_[i + 1]) + calc_camera_distance(C_plum_blossom_points_[i])) / 4.0f;
        plum_2d_front_points_.push_back({idx,plum_blossom_2d_points[i],plum_blossom_2d_points[i + 1], plum_blossom_2d_points[i + 2], plum_blossom_2d_points[i + 3],plum_front_depth});
        plum_2d_up_points_.push_back({idx,plum_blossom_2d_points[i + 4],plum_blossom_2d_points[i + 5], plum_blossom_2d_points[i + 1], plum_blossom_2d_points[i],plum_up_depth});
        if (object_left_depth > object_right_depth) {
            plum_2d_side_points_.push_back({idx,plum_blossom_2d_points[i + 1],plum_blossom_2d_points[i + 5], plum_blossom_2d_points[i + 6], plum_blossom_2d_points[i + 2],plum_right_depth});
        }else{plum_2d_side_points_.push_back({idx,plum_blossom_2d_points[i + 4],plum_blossom_2d_points[i], plum_blossom_2d_points[i + 3], plum_blossom_2d_points[i + 7],plum_left_depth});}  
        
        // std::cout << "idx: " << idx << std::endl;
        // std::cout << "object:  front_depth: " << object_front_depth << ", left_depth: " << object_left_depth << ", right_depth: " << object_right_depth << ", up_depth: " << object_up_depth << std::endl;
        // std::cout << "plum:  front_depth: " << plum_front_depth << ", left_depth: " << plum_left_depth << ", right_depth: " << plum_right_depth << ", up_depth: " << plum_up_depth << std::endl;
    } 
    object_2d_points_.push_back(object_2d_front_points_);
    object_2d_points_.push_back(object_2d_side_points_);
    object_2d_points_.push_back(object_2d_up_points_);
    plum_2d_points_.push_back(plum_2d_front_points_);
    plum_2d_points_.push_back(plum_2d_side_points_);
    plum_2d_points_.push_back(plum_2d_up_points_);
    auto end1 = std::chrono::high_resolution_clock::now();

    auto start2 = std::chrono::high_resolution_clock::now();
    cv::Mat zbuffer = zbuffer_occlusion(object_2d_points_,plum_2d_points_,x,y,1.3,yaw,image);
    auto end2 = std::chrono::high_resolution_clock::now();

    auto start3 = std::chrono::high_resolution_clock::now();
    static std::unordered_set<int> processed_idxs;
    static std::vector<int> allow_idx_;
    if (processed_idxs.size() < 12) {
        allow_idx_ = extract_square_hsv_filterEmptyBoxes(
            zbuffer, image, processed_idxs
        );
    }
    auto end3 = std::chrono::high_resolution_clock::now();

    auto start4 = std::chrono::high_resolution_clock::now();

    draw_zbuffer(
        image,
        zbuffer,
        allow_idx_,
        object_2d_points_[0],  // 前表面
        object_2d_points_[1],  // 侧表面
        object_2d_points_[2]  // 上表面
    );

    roi_best_zbuffer(
    image,
    zbuffer,
    allow_idx_);
    
    // std::vector<int> yolo_result;
    // yolo_result.resize(12);
    // this->debug_image = roi_best_zbuffer_debug(this->getBestRoiImage(), yolo_result);
    //std::cout << "this->getBestRoiImage().size: " <<  this->getBestRoiImage().size() << std::endl;
    auto end4 = std::chrono::high_resolution_clock::now();

    // auto duration1 = std::chrono::duration_cast<std::chrono::milliseconds> (end1 - start1);
    // auto duration2 = std::chrono::duration_cast<std::chrono::milliseconds> (end2 - start2);
    // auto duration3 = std::chrono::duration_cast<std::chrono::milliseconds> (end3 - start3);
    // auto duration4 = std::chrono::duration_cast<std::chrono::milliseconds> (end4 - start4);
    // ROS_INFO("zbuffer_init.count: %ldms", duration1);
    // ROS_INFO("zbuffer_occlsion.count: %ldms", duration2);
    // ROS_INFO("zbuffer_filterEmptyBoxes.count: %ldms", duration3);
    // ROS_INFO("zbuffer_debug.count: %ldms", duration4);

    return this->getBestRoiImage();
}

size_t Ten_zbuffer::getBestRoiImage_size(const nav_msgs::Odometry msg,cv::Mat & image){
    return manage_odom_zbuffer_roi(msg,image).size();
}

float Ten_zbuffer::calc_camera_distance (cv::Point3f& p) {
    return sqrt(pow(p.x, 2) + pow(p.y, 2) + pow(p.z, 2));
};

// 核心函数1：extract_square_hsv_filterEmptyBoxes 实现
std::vector<int> Ten_zbuffer::extract_square_hsv_filterEmptyBoxes(
    const cv::Mat& zbuffer, 
    const cv::Mat& image,
    std::unordered_set<int>& processed_idxs
) {
    // 1. 转换图像到HSV空间
    cv::Mat hsv_image;
    cv::cvtColor(image, hsv_image, cv::COLOR_RGB2HSV);

    // 2. 直方图参数
    const int h_bins = 180;  // H通道：0-179
    const int s_bins = 256;  // S通道：0-255
    const int v_bins = 256;  // V通道：0-255

    // 3. 保存有效idx的HSV众数（仅像素数>800的idx）
    static std::unordered_map<int, std::tuple<int, int, int>> idx_hsv_map;

    // 4. 遍历每个idx的深度数据
    for (const auto& idx_depth_pair : idx_to_depth_) {
        int idx = idx_depth_pair.first;
        const std::vector<float>& depths = idx_depth_pair.second;

        // 跳过空深度/已处理的idx
        if (depths.empty()){
            ROS_INFO("Idx %d's depths not found", idx);
            continue;}
        if(processed_idxs.count(idx)){
            //ROS_INFO("Idx %d processed", idx);
            continue;
        }

        // 5. 收集该idx对应的所有像素坐标
        std::vector<cv::Point> idx_pixels;
        const float depth_tolerance = 0.001f;
        for (float target_depth : depths) {
            for (int y = 0; y < zbuffer.rows; ++y) {
                for (int x = 0; x < zbuffer.cols; ++x) {
                    float current_depth = zbuffer.at<float>(y, x);
                    if (current_depth != FLT_MAX && std::abs(current_depth - target_depth) < depth_tolerance) {
                        idx_pixels.emplace_back(x, y);
                    }
                }
            }
        }

        // 6. 跳过像素数≤1000的idx
        if (idx_pixels.size() <= 1000) {
            ROS_INFO("Idx %d pixels=%zu(<=1000), skip", idx, idx_pixels.size());
            continue;
        }

        // 7. 计算HSV直方图
        cv::Mat h_hist = cv::Mat::zeros(1, h_bins, CV_32S);
        cv::Mat s_hist = cv::Mat::zeros(1, s_bins, CV_32S);
        cv::Mat v_hist = cv::Mat::zeros(1, v_bins, CV_32S);

        for (const auto& pt : idx_pixels) {
            cv::Vec3b hsv = hsv_image.at<cv::Vec3b>(pt.y, pt.x);
            h_hist.at<int>(0, hsv[0])++;
            s_hist.at<int>(0, hsv[1])++;
            v_hist.at<int>(0, hsv[2])++;
        }

        // 8. 找直方图众数（出现次数最多的HSV值）
        cv::Point h_max_loc, s_max_loc, v_max_loc;
        cv::minMaxLoc(h_hist, nullptr, nullptr, nullptr, &h_max_loc);
        cv::minMaxLoc(s_hist, nullptr, nullptr, nullptr, &s_max_loc);
        cv::minMaxLoc(v_hist, nullptr, nullptr, nullptr, &v_max_loc);

        // 9. 保存众数HSV，并标记为已处理
        int h_mode = h_max_loc.x;
        int s_mode = s_max_loc.x;
        int v_mode = v_max_loc.x;
        idx_hsv_map[idx] = {h_mode, s_mode, v_mode};
        processed_idxs.insert(idx);

        //ROS_INFO("Idx %d HSV=(%d,%d,%d), pixels=%zu", idx, h_mode, s_mode, v_mode, idx_pixels.size());
    }

    // 10. 计算最终标准HSV（最小总差值）
    int result_h = 0, result_s = 0, result_v = 0;       // 标准值
    int min_total_h = INT_MAX, min_total_s = INT_MAX, min_total_v = INT_MAX;         // 最小总差值

    std::vector<int> idx_h_result, idx_s_result, idx_v_result;
    for (const auto& [idx, hsv_tuple] : idx_hsv_map) {
        idx_h_result.push_back(std::get<0>(hsv_tuple));
        idx_s_result.push_back(std::get<1>(hsv_tuple));
        idx_v_result.push_back(std::get<2>(hsv_tuple));
    }

    // 计算H通道标准值
    for(int i = 0; i < idx_h_result.size(); i++){
        int total_diff = 0;
        for (int j = 0; j < idx_h_result.size(); j++){
            total_diff += abs(idx_h_result[i] - idx_h_result[j]);
        }
        if(total_diff < min_total_h){
            min_total_h = total_diff;
            result_h = idx_h_result[i];
        }
    }

    // 计算S通道标准值
    for(int i = 0; i < idx_s_result.size(); i++){
        int total_diff = 0;
        for (int j = 0; j < idx_s_result.size(); j++){
            total_diff += abs(idx_s_result[i] - idx_s_result[j]);
        }
        if(total_diff < min_total_s){
            min_total_s = total_diff;
            result_s = idx_s_result[i];
        }
    }

    // 计算V通道标准值
    for(int i = 0; i < idx_v_result.size(); i++){
        int total_diff = 0;
        for (int j = 0; j < idx_v_result.size(); j++){
            total_diff += abs(idx_v_result[i] - idx_v_result[j]);
        }
        if(total_diff < min_total_v){
            min_total_v = total_diff;
            result_v = idx_v_result[i];
        }
    }

    // 11. 计算每个idx的得分
    std::vector<float> score = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f}; 
    for (const auto& [idx, hsv_tuple] : idx_hsv_map) {
        if (idx < 1 || idx > score.size()) continue;

        int h = std::get<0>(hsv_tuple);
        int s = std::get<1>(hsv_tuple);
        int v = std::get<2>(hsv_tuple);
        
        // 得分公式
        score[idx - 1] = std::max(0.0f,75.0f - abs(h - result_h) )
                       + std::max(0.0f,10.0f - 0.2f * abs(s - result_s))
                       + std::max(0.0f,15.0f - 0.2f * abs(v - result_v));
    }

    // 12. 返回得分大于阈值的idx
    std::vector<int> result_idx;
    for (int i = 0; i < score.size(); i++){
        //ROS_INFO("Idx %d score=%.2f", i+1, score[i]);
        if (score[i] >= 85){
            result_idx.push_back(i + 1);
        }
    }
    // std::cout << std::endl;
    // for (int i = 0;i < result_idx.size();i++){
    //     std::cout << result_idx[i] << " ";
    // }
    // std::cout << std::endl;

    // 异常处理
    if (result_idx.size() > 8){
        // 存储result_idx中每个索引对应的分数
        std::vector<std::pair<float, int>> score_idx_pairs;
        for (int idx : result_idx) {
            if (idx >= 1 && idx <= score.size()) {  // 校验索引有效性
                score_idx_pairs.emplace_back(score[idx - 1], idx);  // (分数, 索引)
            }
        }

        // 按分数降序排序（分数高的在前）
        std::sort(score_idx_pairs.begin(), score_idx_pairs.end(),
                [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
                    return a.first > b.first;
                });

        // 取前8个索引（若不足8个则全部保留）
        std::vector<int> top8_idx;
        for (size_t i = 0; i < score_idx_pairs.size() && i < 8; ++i) {
            top8_idx.push_back(score_idx_pairs[i].second);
        }

        // 更新result_idx为前8个结果
        result_idx = top8_idx;
    }
    return result_idx;
}

// 核心函数2：zbuffer_occlusion 实现
cv::Mat Ten_zbuffer::zbuffer_occlusion(
    std::vector<std::vector<surface_2d_point>> box_2d_points_lists,
    std::vector<std::vector<surface_2d_point>> plum_box_2d_points_lists,
    const float camera_y,
    const float camera_x,
    const float camera_z,
    const double yaw,
    const cv::Mat& image
) {
    // 1. 提取各表面2D点列表
    std::vector<surface_2d_point> front_2d = box_2d_points_lists[0];
    std::vector<surface_2d_point> side_2d = box_2d_points_lists[1];
    std::vector<surface_2d_point> up_2d = box_2d_points_lists[2];
    std::vector<surface_2d_point> plum_front_2d = plum_box_2d_points_lists[0];
    std::vector<surface_2d_point> plum_side_2d = plum_box_2d_points_lists[1];
    std::vector<surface_2d_point> plum_up_2d = plum_box_2d_points_lists[2];

    // 2. 防抖判断：相机位置/角度变化小时，复用上一帧深度缓冲
    const float pos_eps = 0.01f;
    const double rot_eps = 0.01;
    bool pos_unchanged = (std::abs(camera_x - last_camera_x_) < pos_eps && std::abs(camera_y - last_camera_y_) < pos_eps);
    bool yaw_unchanged = (std::abs(yaw - last_yaw_) < rot_eps);

    if (pos_unchanged && yaw_unchanged) {
        std::lock_guard<std::mutex> lock(mtx_last_zbuffer_);
        if (!last_zbuffer_.empty()) {
            is_move_ = true;
            //ROS_INFO("not move");
            return last_zbuffer_.clone();
        }
    } else {
        is_move_ = false;
    }

    // 3. 更新相机状态
    last_camera_x_ = camera_x;
    last_camera_y_ = camera_y;
    last_yaw_ = yaw;

    // 4. 检查图像尺寸有效性
    if (image.empty() || image.cols <= 0 || image.rows <= 0) {
        ROS_ERROR("Invalid image size: cols=%d, rows=%d", image.cols, image.rows);
        std::lock_guard<std::mutex> lock(mtx_last_zbuffer_);
        return last_zbuffer_.clone();
    }

    // 5. 初始化深度缓冲（初始值为最大浮点数，表示无深度）
    cv::Mat zbuffer = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
    cv::Mat box_zbuffer = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;

    // 6. 处理台阶（先绘制台阶，再绘制方块，实现遮挡）
    for (size_t i = 0; i < plum_side_2d.size(); i++) {
        if (i >= plum_front_2d.size() || i >= plum_up_2d.size()) break;

        auto& p_front = plum_front_2d[i];
        auto& p_side = plum_side_2d[i];
        auto& p_up = plum_up_2d[i];

        // 收集所有2D点坐标
        std::vector<cv::Point2f> all_points = {
            p_front.left_up, p_front.right_up, p_front.right_down, p_front.left_down,
            p_up.left_up, p_up.right_up, p_up.right_down, p_up.left_down,
            p_side.left_up, p_side.right_up, p_side.right_down, p_side.left_down
        };

        // 判断是否所有点都在图像外
        bool all_outside = true;
        for (const auto& pt : all_points) {
            if (pt.x >= 0 && pt.x < image.cols && pt.y >= 0 && pt.y < image.rows) {
                all_outside = false;
                break;
            }
        }
        if (all_outside) continue; // 完全在外部，跳过处理
        
        // 构建台阶轮廓
        std::vector<cv::Point> front_contour = {
            cv::Point(cvRound(p_front.left_up.x), cvRound(p_front.left_up.y)),
            cv::Point(cvRound(p_front.right_up.x), cvRound(p_front.right_up.y)),
            cv::Point(cvRound(p_front.right_down.x), cvRound(p_front.right_down.y)),
            cv::Point(cvRound(p_front.left_down.x), cvRound(p_front.left_down.y))
        };
        std::vector<cv::Point> up_contour = {
            cv::Point(cvRound(p_up.left_up.x), cvRound(p_up.left_up.y)),
            cv::Point(cvRound(p_up.right_up.x), cvRound(p_up.right_up.y)),
            cv::Point(cvRound(p_up.right_down.x), cvRound(p_up.right_down.y)),
            cv::Point(cvRound(p_up.left_down.x), cvRound(p_up.left_down.y))
        };
        std::vector<cv::Point> side_contour = {
            cv::Point(cvRound(p_side.left_up.x), cvRound(p_side.left_up.y)),
            cv::Point(cvRound(p_side.right_up.x), cvRound(p_side.right_up.y)),
            cv::Point(cvRound(p_side.right_down.x), cvRound(p_side.right_down.y)),
            cv::Point(cvRound(p_side.left_down.x), cvRound(p_side.left_down.y))
        };
        // ----------------------
        // 替换 fillPoly 为直接像素赋值（避免全局填充）
        // 计算轮廓的最小包围盒
        // cv::Rect contour_rect = cv::boundingRect(all_points);
        // cv::Mat contour_mask = cv::Mat::zeros(contour_rect.size(),CV_8UC1);
        // ---------------------
        // 填充台阶深度
        cv::Mat plum_tmp = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
        cv::fillPoly(plum_tmp, std::vector<std::vector<cv::Point>>{front_contour}, cv::Scalar(p_front.surface_depth));
        cv::fillPoly(plum_tmp, std::vector<std::vector<cv::Point>>{up_contour}, cv::Scalar(p_up.surface_depth));
        cv::fillPoly(plum_tmp, std::vector<std::vector<cv::Point>>{side_contour}, cv::Scalar(p_side.surface_depth));

        // 计算台阶像素范围
        std::vector<float> plum_x_value = {
            p_front.left_up.x, p_front.right_up.x, p_front.right_down.x, p_front.left_down.x,
            p_up.left_up.x, p_up.right_up.x, p_up.right_down.x, p_up.left_down.x,
            p_side.left_up.x, p_side.right_up.x, p_side.right_down.x, p_side.left_down.x
        };
        std::vector<float> plum_y_value = {
            p_front.left_up.y, p_front.right_up.y, p_front.right_down.y, p_front.left_down.y,
            p_up.left_up.y, p_up.right_up.y, p_up.right_down.y, p_up.left_down.y,
            p_side.left_up.y, p_side.right_up.y, p_side.right_down.y, p_side.left_down.y
        };
        auto plum_x_max = std::max_element(plum_x_value.begin(), plum_x_value.end());
        auto plum_x_min = std::min_element(plum_x_value.begin(), plum_x_value.end());
        auto plum_y_max = std::max_element(plum_y_value.begin(), plum_y_value.end());
        auto plum_y_min = std::min_element(plum_y_value.begin(), plum_y_value.end());

        // 合并到全局深度缓冲（近的深度覆盖远的）
        // 写入主zbuffer（台阶深度更近则更新）
        for (int row = int(*plum_y_min) - 5; row < int(*plum_y_max) + 5; ++row) {
            for (int col = int(*plum_x_min) - 5; col < int(*plum_x_max) + 5; ++col) {
                if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) continue;
                if (plum_tmp.at<float>(row, col) < zbuffer.at<float>(row, col)) {
                    zbuffer.at<float>(row, col) = plum_tmp.at<float>(row, col);
                }
            }
        }
    }

    // 7. 清空idx→深度映射
    idx_to_depth_.clear();

    // 8. 处理方块（后绘制，覆盖台阶的遮挡区域）
    for (size_t i = 0; i < side_2d.size(); i++) {
        if (i >= front_2d.size() || i >= up_2d.size()) break;

        auto& f_front = front_2d[i];
        auto& f_side = side_2d[i];
        auto& f_up = up_2d[i];

        // 收集所有2D点坐标
        std::vector<cv::Point2f> all_points = {
            f_front.left_up, f_front.right_up, f_front.right_down, f_front.left_down,
            f_up.left_up, f_up.right_up, f_up.right_down, f_up.left_down,
            f_side.left_up, f_side.right_up, f_side.right_down, f_side.left_down
        };

        // 判断是否所有点都在图像外
        bool all_outside = true;
        for (const auto& pt : all_points) {
            if (pt.x >= 0 && pt.x < image.cols && pt.y >= 0 && pt.y < image.rows) {
                all_outside = false;
                break;
            }
        }
        if (all_outside) continue; // 完全在外部，跳过处理
        // 构建方块轮廓
        std::vector<cv::Point> front_contour = {
            cv::Point(cvRound(f_front.left_up.x), cvRound(f_front.left_up.y)),
            cv::Point(cvRound(f_front.right_up.x), cvRound(f_front.right_up.y)),
            cv::Point(cvRound(f_front.right_down.x), cvRound(f_front.right_down.y)),
            cv::Point(cvRound(f_front.left_down.x), cvRound(f_front.left_down.y))
        };
        std::vector<cv::Point> up_contour = {
            cv::Point(cvRound(f_up.left_up.x), cvRound(f_up.left_up.y)),
            cv::Point(cvRound(f_up.right_up.x), cvRound(f_up.right_up.y)),
            cv::Point(cvRound(f_up.right_down.x), cvRound(f_up.right_down.y)),
            cv::Point(cvRound(f_up.left_down.x), cvRound(f_up.left_down.y))
        };
        std::vector<cv::Point> side_contour = {
            cv::Point(cvRound(f_side.left_up.x), cvRound(f_side.left_up.y)),
            cv::Point(cvRound(f_side.right_up.x), cvRound(f_side.right_up.y)),
            cv::Point(cvRound(f_side.right_down.x), cvRound(f_side.right_down.y)),
            cv::Point(cvRound(f_side.left_down.x), cvRound(f_side.left_down.y))
        };

        // 填充方块深度
        cv::Mat tmp = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
        cv::fillPoly(tmp, std::vector<std::vector<cv::Point>>{front_contour}, cv::Scalar(f_front.surface_depth));
        cv::fillPoly(tmp, std::vector<std::vector<cv::Point>>{up_contour}, cv::Scalar(f_up.surface_depth));
        cv::fillPoly(tmp, std::vector<std::vector<cv::Point>>{side_contour}, cv::Scalar(f_side.surface_depth));

        // 保存idx→深度映射
        idx_to_depth_[f_front.idx] = {f_front.surface_depth, f_up.surface_depth, f_side.surface_depth};

        // 计算方块像素范围
        std::vector<float> x_value = {
            f_front.left_up.x, f_front.right_up.x, f_front.right_down.x, f_front.left_down.x,
            f_up.left_up.x, f_up.right_up.x, f_up.right_down.x, f_up.left_down.x,
            f_side.left_up.x, f_side.right_up.x, f_side.right_down.x, f_side.left_down.x
        };
        std::vector<float> y_value = {
            f_front.left_up.y, f_front.right_up.y, f_front.right_down.y, f_front.left_down.y,
            f_up.left_up.y, f_up.right_up.y, f_up.right_down.y, f_up.left_down.y,
            f_side.left_up.y, f_side.right_up.y, f_side.right_down.y, f_side.left_down.y
        };
        auto x_max = std::max_element(x_value.begin(), x_value.end());
        auto x_min = std::min_element(x_value.begin(), x_value.end());
        auto y_max = std::max_element(y_value.begin(), y_value.end());
        auto y_min = std::min_element(y_value.begin(), y_value.end());

        // 合并到方块深度缓冲 + 全局深度缓冲
        for (int row = int(*y_min) - 5; row < int(*y_max) + 5; ++row) {
            for (int col = int(*x_min) - 5; col < int(*x_max) + 5; ++col) {
                if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) continue;
                if (tmp.at<float>(row, col) < zbuffer.at<float>(row, col)) {
                    zbuffer.at<float>(row, col) = tmp.at<float>(row, col);
                    box_zbuffer.at<float>(row, col) = tmp.at<float>(row, col);
                }
            }
        }
    }

    // 9. 保存当前深度缓冲（用于下一帧防抖）
    {
        std::lock_guard<std::mutex> lock(mtx_last_zbuffer_);
        box_zbuffer.copyTo(last_zbuffer_);
    }

    return box_zbuffer;
}


void Ten_zbuffer::roi_best_zbuffer(
    cv::Mat& image,
    const cv::Mat& zbuffer,
    const std::vector<int> allow_idx_lists
){
    // ========== 新增：核心防护 - 提前校验关键条件 ==========
    if (image.empty() || zbuffer.empty()) {
        ROS_WARN("Image or zbuffer is empty, skip ROI process");
        return;
    }
    // 校验图像类型（必须是8位3通道BGR，否则copyTo会出错）
    if (image.type() != CV_8UC3) {
        ROS_ERROR("Image type error! Expected CV_8UC3, got %d", image.type());
        return;
    }
    // 校验zbuffer类型（必须是32位浮点型）
    if (zbuffer.type() != CV_32F) {
        ROS_ERROR("Zbuffer type error! Expected CV_32F, got %d", zbuffer.type());
        return;
    }
    // 校验图像和zbuffer尺寸一致
    if (image.size() != zbuffer.size()) {
        ROS_ERROR("Image size(%dx%d) != Zbuffer size(%dx%d)", 
                  image.cols, image.rows, zbuffer.cols, zbuffer.rows);
        return;
    }

    // 2. 生成有效深度掩码（排除FLT_MAX）
    cv::Mat valid_mask = (zbuffer != FLT_MAX);
    if (cv::countNonZero(valid_mask) == 0) {
        ROS_WARN("No valid depth in zbuffer");
        return;
    }

    // 3. 深度值归一化（可选，不影响核心逻辑）
    double min_depth, max_depth;
    cv::minMaxLoc(zbuffer, &min_depth, &max_depth, nullptr, nullptr, valid_mask);
    cv::Mat depth_8u;
    zbuffer.convertTo(depth_8u, CV_8U, 255.0/(max_depth - min_depth), -255.0*min_depth/(max_depth - min_depth));
    depth_8u.setTo(0, ~valid_mask);

    // 4. 查找深度区域（idx_to_depth_中记录的深度）
    std::unordered_map<float, std::vector<cv::Point>> depth_regions;
    for (int y = 0; y < zbuffer.rows; ++y) {
        for (int x = 0; x < zbuffer.cols; ++x) {
            float d = zbuffer.at<float>(y, x);
            if (d == FLT_MAX) continue;
            depth_regions[d].emplace_back(x, y);
        }
    }

    // ========== 修复点2：深度映射表构建（避免迭代器失效） ==========
    std::unordered_set<float> valid_depths;
    std::unordered_map<float,int> depth_to_idx;
    std::vector<int> to_erase_idxs;  // 先收集要删除的idx，避免遍历中erase导致迭代器失效

    // 第一步：收集有效深度和待删除的idx
    for (const auto& [idx, depths] : idx_to_depth_) {
        if (std::find(allow_idx_lists.begin(), allow_idx_lists.end(), idx) != allow_idx_lists.end()) {
            for (float d : depths) {
                valid_depths.insert(d);
                depth_to_idx[d] = idx;  // 注：若一个深度对应多个idx，后插入的会覆盖前一个
            }
        } else {
            to_erase_idxs.push_back(idx);  // 收集待删除的idx
        }
    }

    // 第二步：批量删除无效idx（遍历结束后操作，避免迭代器失效）
    for (int idx : to_erase_idxs) {
        idx_to_depth_.erase(idx);
    }

    // ========== 修复点3：遍历深度区域，核心逻辑加固 ==========
    for (const auto& [depth, points] : depth_regions) {
        if (points.empty()) continue;

        // 5.1 检查当前深度是否有效
        bool is_valid = false;
        float matched_depth = -1.0f;
        for (float valid_d : valid_depths) {
            if (std::abs(depth - valid_d) < 1e-4) {
                is_valid = true;
                matched_depth = valid_d;
                break;
            }
        }
        if (!is_valid) continue;

        // ========== 修复点4：检查depth_to_idx中是否存在matched_depth ==========
        if (depth_to_idx.find(matched_depth) == depth_to_idx.end()) {
            ROS_WARN("Depth %.4f not found in depth_to_idx, skip", matched_depth);
            continue;
        }
        int now_manage_idx = depth_to_idx[matched_depth];

        // 5.3 过滤点集 + 计算边界（修复边界初始化值错误）
        std::vector<cv::Point> valid_points;
        int x_min = INT_MAX, x_max = INT_MIN;  // 修复：初始值改为INT_MAX/INT_MIN，避免image.cols初始值导致边界无效
        int y_min = INT_MAX, y_max = INT_MIN;

        for (const auto& p : points) {
            // 手动clamp，避免越界
            int x = p.x < 0 ? 0 : (p.x >= image.cols ? image.cols - 1 : p.x);
            int y = p.y < 0 ? 0 : (p.y >= image.rows ? image.rows - 1 : p.y);
            valid_points.emplace_back(x, y);

            // 更新边界
            x_min = std::min(x_min, x);
            x_max = std::max(x_max, x);
            y_min = std::min(y_min, y);
            y_max = std::max(y_max, y);
        }

        // 边界有效性校验
        if (x_min > x_max || y_min > y_max || x_min < 0 || y_min < 0 || x_max >= image.cols || y_max >= image.rows) {
            ROS_WARN("Invalid ROI boundary: x[%d,%d], y[%d,%d], skip", x_min, x_max, y_min, y_max);
            continue;
        }

        // 5.4 比较点集大小（修复：首次访问best_roi_count时的默认值问题）
        int current_count = static_cast<int>(valid_points.size());
        auto count_it = best_roi_count.find(now_manage_idx);
        if (count_it != best_roi_count.end()) {
            if (current_count <= count_it->second) continue;
        }
        // 首次访问时，best_roi_count[now_manage_idx]会默认初始化0，无需额外处理

        // 5.5 创建掩码（加固：掩码尺寸和image一致）
        cv::Mat roi_mask = cv::Mat::zeros(image.size(), CV_8UC1);
        for (const auto& p : valid_points) {
            if (p.y >= 0 && p.y < roi_mask.rows && p.x >= 0 && p.x < roi_mask.cols) {
                roi_mask.at<uchar>(p.y, p.x) = 255;
            }
        }

        // 5.6 裁剪ROI（加固：校验roi_rect有效性）
        cv::Rect roi_rect(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
        // 再次校验roi_rect，避免宽高为负
        if (roi_rect.width <= 0 || roi_rect.height <= 0 || 
            roi_rect.x + roi_rect.width > image.cols || 
            roi_rect.y + roi_rect.height > image.rows) {
            ROS_WARN("Invalid ROI rect: x=%d, y=%d, w=%d, h=%d, skip", 
                      roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height);
            continue;
        }

        cv::Mat image_roi = image(roi_rect).clone();
        cv::Mat mask_roi = roi_mask(roi_rect).clone();

        // 5.7 拷贝有效区域（加固：检查mask_roi和image_roi尺寸一致）
        if (image_roi.size() != mask_roi.size()) {
            ROS_WARN("Image ROI size != Mask ROI size, skip");
            continue;
        }
        cv::Mat crop_roi = cv::Mat::zeros(image_roi.size(), image_roi.type());
        image_roi.copyTo(crop_roi, mask_roi);

        // 5.8 转为正方形（无修改，逻辑安全）
        int roi_width = crop_roi.cols;
        int roi_height = crop_roi.rows;
        int max_side = std::max(roi_width, roi_height);
        cv::Mat square_roi = cv::Mat::zeros(max_side, max_side, crop_roi.type());
        int x_offset = (max_side - roi_width) / 2;
        int y_offset = (max_side - roi_height) / 2;
        cv::Rect paste_rect(x_offset, y_offset, roi_width, roi_height);

        // 最后一次校验paste_rect
        if (paste_rect.x >= 0 && paste_rect.y >= 0 && 
            paste_rect.x + paste_rect.width <= square_roi.cols && 
            paste_rect.y + paste_rect.height <= square_roi.rows) {
            crop_roi.copyTo(square_roi(paste_rect));
        } else {
            ROS_WARN("Invalid paste rect for square ROI, skip");
            continue;
        }

        // 5.9 更新最优ROI
        best_roi_count[now_manage_idx] = current_count;
        best_roi_image[now_manage_idx] = square_roi.clone();

        ROS_INFO("Idx %d: crop ROI (w=%d, h=%d) → square (size=%d), points=%zu",
                now_manage_idx, roi_width, roi_height, max_side, valid_points.size());
    }
}

// 调试函数：draw_zbuffer 实现
void Ten_zbuffer::draw_zbuffer(
    cv::Mat& image, 
    const cv::Mat& zbuffer, 
    const std::vector<int> allow_idx_lists,
    const std::vector<surface_2d_point>& front_2d,
    const std::vector<surface_2d_point>& side_2d,
    const std::vector<surface_2d_point>& up_2d
) {
    // 1. 检查输入有效性
    if (image.empty() || zbuffer.empty()) {
        ROS_WARN("Image or zbuffer is empty, skip draw");
        return;
    }

    // 2. 生成有效深度掩码（排除FLT_MAX）
    cv::Mat valid_mask = (zbuffer != FLT_MAX);
    if (cv::countNonZero(valid_mask) == 0) {
        ROS_WARN("No valid depth in zbuffer");
        return;
    }

    // 3. 深度值归一化到0-255（用于可视化）
    double min_depth, max_depth;
    cv::minMaxLoc(zbuffer, &min_depth, &max_depth, nullptr, nullptr, valid_mask);
    cv::Mat depth_8u;
    zbuffer.convertTo(depth_8u, CV_8U, 255.0/(max_depth - min_depth), -255.0*min_depth/(max_depth - min_depth));
    depth_8u.setTo(0, ~valid_mask);

    // 4. 绘制深度区域（非idx_to_depth_中记录的深度）
    std::unordered_map<float, std::vector<cv::Point>> depth_regions;
    for (int y = 0; y < zbuffer.rows; ++y) {
        for (int x = 0; x < zbuffer.cols; ++x) {
            float d = zbuffer.at<float>(y, x);
            if (d == FLT_MAX) continue;
            depth_regions[d].emplace_back(x, y);
        }
    }

    // 从idx_to_depth_收集所有有效的深度值
    std::unordered_set<float> valid_depths;
    for (const auto& [idx, depths] : idx_to_depth_) {
        if (std::find(allow_idx_lists.begin(),allow_idx_lists.end(),idx) != allow_idx_lists.end()){
        for (float d : depths) {
            valid_depths.insert(d);
        }
        }
    }

    // 遍历每个深度值的区域，判断是否为有效深度
    for (const auto& [d, points] : depth_regions) {
        // 跳过idx_to_depth_中记录的有效深度
        bool is_valid = false;
        for (float valid_d : valid_depths) {
            if (std::abs(d - valid_d) < 1e-4) {  // 浮点数精度容错
                is_valid = true;
                break;
            }
        }
        if (!is_valid) continue;

        // 绘制区域轮廓
        cv::Mat region_mask = cv::Mat::zeros(zbuffer.size(), CV_8U);
        for (const auto& p : points) region_mask.at<uchar>(p) = 255;
        
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(region_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        for (const auto& cnt : contours) {
            if (cnt.size() < 4) continue;
            cv::polylines(image, cnt, true, cv::Scalar(0, 0, 255), 2); // 红色轮廓标记异常区域
        }
    }

    // 5. 收集每个idx的所有2D点，用于绘制索引
    std::unordered_map<int, std::vector<cv::Point2f>> idx_points;
    auto collect_points = [&](const std::vector<surface_2d_point>& pts) {
        for (const auto& p : pts) {
            idx_points[p.idx].push_back(p.left_up);
            idx_points[p.idx].push_back(p.right_up);
            idx_points[p.idx].push_back(p.right_down);
            idx_points[p.idx].push_back(p.left_down);
        }
    };
    collect_points(front_2d);
    collect_points(side_2d);
    collect_points(up_2d);

    // 6. 绘制方块索引（绿色文字）
    cv::Scalar text_color(0, 255, 0);
    int font = cv::FONT_HERSHEY_SIMPLEX;
    for (const auto& [idx, pts] : idx_points) {
        // 计算中心点
        cv::Point2f center(0, 0);
        for (const auto& p : pts) center += p;
        // 修复：将size_t转为float，避免运算符不匹配
        center /= static_cast<float>(pts.size());

        // 绘制文字（避免越界）
        int x = std::max(10, std::min(image.cols-50, (int)center.x));
        int y = std::max(20, std::min(image.rows-10, (int)center.y));
        cv::putText(image, std::to_string(idx), cv::Point(x, y), font, 0.8, text_color, 2);
    }
}

// 调试函数：roi_best_zbuffer_debug 实
cv::Mat Ten_zbuffer::roi_best_zbuffer_debug(const std::unordered_map<int, cv::Mat>& best_roi_image,const std::vector<int> yolo_result) {
    // 1. 配置固定参数（避免魔法数）
    const int SINGLE_SIZE = 160;    // 单个图的目标尺寸（160×160）
    const int STITCH_WIDTH = 640;   // 拼接后总宽度（160×4）
    const int STITCH_HEIGHT = 480;  // 拼接后总高度（160×3）
    const int COL_NUM = 4;          // 每行列数
    const int ROW_NUM = 3;          // 总行数
    const int TOTAL_IMGS = 12;      // 总图片数（1-12）

    // 2. 初始化12个160×160的全黑图（BGR格式，CV_8UC3）
    std::vector<cv::Mat> roi_images(TOTAL_IMGS, cv::Mat::zeros(SINGLE_SIZE, SINGLE_SIZE, CV_8UC3));

    // 3. 填充有效ROI图（idx1-12）
    for (int idx = 1; idx <= TOTAL_IMGS; ++idx) {
        // 计算当前idx在vector中的索引（idx1→0，idx12→11）
        int vec_idx = idx - 1;

        // 检查best_roi_image中是否有该idx的图
        auto it = best_roi_image.find(idx);
        if (it != best_roi_image.end() && !it->second.empty()) {
            const cv::Mat& src_img = it->second;
            // 校验源图类型（必须是BGR格式，避免拼接颜色异常）
            if (src_img.type() != CV_8UC3) {
                ROS_WARN("Idx %d image type error (not CV_8UC3), use black image", idx);
                continue;
            }
            // resize为160×160（原正方形，无畸变）
            cv::Mat resized_img;
            cv::resize(src_img, resized_img, cv::Size(SINGLE_SIZE, SINGLE_SIZE), 0, 0, cv::INTER_LINEAR);
            // 替换初始化的黑图
            roi_images[vec_idx] = resized_img.clone();
            cv::putText(roi_images[vec_idx], std::to_string(yolo_result[vec_idx]), cv::Point(roi_images[vec_idx].cols - 10 , roi_images[vec_idx].rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        } else {
            ROS_DEBUG("Idx %d no image in best_roi_image, use black image", idx);
        }
    }

    // 4. 拼接成640×480的大图
    cv::Mat stitch_img = cv::Mat::zeros(STITCH_HEIGHT, STITCH_WIDTH, CV_8UC3);
    for (int row = 0; row < ROW_NUM; ++row) {
        for (int col = 0; col < COL_NUM; ++col) {
            // 计算当前小图在vector中的索引
            int vec_idx = row * COL_NUM + col;
            if (vec_idx >= TOTAL_IMGS) break; // 防止越界（理论上不会触发）

            // 计算当前小图在拼接图中的位置
            int x = col * SINGLE_SIZE;
            int y = row * SINGLE_SIZE;
            cv::Rect roi_rect(x, y, SINGLE_SIZE, SINGLE_SIZE);

            // 将小图复制到拼接图对应位置
            roi_images[vec_idx].copyTo(stitch_img(roi_rect));
        }
    }

    return stitch_img;
}

cv::Mat roi_best_zbuffer_debug2(const std::unordered_map<int, cv::Mat>& best_roi_image,const std::vector<int> yolo_result) {
    // 1. 配置固定参数（避免魔法数）
    const int SINGLE_SIZE = 160;    // 单个图的目标尺寸（160×160）
    const int STITCH_WIDTH = 640;   // 拼接后总宽度（160×4）
    const int STITCH_HEIGHT = 480;  // 拼接后总高度（160×3）
    const int COL_NUM = 4;          // 每行列数
    const int ROW_NUM = 3;          // 总行数
    const int TOTAL_IMGS = 12;      // 总图片数（1-12）

    // 2. 初始化12个160×160的全黑图（BGR格式，CV_8UC3）
    std::vector<cv::Mat> roi_images(TOTAL_IMGS, cv::Mat::zeros(SINGLE_SIZE, SINGLE_SIZE, CV_8UC3));

    // 3. 填充有效ROI图（idx1-12）
    for (int idx = 1; idx <= TOTAL_IMGS; ++idx) {
        // 计算当前idx在vector中的索引（idx1→0，idx12→11）
        int vec_idx = idx - 1;

        // 检查best_roi_image中是否有该idx的图
        auto it = best_roi_image.find(idx);
        if (it != best_roi_image.end() && !it->second.empty()) {
            const cv::Mat& src_img = it->second;
            // 校验源图类型（必须是BGR格式，避免拼接颜色异常）
            if (src_img.type() != CV_8UC3) {
                ROS_WARN("Idx %d image type error (not CV_8UC3), use black image", idx);
                continue;
            }
            // resize为160×160（原正方形，无畸变）
            cv::Mat resized_img;
            cv::resize(src_img, resized_img, cv::Size(SINGLE_SIZE, SINGLE_SIZE), 0, 0, cv::INTER_LINEAR);
            // 替换初始化的黑图
            roi_images[vec_idx] = resized_img.clone();
            cv::putText(roi_images[vec_idx], std::to_string(yolo_result[vec_idx]), cv::Point(roi_images[vec_idx].cols - 10 , roi_images[vec_idx].rows - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 255, 0), 2);
        } else {
            ROS_DEBUG("Idx %d no image in best_roi_image, use black image", idx);
        }
    }

    // 4. 拼接成640×480的大图
    cv::Mat stitch_img = cv::Mat::zeros(STITCH_HEIGHT, STITCH_WIDTH, CV_8UC3);
    for (int row = 0; row < ROW_NUM; ++row) {
        for (int col = 0; col < COL_NUM; ++col) {
            // 计算当前小图在vector中的索引
            int vec_idx = row * COL_NUM + col;
            if (vec_idx >= TOTAL_IMGS) break; // 防止越界（理论上不会触发）

            // 计算当前小图在拼接图中的位置
            int x = col * SINGLE_SIZE;
            int y = row * SINGLE_SIZE;
            cv::Rect roi_rect(x, y, SINGLE_SIZE, SINGLE_SIZE);

            // 将小图复制到拼接图对应位置
            roi_images[vec_idx].copyTo(stitch_img(roi_rect));
        }
    }

    return stitch_img;
}

} // namespace Ten
#endif