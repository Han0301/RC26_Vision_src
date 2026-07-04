#ifndef _Ten_occlusion_handing_H_
#define _Ten_occlusion_handing_H_
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
    int roi_valid_flag = 0;                   // 用于表示当前的roi 图像是否有效
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

    // 无参构造函数
    init_3d_box()
    :pcl_LM_plum_object_points_(new pcl::PointCloud<pcl::PointXYZ>()),
    pcl_C_plum_object_points_(new pcl::PointCloud<pcl::PointXYZ>())
    {
        object_plum_2d_points_.resize(96*2);
        W_object_plum_points_.resize(96*2);
        LM_object_plum_points_.resize(96*2);
        C_object_plum_points_.resize(96*2);

        // 初始化 box_lists_
        box_lists_.resize(12);

        for(int i = 0; i < 12; i++)
        {
            box_lists_[i].idx = i + 1;
            box_lists_[i].roi_image = cv::Mat::zeros(160, 160, CV_8UC3);
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

        // 初始化 LM_object_plum_points_，C_object_plum_points_，pcl_LM_plum_object_points_，pcl_C_plum_object_points_
        for(int i = 0; i < 96 * 2; i++){
            //减雷达高度
            LM_object_plum_points_[i] = cv::Point3f(W_object_plum_points_[i].x, W_object_plum_points_[i].y, W_object_plum_points_[i].z - LIDAR_HEIGHT_);
            C_object_plum_points_[i]  = cv::Point3f(W_object_plum_points_[i].x, W_object_plum_points_[i].y, W_object_plum_points_[i].z);
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

class Ten_occlusion_handing
{
public:
    /**
     * @brief 由用户自己设置存在方块的数组（zbuffer矩阵将仅更新 非空方块 且 感兴趣的 位置处的方块深度信息）
     * @param exist_boxes 输入 int， 12 数组， 1 表示存在， 0 表示不存在， -1 表示异常/未处理
     */
    void set_exist_boxes(int exist_boxes[12])
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for(int i = 0; i < 12; i++){exist_boxes_[i] = exist_boxes[i];}
    }

    /**
     * @brief 由用户自己设置感兴趣的方块的数组（zbuffer矩阵将仅更新 非空方块 且 感兴趣的 位置处的方块深度信息）
     * @param interested_boxes 输入 int， 12 数组， 1 表示感兴趣， 0 表示不感兴趣
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
     * @param box_lists 方块的列表
     */
    void set_box_lists_(
        const cv::Mat& image,     
        const std::vector<cv::Point3f>& C_object_plum_points,
        const std::vector<cv::Point2f>& object_plum_2d_points,
        std::vector<box>& box_lists);

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
     * @brief 拼接调试图像
     * @param box_lists  方块的列表，std::vector<box>
     * @param debug_best_roi_image 必须为 cv::Mat::zeros(480, 640, CV_8UC3) !!!
     */
    void set_debug_roi_image(
        std::vector<Ten::box>box_lists,
        cv::Mat& debug_best_roi_image);
private:
    int exist_boxes_[12] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};
    int interested_boxes_[12]= {1,1,1,1,1,1,1,1,1,1,1,1};
    mutable std::mutex mtx_;

    // 功能函数1： 根据该面的四个角点 来 计算该面的平均深度(仅在 set_surface_2d_point 函数中被调用)
    float cal_distance (const cv::Point3f& p1,const cv::Point3f& p2,const cv::Point3f& p3,const cv::Point3f& p4) {
        float total_depth = 0.0f; 
        total_depth += sqrt(powf(p1.x, 2) + powf(p1.y, 2) + powf(p1.z, 2));
        total_depth += sqrt(powf(p2.x, 2) + powf(p2.y, 2) + powf(p2.z, 2));
        total_depth += sqrt(powf(p3.x, 2) + powf(p3.y, 2) + powf(p3.z, 2));
        total_depth += sqrt(powf(p4.x, 2) + powf(p4.y, 2) + powf(p4.z, 2));
        return (total_depth / 4.0f);
    }
    // 功能函数2： 根据深度信息 更新2d点列表 object_2d,plum_2d (仅在 set_box_lists_ 函数中被调用)
    void set_surface_2d_point(
        const std::vector<cv::Point3f>& C_object_plum_points,
        const std::vector<cv::Point2f>& object_plum_2d_points,
        std::vector<surface_2d_point>& surface_2d,
        std::string label)
    {   
        int j = 0;
        if (label == "plum")
        {
            j = 96;
        }
        for (int i = 0;i < 96;i +=8)
        {
            int idx = i / 8 + 1;
            float front_depth = cal_distance(C_object_plum_points[j + i], C_object_plum_points[j + i + 1], C_object_plum_points[j + i + 2],C_object_plum_points[j + i + 3]);
            float back_depth = cal_distance(C_object_plum_points[j + i + 4], C_object_plum_points[j + i + 5], C_object_plum_points[j + i + 6],C_object_plum_points[j + i + 7]);
            float left_depth = cal_distance(C_object_plum_points[j + i + 4], C_object_plum_points[j + i], C_object_plum_points[j + i + 3],C_object_plum_points[j + i + 7]);
            float right_depth = cal_distance(C_object_plum_points[j + i + 1], C_object_plum_points[j + i + 5], C_object_plum_points[j + i + 6],C_object_plum_points[j + i + 2]);
            float up_depth = cal_distance(C_object_plum_points[j + i + 4], C_object_plum_points[j + i + 5], C_object_plum_points[j + i + 1],C_object_plum_points[j + i]);
            float down_depth = cal_distance(C_object_plum_points[j + i + 6], C_object_plum_points[j + i + 7], C_object_plum_points[j + i + 3],C_object_plum_points[j + i + 2]);
            if (front_depth < back_depth)
            {
                surface_2d.push_back({idx, object_plum_2d_points[j + i], object_plum_2d_points[j + i + 1], object_plum_2d_points[j + i + 2],object_plum_2d_points[j + i + 3],front_depth});
            }
            else
            {
                surface_2d.push_back({idx, object_plum_2d_points[j + i + 4], object_plum_2d_points[j + i + 5], object_plum_2d_points[j + i + 6],object_plum_2d_points[j + i + 7],back_depth});
            }
            if (left_depth < right_depth)
            {
                surface_2d.push_back({idx, object_plum_2d_points[j + i + 4], object_plum_2d_points[j + i], object_plum_2d_points[j + i + 3],object_plum_2d_points[j + i + 7],left_depth});
            }
            else
            {
                surface_2d.push_back({idx, object_plum_2d_points[j + i + 1], object_plum_2d_points[j + i + 5], object_plum_2d_points[j + i + 6],object_plum_2d_points[j + i + 2],right_depth});
            }
            if (up_depth < down_depth)
            {
                surface_2d.push_back({idx, object_plum_2d_points[j + i + 4], object_plum_2d_points[j + i + 5], object_plum_2d_points[j + i + 1],object_plum_2d_points[j + i],up_depth});
            }
            else
            {
                surface_2d.push_back({idx, object_plum_2d_points[j + i + 6], object_plum_2d_points[j + i + 7], object_plum_2d_points[j + i + 3],object_plum_2d_points[j + i + 2],down_depth});
            }
        }


    }
    // 功能函数3： 收集方块所有2D点坐标， 并判断方块的所有点是否都在图像外(仅在 set_box_lists_ 函数中被调用)
    bool set_all_outside(
        const Ten::surface_2d_point& front_2d, 
        const Ten::surface_2d_point& side_2d, 
        const Ten::surface_2d_point& up_2d,
        const int cols,
        const int rows,
        std::vector<cv::Point2f>& all_points
    )
    {
        all_points = {
            front_2d.left_up, front_2d.right_up, front_2d.right_down, front_2d.left_down,
            side_2d.left_up, side_2d.right_up, side_2d.right_down, side_2d.left_down,
            up_2d.left_up, up_2d.right_up, up_2d.right_down, up_2d.left_down
        };
        bool all_outside = true;
        for (const auto& pt : all_points) {
            if (pt.x >= 0 && pt.x < cols && pt.y >= 0 && pt.y < rows) {
                all_outside = false;
                break;
            }
        }
        return all_outside;
    }
    // 功能函数4：构建台阶轮廓，填充深度到临时矩阵 temp(仅在 set_box_lists_ 函数中被调用)
    void set_temp(
        const Ten::surface_2d_point& front_2d, 
        const Ten::surface_2d_point& side_2d, 
        const Ten::surface_2d_point& up_2d,
        cv::Mat& temp
    )
    {
        std::vector<cv::Point> front_contour = {
            cv::Point(cvRound(front_2d.left_up.x), cvRound(front_2d.left_up.y)),
            cv::Point(cvRound(front_2d.right_up.x), cvRound(front_2d.right_up.y)),
            cv::Point(cvRound(front_2d.right_down.x), cvRound(front_2d.right_down.y)),
            cv::Point(cvRound(front_2d.left_down.x), cvRound(front_2d.left_down.y))
        };
        std::vector<cv::Point> side_contour = {
            cv::Point(cvRound(side_2d.left_up.x), cvRound(side_2d.left_up.y)),
            cv::Point(cvRound(side_2d.right_up.x), cvRound(side_2d.right_up.y)),
            cv::Point(cvRound(side_2d.right_down.x), cvRound(side_2d.right_down.y)),
            cv::Point(cvRound(side_2d.left_down.x), cvRound(side_2d.left_down.y))
        };
        std::vector<cv::Point> up_contour = {
            cv::Point(cvRound(up_2d.left_up.x), cvRound(up_2d.left_up.y)),
            cv::Point(cvRound(up_2d.right_up.x), cvRound(up_2d.right_up.y)),
            cv::Point(cvRound(up_2d.right_down.x), cvRound(up_2d.right_down.y)),
            cv::Point(cvRound(up_2d.left_down.x), cvRound(up_2d.left_down.y))
        };
        cv::fillPoly(temp, std::vector<std::vector<cv::Point>>{front_contour}, cv::Scalar(front_2d.surface_depth));
        cv::fillPoly(temp, std::vector<std::vector<cv::Point>>{side_contour}, cv::Scalar(side_2d.surface_depth));
        cv::fillPoly(temp, std::vector<std::vector<cv::Point>>{up_contour}, cv::Scalar(up_2d.surface_depth));
    }
    // 功能函数5： 计算像素范围(仅在 set_box_lists_ 函数中被调用)
    void cal_points_range(
        const std::vector<cv::Point2f>& all_points,
        float& plum_x_min,
        float& plum_y_min,
        float& plum_x_max,
        float& plum_y_max
    )
    {
        for(const auto& p : all_points) {
            if(p.x > plum_x_max) plum_x_max = p.x;
            if(p.x < plum_x_min) plum_x_min = p.x;
            if(p.y > plum_y_max) plum_y_max = p.y;
            if(p.y < plum_y_min) plum_y_min = p.y;
        }
    }
    // 功能函数6： 不更新 roi_image 的条件(仅在 set_box_lists_ 函数中被调用)
    bool is_update_image(
        const std::vector<box>& box_lists,
        const std::vector<cv::Point2f>& valid_max_points,
        const int exist_boxes[12],
        const int interested_boxes[12],
        const int i
    )
    {
        bool update_image = true;
        if (valid_max_points.empty() || valid_max_points.size() <= 800) {
            std::cout << "🤡in func: set_box_lists_ 4.2, box idx= " <<  i / 3 + 1 <<  ", valid_max_points is empty or size() = " <<  valid_max_points.size()<< " < 600, skip crop ROI" << std::endl;
            // box_lists[i].zbuffer_flag = -1; // 标记异常
            update_image = false;
        }
        else if (!(exist_boxes[i / 3] != 0 && interested_boxes[i / 3] == 1))
        {
            std::cout << "🤡in func: set_box_lists_ 4.2,!(exist_boxes[i] != 0 && interested_boxes[i] == 1), skip crop ROI, box idx= " <<  i / 3 + 1 << std::endl;
            update_image = false;
        }
        else if (box_lists[i / 3].zbuffer_flag == -1)
        {
            std::cout << "🤡in func: set_box_lists_ 4.2,box_lists[i].zbuffer_flag == -1, skip crop ROI, box idx= " <<  i / 3 + 1 << std::endl;
            update_image = false;
        } 
        return update_image;
    }
};
    extern Ten::Ten_occlusion_handing _OCCLUSION_HANDING_;
    extern Ten::init_3d_box _INIT_3D_BOX_;
}       // namespace Ten
#endif