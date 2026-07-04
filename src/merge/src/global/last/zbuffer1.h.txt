#ifndef __ZBUFFER_H_
#define __ZBUFFER_H_

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>


namespace Ten
{

#define L_ 1.2f                 // 台阶长度
#define H_ 0.2f                 // 台阶高度
#define lx1_ 0.425f             // 台阶到方块的间距
#define ly1_ 0.425f             // 台阶到方块的间距
#define lh_ 0.35f               // 方块的长度
#define X_  3.2f         //3.2f                 // 初始位置到梅花林1号位置边角的x轴距离
#define Y_   -1.2 //-0.83f        //-1.2f                // 初始位置到梅花林1号位置边角的y轴距离
#define LIDAR_HEIGHT_   0 //0.79f    //0         // 雷达的高度 
#define box_half_length_ 0.175  // 方块长度的一半
#define step_half_length_ 0.6   // 台阶水平边长的一半
#define offset_x_ 0             // x方向上的偏移量
#define offset_y_ 0             // y方向上的偏移量
#define offset_z_ 0             // z方向上的偏移量

// struct surface_2d_point {        
//     int idx;                       // 对应方块索引
//     cv::Point2f left_up;           // 左上2D点
//     cv::Point2f right_up;          // 右上2D点
//     cv::Point2f right_down;        // 右下2D点
//     cv::Point2f left_down;         // 左下2D点
//     float surface_depth;           // 该表面的深度值
// };

struct box{
    int idx;                             // 表示位置的下标索引
    cv::Mat roi_image;                   // 裁剪出来的roi图片
    int cls = 0;                             // 识别类别
    float confidence = 0.0f;                // 自信度
    int zbuffer_flag = 0;                    // zbuffer是否处理的标志位， 0 表示未处理， 1 表示已处理， -1 表示异常
    int exist_flag = -1;                      // 是否筛空的标志位， 0 表示空， 1 表示有方块， -1 表示未处理
    int roi_valid_flag = 0;                   // 用于表示当前的roi 图像是否有效
};

// 长方体（正方体）定义：8个世界坐标角点 + OBB包围盒（用于遮挡检测）
// struct Cube {
//     std::vector<Eigen::Vector3d> world_corners; // 8个角点（世界坐标系）
//     Eigen::Vector3d center;                     // 中心（OBB用）
//     Eigen::Vector3d axes[3];                    // OBB正交轴（世界坐标系）
//     Eigen::Vector3d extents;                    // OBB半长度（世界坐标系）
//     int valid[8] = {0};                         //0没有角点， 1有角点
// };



}


#endif

