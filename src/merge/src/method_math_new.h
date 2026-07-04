#ifndef __METHOD_MATH_NEW_H_
#define __METHOD_MATH_NEW_H_
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
// 引入tf库，用于四元数转欧拉角
#include <tf/transform_datatypes.h>
// 引入数学库，用于弧度转角度（可选）
#include <cmath>
#include <eigen3/Eigen/Core>       // 核心矩阵/向量
#include <eigen3/Eigen/Geometry>   // 几何变换（旋转、平移）
#include <opencv2/opencv.hpp>
#include <sensor_msgs/PointCloud2.h>
// PCL转换头文件
#include <pcl_conversions/pcl_conversions.h>
// PCL点类型和点云头文件
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>         // PCD 格式读写
#include <cmath>  // 必须包含：fabs、isnan 依赖
#include "method_math.h"

namespace Ten
{

    
    namespace math
    {
        
        /**
            @brief 创建旋转矩阵 固定轴
            @param rx: roll (弧度)
            @param ry: pitch (弧度)
            @param rz: yaw (弧度)
            @return Eigen::Matrix3d: 3x3的旋转矩阵
        */
        Eigen::Matrix3d createRotationMatrix_fixed(double rx, double ry, double rz);
        /**
            @brief 创建平移矩阵 固定轴
            @param tx: x (米)
            @param ty: y (米)
            @param tz: z (米)
            @return Eigen::Vector3d: 1x3平移矩阵
        */
        Eigen::Vector3d createTranslationVector_fixed(double tx, double ty, double tz);
        /**
            @brief 创建旋转矩阵 固定轴
            @param rotation: 旋转矩阵
            @param translation: 平移矩阵
            @return Eigen::Matrix4d: 4x4的RT矩阵
        */
        //组合现有旋转矩阵与平移向量
        Eigen::Matrix4d combineRotationAndTranslation_fixed(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);

        /**
         * @brief 将变化矩阵转为XYZRPY 固定轴
         * @param transform_matrix： 变换矩阵
         */
        XYZRPY transform_matrixtoXYZRPY_fixed(Eigen::Matrix4d transform_matrix);

        /**
         * @brief 将XYZRPY转为变化矩阵 固定轴
         * @param xyzrpy： 旋转和平移
         */
        Eigen::Matrix4d XYZRPYtotransform_matrix_fixed(XYZRPY xyzrpy);

        /**
         * @brief Eigen旋转矩阵转欧拉角（Z-Y-X 顺序：Yaw-Pitch-Roll） 固定轴
         * @param R 3x3 Eigen旋转矩阵
         * @return RPY: roll pitch yaw (弧度)
         */
        RPY rotationMatrixToEulerAngles_fixed(const Eigen::Matrix3d R);

    }

   

}

#endif

