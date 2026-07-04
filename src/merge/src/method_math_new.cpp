#ifndef __METHOD_MATH_NEW_CPP_
#define __METHOD_MATH_NEW_CPP_
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
#include "method_math_new.h"

namespace Ten
{

    namespace math
    {

        /**
            @brief 创建旋转矩阵
            @param rx: roll (弧度)
            @param ry: pitch (弧度)
            @param rz: yaw (弧度)
            @return Eigen::Matrix3d: 3x3的旋转矩阵
        */
        Eigen::Matrix3d createRotationMatrix_fixed(double rx, double ry, double rz) {
            // 弧度
            // 创建绕各轴的旋转矩阵
            Eigen::Matrix3d R_x;
            R_x << 1, 0, 0,
                0, cos(rx), -sin(rx),
                0, sin(rx), cos(rx);
            Eigen::Matrix3d R_y;
            R_y << cos(ry), 0, sin(ry),
                0, 1, 0,
                -sin(ry), 0, cos(ry);
            Eigen::Matrix3d R_z;
            R_z << cos(rz), -sin(rz), 0,
                sin(rz), cos(rz), 0,
                0, 0, 1;
            // 组合旋转矩阵 (Z-Y-X顺序: R = R_z * R_y * R_x)
            return R_z * R_y * R_x;
        }
        /**
            @brief 创建平移矩阵
            @param tx: x (米)
            @param ty: y (米)
            @param tz: z (米)
            @return Eigen::Vector3d: 1x3平移矩阵
        */
        Eigen::Vector3d createTranslationVector_fixed(double tx, double ty, double tz) {
            Eigen::Vector3d translation(tx, ty, tz);
            return translation;
        }
        /**
            @brief 创建旋转矩阵
            @param rotation: 旋转矩阵
            @param translation: 平移矩阵
            @return Eigen::Matrix4d: 4x4的RT矩阵
        */
        //组合现有旋转矩阵与平移向量
        Eigen::Matrix4d combineRotationAndTranslation_fixed(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
            Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
            transform.block<3, 3>(0, 0) = rotation;
            transform.block<3, 1>(0, 3) = translation;
            return transform;
        }


        //#define EPS 1e-6
        /**
         * @brief Eigen旋转矩阵转欧拉角（Z-Y-X 顺序：Yaw-Pitch-Roll）
         * @param R 3x3 Eigen旋转矩阵
         * @return RPY: roll pitch yaw (弧度)
         */
        RPY rotationMatrixToEulerAngles_fixed(const Eigen::Matrix3d R)
        {
            const double EPS = 1e-6;
            double roll = 0.0;
            double pitch  = 0.0;
            double yaw = 0.0;

            double sin_pitch = -R(2, 0);
            sin_pitch = std::max(std::min(sin_pitch, 1.0), -1.0);
            pitch = std::asin(sin_pitch);

            double cos_pitch = std::cos(pitch);

            if (std::abs(cos_pitch) > EPS) {
                roll = std::atan2(R(2, 1), R(2, 2));
                yaw = std::atan2(R(1, 0), R(0, 0));
            } else {
                yaw = 0.0;
                if (pitch > 0) {
                    roll = std::atan2(-R(0, 1), R(0, 2));
                } else {
                    roll = std::atan2(R(0, 1), -R(0, 2));
                }
            }
            RPY rpy;
            rpy._roll = roll;
            rpy._pitch = pitch;
            rpy._yaw = yaw;
            return rpy;
        }


        /**
         * @brief 将变化矩阵转为XYZRPY
         * @param transform_matrix： 变换矩阵
         */
        XYZRPY transform_matrixtoXYZRPY_fixed(Eigen::Matrix4d transform_matrix)
        {
            Eigen::Matrix3d R = transform_matrix.block<3, 3>(0, 0).cast<double>();
            Eigen::Vector3d t = transform_matrix.block<3, 1>(0, 3).cast<double>();
            RPY rpy = rotationMatrixToEulerAngles_fixed(R);
            XYZRPY xyzrpy;
            xyzrpy._xyz._x = t[0];
            xyzrpy._xyz._y = t[1];
            xyzrpy._xyz._z = t[2];

            xyzrpy._rpy = rpy;
            return xyzrpy;
        }

        /**
         * @brief 将XYZRPY转为变化矩阵
         * @param xyzrpy： 旋转和平移
         */
        Eigen::Matrix4d XYZRPYtotransform_matrix_fixed(XYZRPY xyzrpy)
        {
            Eigen::Matrix3d rot = createRotationMatrix_fixed(xyzrpy._rpy._roll, xyzrpy._rpy._pitch, xyzrpy._rpy._yaw);
            Eigen::Vector3d tra = createTranslationVector_fixed(xyzrpy._xyz._x, xyzrpy._xyz._y, xyzrpy._xyz._z);
            Eigen::Matrix4d T = combineRotationAndTranslation_fixed(rot, tra);
            return T;
        }

    }

}




#endif

