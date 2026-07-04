#ifndef __METHOD_MATH_CPP_
#define __METHOD_MATH_CPP_
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
#include "method_math.h"

namespace Ten
{

    // struct XYZ
    // {
    //     double _x = 0.0;
    //     double _y = 0.0;
    //     double _z = 0.0;
    // };

    // struct RPY
    // {
    //     double _roll = 0.0;
    //     double _pitch = 0.0;
    //     double _yaw = 0.0;
    // };

    // struct XYZRPY
    // {
    //     XYZ _xyz;
    //     RPY _rpy;
    // };

    /**
        @brief 里程计消息转xyzrpy
        @param msg: nav_msgs::Odometry
        @return xyzrpy
    */
    XYZRPY Nav_Odometrytoxyzrpy(const nav_msgs::Odometry msg)
    {
        XYZRPY change;

        // 1. 获取位姿基础数据（位置 + 四元数）
        change._xyz._x = msg.pose.pose.position.x;
        change._xyz._y = msg.pose.pose.position.y;
        change._xyz._z = msg.pose.pose.position.z;

        double ori_x = msg.pose.pose.orientation.x;
        double ori_y = msg.pose.pose.orientation.y;
        double ori_z = msg.pose.pose.orientation.z;
        double ori_w = msg.pose.pose.orientation.w;

        // 2. 四元数转欧拉角（Roll/Pitch/Yaw，单位：弧度）
        // 步骤1：构造tf四元数对象
        tf::Quaternion quat(ori_x, ori_y, ori_z, ori_w);
        // 步骤2：定义存储欧拉角的变量（roll:绕X轴, pitch:绕Y轴, yaw:绕Z轴）
        double roll, pitch, yaw;
        // 步骤3：转换为欧拉角 弧度
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        change._rpy._roll = roll;
        change._rpy._pitch = pitch;
        change._rpy._yaw = yaw;
        return change;
    }

    /**
        @brief 创建旋转矩阵
        @param rx: roll (弧度)
        @param ry: pitch (弧度)
        @param rz: yaw (弧度)
        @return Eigen::Matrix3d: 3x3的旋转矩阵
    */
    Eigen::Matrix3d createRotationMatrix(double rx, double ry, double rz) {
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
    Eigen::Vector3d createTranslationVector(double tx, double ty, double tz) {
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
    Eigen::Matrix4d combineRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation) {
        Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
        transform.block<3, 3>(0, 0) = rotation;
        transform.block<3, 1>(0, 3) = translation;
        return transform;
    }

    /**
        @brief 世界到当前 点变化
        @param xyz：当前点在世界坐标系的xyz坐标
        @param rpy: 当前点相对于世界坐标系的旋转和平移
    */
    Eigen::Matrix4d worldtocurrent(XYZ xyz, RPY rpy)
    {
        Eigen::Matrix3d rot = createRotationMatrix(-rpy._roll, -rpy._pitch, -rpy._yaw);
        Eigen::Vector3d tra = -(rot*createTranslationVector(xyz._x, xyz._y, xyz._z));
        Eigen::Matrix4d T = combineRotationAndTranslation(rot, tra);
        return T;
    }

    /**
        @brief 当前到世界 点变化
        @param xyz：当前点在世界坐标系的xyz坐标
        @param rpy: 当前点相对于世界坐标系的旋转和平移
    */
    Eigen::Matrix4d currenttoworld(XYZ xyz, RPY rpy)
    {
        Eigen::Matrix4d T = worldtocurrent(xyz, rpy);
        return T.inverse();
    }




    /**
        @brief 把Eigen::Matrix3d转cv::Mat rvec
        @param R: 旋转矩阵
        @return cv::Mat：rvec
    */
    cv::Mat RotationMatrixtorvec(Eigen::Matrix3d R)
    {
        cv::Mat rot_matrix = (cv::Mat_<double>(3, 3) <<
        R(0,0), R(0,1), R(0,2),
        R(1,0), R(1,1), R(1,2),
        R(2,0), R(2,1), R(2,2));
        cv::Mat rvec;
        cv::Rodrigues(rot_matrix, rvec);
        return rvec;
    }

    /**
        @brief 把cv::Mat rvec转 Eigen::Matrix3d
        @param rvec: 旋转向量 cv::Mat
        @return Eigen::Matrix3d：R
    */
    Eigen::Matrix3d rvectoRotationMatrix(cv::Mat rvec)
    {
        if(rvec.rows != 3 && rvec.cols != 1)
        {
            std::cout<< "error! Eigen::Matrix3d RotationMatrixtorvec(cv::Mat rvec)" << std::endl;
            return Eigen::Matrix3d();
        }
        cv::Mat rot_matrix;
        cv::Rodrigues(rvec, rot_matrix);
        Eigen::Matrix3d R;
        for(int i = 0; i < 3; i ++)
        {
            for(int j = 0; j < 3; j++)
            {
                R(i,j) = rot_matrix.at<double>(i, j);
            }
        }
        return R;
    }



    // /**
    //  * @brief 点云回调函数：将sensor_msgs::PointCloud2转为PointT
    //  * @param msg 订阅到的sensor_msgs/PointCloud2消息
    //  * @return pcl::PointCloud<PointT>::Ptr 对象的智能指针
    //  */
    // template<typename PointT>
    // typename pcl::PointCloud<PointT>::Ptr sensor_msgs_PointCloud2topcltpye(const sensor_msgs::PointCloud2ConstPtr& msg)
    // {
    //     // 声明PCL点云对象（智能指针避免内存泄漏）  修复：添加 typename 声明嵌套类型
    //     typename pcl::PointCloud<PointT>::Ptr cloud_xyz_inormal(new pcl::PointCloud<PointT>);
    //     try
    //     {
    //         // 核心转换函数：ROS PointCloud2 -> PCL PointXYZINormal
    //         pcl::fromROSMsg(*msg, *cloud_xyz_inormal);
    //         return cloud_xyz_inormal;
    //     }
    //     catch (const std::exception& e)
    //     {
    //         // 捕获转换异常（如字段缺失、类型不匹配）
    //         std::cout << e.what() << std::endl;
    //         return nullptr;
    //     } 
    // }


    #define EPS 1e-6
    /**
     * @brief Eigen旋转矩阵转欧拉角（Z-Y-X 顺序：Yaw-Pitch-Roll）
     * @param R 3x3 Eigen旋转矩阵
     * @return RPY: roll pitch yaw (弧度)
     */
    RPY rotationMatrixToEulerAngles(const Eigen::Matrix3d R)
    {
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
        rpy._roll = -roll;
        rpy._pitch = -pitch;
        rpy._yaw = -yaw;
        return rpy;
    }


    /**
     * @brief 将变化矩阵转为XYZRPY
     * @param transform_matrix： 变换矩阵
     */
    XYZRPY transform_matrixtoXYZRPY(Eigen::Matrix4d transform_matrix)
    {
        Eigen::Matrix3d R = transform_matrix.block<3, 3>(0, 0).cast<double>();
        Eigen::Vector3d t = transform_matrix.block<3, 1>(0, 3).cast<double>();
        RPY rpy = rotationMatrixToEulerAngles(R);
        Eigen::Vector3d ttt = - R.inverse() * t;
        XYZRPY xyzrpy;
        xyzrpy._xyz._x = ttt[0];
        xyzrpy._xyz._y = ttt[1];
        xyzrpy._xyz._z = ttt[2];

        xyzrpy._rpy = rpy;
        return xyzrpy;
    }

    /**
     * @brief 将XYZRPY转为变化矩阵
     * @param xyzrpy： 旋转和平移
     */
    Eigen::Matrix4d XYZRPYtotransform_matrix(XYZRPY xyzrpy)
    {
        Eigen::Matrix3d rot = createRotationMatrix(-xyzrpy._rpy._roll, -xyzrpy._rpy._pitch, -xyzrpy._rpy._yaw);
        Eigen::Vector3d tra = -(rot*createTranslationVector(xyzrpy._xyz._x, xyzrpy._xyz._y, xyzrpy._xyz._z));
        Eigen::Matrix4d T = combineRotationAndTranslation(rot, tra);
        return T;
    }

    /**
     * @brief 平移向量 vector3dtotevc
     * @param T: 平移向量
     * @return cv::Mat:平移向量
     */
    cv::Mat vector3dtotevc(Eigen::Vector3d T)
    {
        cv::Mat tevc;
        //tevc = cv::Mat(3, 1, CV_32FC1, T.data()).clone();
        tevc = (cv::Mat_<float>(3, 1) << T(0), T(1), T(2));
        return tevc;
    }

    /**
     * @brief 画调试图像
     * @param image: 图像
     * @param imagePoints：2d点对
     */
    void debug_draw_img(cv::Mat& image, std::vector<cv::Point2f>& imagePoints)
    {
        for (int i = 0; i < imagePoints.size(); i += 4) 
        {
            if (i < 96){
            cv::line(image, cv::Point(cvRound(imagePoints[i].x), cvRound(imagePoints[i].y)),cv::Point(cvRound(imagePoints[i+1].x), cvRound(imagePoints[i+1].y)),cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(cvRound(imagePoints[i+1].x), cvRound(imagePoints[i+1].y)),cv::Point(cvRound(imagePoints[i+2].x), cvRound(imagePoints[i+2].y)),cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(cvRound(imagePoints[i+2].x), cvRound(imagePoints[i+2].y)),cv::Point(cvRound(imagePoints[i+3].x), cvRound(imagePoints[i+3].y)),cv::Scalar(0, 0, 255), 2);
            cv::line(image, cv::Point(cvRound(imagePoints[i+3].x), cvRound(imagePoints[i+3].y)),cv::Point(cvRound(imagePoints[i].x), cvRound(imagePoints[i].y)),cv::Scalar(0, 0, 255), 2);
        } 
        }
    }

}

#endif

