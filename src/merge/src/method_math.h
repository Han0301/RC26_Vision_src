#ifndef __METHOD_MATH_H_
#define __METHOD_MATH_H_
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

namespace Ten
{

    struct XYZ
    {
        XYZ operator-(const XYZ& other) const
        {
            XYZ result;
            result._x = this->_x - other._x;
            result._y = this->_y - other._y;
            result._z = this->_z - other._z;
            return result;
        }

        XYZ operator-() const
        {
            XYZ result;
            result._x = -this->_x;
            result._y = -this->_y;
            result._z = -this->_z;
            return result;
        }

        double _x = 0.0;
        double _y = 0.0;
        double _z = 0.0;
    };

    struct RPY
    {
        RPY operator-(const RPY& other) const
        {
            RPY result;
            result._roll = this->_roll - other._roll;
            result._pitch = this->_pitch - other._pitch;
            result._yaw = this->_yaw - other._yaw;
            return result;
        }

        RPY operator-() const
        {
            RPY result;
            result._roll = -this->_roll;
            result._pitch = -this->_pitch;
            result._yaw = -this->_yaw;
            return result;
        }

        double _roll = 0.0;
        double _pitch = 0.0;
        double _yaw = 0.0;
    };

    struct XYZRPY
    {
        XYZRPY operator-(const XYZRPY& other) const
        {
            XYZRPY result;
            result._xyz = this->_xyz - other._xyz;
            result._rpy = this->_rpy - other._rpy;
            return result;
        }

        XYZRPY operator-() const
        {
            XYZRPY result;
            result._xyz = -this->_xyz; 
            result._rpy = -this->_rpy;   
            return result;
        }

        XYZ _xyz;
        RPY _rpy;
    };

    /**
        @brief 里程计消息转xyzrpy
        @param msg: nav_msgs::Odometry
        @return xyzrpy
    */
    XYZRPY Nav_Odometrytoxyzrpy(const nav_msgs::Odometry msg);

    /**
        @brief 创建旋转矩阵
        @param rx: roll (弧度)
        @param ry: pitch (弧度)
        @param rz: yaw (弧度)
        @return Eigen::Matrix3d: 3x3的旋转矩阵
    */
    Eigen::Matrix3d createRotationMatrix(double rx, double ry, double rz);
    /**
        @brief 创建平移矩阵
        @param tx: x (米)
        @param ty: y (米)
        @param tz: z (米)
        @return Eigen::Vector3d: 1x3平移矩阵
    */
    Eigen::Vector3d createTranslationVector(double tx, double ty, double tz);
    /**
        @brief 创建旋转矩阵
        @param rotation: 旋转矩阵
        @param translation: 平移矩阵
        @return Eigen::Matrix4d: 4x4的RT矩阵
    */
    //组合现有旋转矩阵与平移向量
    Eigen::Matrix4d combineRotationAndTranslation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation);
    /**
        @brief 世界到当前
        @param xyz：当前点在世界坐标系的xyz坐标
        @param rpy: 当前点相对于世界坐标系的旋转和平移
    */
    Eigen::Matrix4d worldtocurrent(XYZ xyz, RPY rpy);

    /**
        @brief 当前到世界
        @param xyz：当前点在世界坐标系的xyz坐标
        @param rpy: 当前点相对于世界坐标系的旋转和平移
    */
    Eigen::Matrix4d currenttoworld(XYZ xyz, RPY rpy);
    /**
        @brief 把Eigen::Matrix3d转cv::Mat rvec
        @param R: 旋转矩阵
        @return cv::Mat：rvec
    */
    cv::Mat RotationMatrixtorvec(Eigen::Matrix3d R);

    /**
        @brief 把cv::Mat rvec转 Eigen::Matrix3d
        @param rvec: 旋转向量 cv::Mat
        @return Eigen::Matrix3d：R
    */
    Eigen::Matrix3d rvectoRotationMatrix(cv::Mat rvec);



    /**
     * @brief 点云回调函数：将sensor_msgs::PointCloud2转为PointT
     * @param msg 订阅到的sensor_msgs/PointCloud2消息
     * @return pcl::PointCloud<PointT>::Ptr 对象的智能指针
     */
    template<typename PointCloudT>
    typename PointCloudT::Ptr sensor_msgs_PointCloud2topcltype(const sensor_msgs::PointCloud2& msg)
    {
        // 声明PCL点云对象（智能指针避免内存泄漏）  修复：添加 typename 声明嵌套类型
        typename PointCloudT::Ptr cloud_xyz_inormal(new PointCloudT);
        try
        {
            // 核心转换函数：ROS PointCloud2 -> PCL PointXYZINormal
            //std::cout<< "pcl::fromROSMsg(msg, *cloud_xyz_inormal);"<< std::endl;
            pcl::fromROSMsg(msg, *cloud_xyz_inormal);
            //std::cout<< "pcl::fromROSMsg(msg, *cloud_xyz_inormal) finish"<< std::endl;
            return cloud_xyz_inormal;
        }
        catch (const std::exception& e)
        {
            // 捕获转换异常（如字段缺失、类型不匹配）
            std::cout << e.what() << std::endl;
            return nullptr;
        } 
    }


    //#define EPS 1e-6
    /**
     * @brief Eigen旋转矩阵转欧拉角（Z-Y-X 顺序：Yaw-Pitch-Roll）
     * @param R 3x3 Eigen旋转矩阵
     * @return RPY: roll pitch yaw (弧度)
     */
    RPY rotationMatrixToEulerAngles(const Eigen::Matrix3d R);


    /**
     * @brief 将变化矩阵转为XYZRPY
     * @param transform_matrix： 变换矩阵
     */
    XYZRPY transform_matrixtoXYZRPY(Eigen::Matrix4d transform_matrix);

    /**
     * @brief 将XYZRPY转为变化矩阵
     * @param xyzrpy： 旋转和平移
     */
    Eigen::Matrix4d XYZRPYtotransform_matrix(XYZRPY xyzrpy);

    /**
     * @brief 平移向量 vector3dtotevc
     * @param T: 平移向量
     * @return cv::Mat:平移向量
     */
    cv::Mat vector3dtotevc(Eigen::Vector3d T);

    /**
     * @brief 画调试图像
     * @param image: 图像
     * @param imagePoints：2d点对
     */
    void debug_draw_img(cv::Mat& image, std::vector<cv::Point2f>& imagePoints);

}

#endif

