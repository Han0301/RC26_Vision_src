#ifndef __CAMERA_CALIBRATION_H_
#define __CAMERA_CALIBRATION_H_
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>       // 核心矩阵/向量
#include <eigen3/Eigen/Geometry>   // 几何变换（旋转、平移）
#include <opencv2/opencv.hpp>
#include "../method_math.h"


namespace Ten
{

// typedef const double (* const D435_ARR_PTR)[4];


class Ten_camerainfo
{
public:
    Ten_camerainfo& operator=(const Ten_camerainfo& camerainfo) = delete;
    Ten_camerainfo(const Ten_camerainfo& camerainfo) = delete;
    Ten_camerainfo()
    {
        K_ = (cv::Mat_<double>(3,3) <<1380.4350, 0, 974.0183,0,  1385.0788, 541.4301, 0, 0, 1);
        distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);
    }
    ~Ten_camerainfo(){}

    /**
     * @brief 设置畸变矩阵
     * @param distCoeffs:5x1畸变矩阵
     */
    void set_distCoeffs(cv::Mat distCoeffs)
    {
        if(distCoeffs.rows != 5 || distCoeffs.cols != 1)
        {
            std::cout<< "distCoeffs.rows != 5 || distCoeffs.cols != 1" << std::endl;
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        distCoeffs_ = distCoeffs;
    }

    /**
     * @brief 设置内参矩阵
     * @param K:3x3内参矩阵
     */
    void set_K(cv::Mat K)
    {
        if(K.rows != 3 || K.cols != 3)
        {
            std::cout<< "K.rows != 3 || K.cols != 3" << std::endl;
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        K_ = K;
    }


    /**
     * @brief 设置外参
     * @param extrinsic:4x4外参矩阵
     */
    void set_Extrinsic_Matrix(Eigen::Matrix4d extrinsic)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        extrinsic_ = extrinsic;
        resetRTrt();
    }

    /**
     * @brief 设置外参
     * @param extrinsic:4x4外参矩阵
     */
    void set_Extrinsic_Matrix(const double arr[4][4])
    {
        std::lock_guard<std::mutex> lock(mtx_);
        //extrinsic_ = extrinsic;
        for(size_t i = 0; i < 4; i++)
        {
            for(size_t j = 0; j < 4; j++)
            {
                extrinsic_(i,j) = arr[i][j];
            }
        }
        resetRTrt();
    }

    /**
     * @brief 获取旋转矩阵
     * @return Eigen::Matrix3d
     */
    Eigen::Matrix3d R() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return R_;
    }

    /**
     * @brief 获取平移向量
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d T() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return T_;
    }

    /**
     * @brief 获取旋转向量
     * @return cv::Mat
     */
    cv::Mat revc() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return revc_;
    }

    /**
     * @brief 获取平移向量
     * @return cv::Mat
     */    
    cv::Mat tevc() const
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return tevc_;
    }

    /**
     * @brief 获取相机内参
     * @return cv::Mat
     */    
    cv::Mat K() const 
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return K_;
    }

    /**
     * @brief 获取相机畸变矩阵
     * @return cv::Mat
     */    
    cv::Mat distCoeffs() const 
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return distCoeffs_;
    }

    /**
     * @brief 获取变换矩阵
     * @return Eigen::Matrix4d
     */    
    Eigen::Matrix4d extrinsic()
    {
        std::lock_guard<std::mutex> lock(mtx_);
        return extrinsic_;
    }

private:
    void resetRTrt() 
    {
        R_ = extrinsic_.block<3, 3>(0, 0);
        T_ = extrinsic_.block<3, 1>(0, 3);
        revc_ = RotationMatrixtorvec(R_);
        tevc_ = vector3dtotevc(T_);
    }


cv::Mat K_;
cv::Mat distCoeffs_;
Eigen::Matrix4d extrinsic_;
cv::Mat revc_;
cv::Mat tevc_;
Eigen::Matrix3d R_;
Eigen::Vector3d T_;
mutable std::mutex mtx_;
};



// const cv::Mat& get_D435_K() {
//     static const cv::Mat K = (cv::Mat_<double>(3,3) <<
//         1380.4350, 0,        974.0183,
//         0,        1385.0788, 541.4301,
//         0,        0,        1);
//     return K;
// }


// D435_ARR_PTR get_D435_E() {
//     static const double arr[4][4] = {-0.0530959,  -0.998589,  -1.9891e-05,  0.0214,
//                                     -0.0403182,  0.00216366,  -0.999185,  0.3877,
//                                     0.997775,  -0.0530518,  -0.0403762,  0.4997,
//                                     0,         0,           0,           1};
//     return arr;
// }

// inline Ten_camerainfo& get_D435_CAMERAINFO() {
//     static Ten_camerainfo cameraInfo(get_D435_K(), get_D435_E()); 
//     return cameraInfo;
// }


// #define _D435_CAMERAINFO_ get_D435_CAMERAINFO()




}



#endif



