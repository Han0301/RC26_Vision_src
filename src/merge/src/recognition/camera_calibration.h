#ifndef __CAMERA_CALIBRATION_H_
#define __CAMERA_CALIBRATION_H_
#include <ros/ros.h>
#include <iostream>
#include <cmath>
#include <eigen3/Eigen/Core>       // 核心矩阵/向量
#include <eigen3/Eigen/Geometry>   // 几何变换（旋转、平移）
#include <opencv2/opencv.hpp>
#include <fstream>
#include <sstream>
#include <dirent.h>
#include <algorithm>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <mutex>
#include "../method_math.h"


namespace Ten
{

// typedef const double (* const D435_ARR_PTR)[4];


class Ten_camerainfo
{
public:
    //Ten_camerainfo& operator=(const Ten_camerainfo& camerainfo) = delete;
    //Ten_camerainfo(const Ten_camerainfo& camerainfo) = delete;
    
    Ten_camerainfo()
    {
        K_ = (cv::Mat_<double>(3,3) <<1380.4350, 0, 974.0183,0,  1385.0788, 541.4301, 0, 0, 1);
        distCoeffs_ = cv::Mat::zeros(5, 1, CV_64F);
        // 修复：初始化rvec_/tvec_为3x1 CV_64F，避免空矩阵
        rvec_ = cv::Mat::zeros(3, 1, CV_64F);
        tvec_ = cv::Mat::zeros(3, 1, CV_64F);
        // 初始化外参矩阵
        extrinsic_.setIdentity();
        R_.setIdentity();
        T_.setZero();
    }
    ~Ten_camerainfo(){}

    /**
     * @brief 设置畸变矩阵
     * @param distCoeffs:5x1畸变矩阵
     */
    void set_distCoeffs(cv::Mat distCoeffs)
    {
        //std::mutex mtx_;
        if(distCoeffs.rows != 5 || distCoeffs.cols != 1)
        {
            std::cout<< "distCoeffs.rows != 5 || distCoeffs.cols != 1" << std::endl;
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        //distCoeffs_ = distCoeffs;
        distCoeffs.copyTo(distCoeffs_);
    }

    /**
     * @brief 设置内参矩阵
     * @param K:3x3内参矩阵
     */
    void set_K(cv::Mat K)
    {
        //std::mutex mtx_;
        if(K.rows != 3 || K.cols != 3)
        {
            std::cout<< "K.rows != 3 || K.cols != 3" << std::endl;
            return;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        //K_ = K;
        K.copyTo(K_);
    }

    /**
     * @brief 设置外参RT
     * @param rvec:旋转向量
     * @param tvec:平移向量
     */
    void set_RT(cv::Mat rvec, cv::Mat tvec)
    {
        //std::mutex mtx_;
        if(rvec.rows != 3 && rvec.cols != 1)
        {
            std::cout<< "error! Eigen::Matrix3d RotationMatrixtorvec(cv::Mat rvec)" << std::endl;
            return ;
        }
        if(tvec.rows != 3 && tvec.cols != 1)
        {
            std::cout<< "error! Eigen::Matrix3d RotationMatrixtorvec(cv::Mat tvec)" << std::endl;
            return ;
        }
        std::lock_guard<std::mutex> lock(mtx_);
        // rvec_ = rvec;
        // tvec_ = tvec;
        rvec.copyTo(rvec_);
        tvec.copyTo(tvec_);
        resetEXrt();
    }

    /**
     * @brief 设置图片大小
     * @param image_H: 图片高
     * @param image_W: 图片宽
     */
    void set_HW(int image_H, int image_W)
    {
        //std::mutex mtx_;
        std::lock_guard<std::mutex> lock(mtx_);
        image_H_ = image_H;
        image_W_ = image_W;
    }


    /**
     * @brief 设置外参
     * @param extrinsic:4x4外参矩阵
     */
    void set_Extrinsic_Matrix(Eigen::Matrix4d extrinsic)
    {
        //std::mutex mtx_;
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
        //std::mutex mtx_;
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
        //std::mutex mtx_;
        std::lock_guard<std::mutex> lock(mtx_);
        return R_;
    }

    /**
     * @brief 获取平移向量
     * @return Eigen::Vector3d
     */
    Eigen::Vector3d T() const
    {
        //std::mutex mtx_;
        std::lock_guard<std::mutex> lock(mtx_);
        return T_;
    }

    /**
     * @brief 获取旋转向量
     * @return cv::Mat
     */
    cv::Mat rvec() const
    {
        //std::mutex mtx_;
        std::lock_guard<std::mutex> lock(mtx_);
        return rvec_;
    }

    /**
     * @brief 获取平移向量
     * @return cv::Mat
     */    
    cv::Mat tvec() const
    {
        //std::mutex mtx_;
        std::lock_guard<std::mutex> lock(mtx_);
        return tvec_;
    }

    /**
     * @brief 获取相机内参
     * @return cv::Mat
     */    
    cv::Mat K() const 
    {
        //std::mutex mtx_;
        std::lock_guard<std::mutex> lock(mtx_);
        return K_;
    }

    /**
     * @brief 获取相机畸变矩阵
     * @return cv::Mat
     */    
    cv::Mat distCoeffs() const 
    {
        //std::mutex mtx_;
        std::lock_guard<std::mutex> lock(mtx_);
        return distCoeffs_;
    }

    /**
     * @brief 获取变换矩阵
     * @return Eigen::Matrix4d
     */    
    Eigen::Matrix4d extrinsic() const
    {
        //std::mutex mtx_;
        std::lock_guard<std::mutex> lock(mtx_);
        return extrinsic_;
    }

    /**
     * @brief 获取图片高
     * @return int
     */    
    int H() const
    {
        return image_H_;
    }

    /**
     * @brief 获取图片宽
     * @return int
     */    
    int W() const
    {
        return image_W_;
    }


private:
    void resetRTrt() 
    {
        R_ = extrinsic_.block<3, 3>(0, 0);
        T_ = extrinsic_.block<3, 1>(0, 3);
        rvec_ = RotationMatrixtorvec(R_);
        tvec_ = vector3dtotvec(T_);
    }
    void resetEXrt() 
    {
        R_ = rvectoRotationMatrix(rvec_);
        T_ = tvectovector3d(tvec_);
        extrinsic_.block<3, 3>(0, 0) = R_;
        extrinsic_.block<3, 1>(0, 3) = T_;
    }

cv::Mat K_;
cv::Mat distCoeffs_;
Eigen::Matrix4d extrinsic_;
cv::Mat rvec_;
cv::Mat tvec_;
Eigen::Matrix3d R_;
Eigen::Vector3d T_;
int image_H_ = 1080;
int image_W_ = 1920;
mutable std::mutex mtx_;
};







}



#endif


