#ifndef __COORDINATE_H_
#define __COORDINATE_H_

#include "method_math.h"


namespace Ten
{

    class Ten_coordinate
    {
    public:
        Ten_coordinate(){}
        ~Ten_coordinate(){}

        /**
         * @brief 设置lidar坐标系到车坐标系的坐标变换
         * @param lidartocar: lidar坐标系到车坐标系的旋转平移
         */
        void set_lidartocar(Ten::XYZRPY lidartocar)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            lidartocar_ = lidartocar;
        }

        /**
         * @brief 设置当前世界坐标系到雷达坐标系的坐标变换
         * @param worldtolidar: 当前世界坐标系到雷达坐标系的旋转平移
         */
        void set_worldtolidar(Ten::XYZRPY worldtolidar)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            worldtolidar_ = worldtolidar;
        }

        /**
         * @brief 设置上一个世界坐标系到当前世界坐标系的坐标变换
         * @param world2toworld1: 上一个世界坐标系到当前世界坐标系的旋转平移
         */
        void set_world2toworld1(Ten::XYZRPY world2toworld1)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            world2toworld1_ = world2toworld1;
        }        

        /**
         * @brief 设置稳态误差
         * @param stead_state_error: 设置稳态误差
         */
        void set_stead_state_error(Ten::XYZRPY stead_state_error)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            stead_state_error_ = stead_state_error;
        }  

        // Ten::XYZRPY getXYZRPY()
        // {
        //     std::lock_guard<std::mutex> lock(mtx_);
        //     Eigen::Matrix4d T1 = XYZRPYtotransform_matrix(lidartocar_);
        //     Eigen::Matrix4d T2 = XYZRPYtotransform_matrix(worldtolidar_);
        //     Eigen::Matrix4d T3 = XYZRPYtotransform_matrix(world2toworld1_);
        //     Eigen::Matrix4d mix = T1 * T2 * T3;
        //     XYZRPY xyzrpy = transform_matrixtoXYZRPY(mix) - stead_state_error_; 
        //     return xyzrpy;
        // }

        Ten::XYZRPY getXYZRPY()
        {
            std::lock_guard<std::mutex> lock(mtx_);
            Eigen::Matrix4d T1 = XYZRPYtotransform_matrix(lidartocar_);
            Eigen::Matrix4d T1_N = T1.inverse();
            Eigen::Matrix4d T2 = XYZRPYtotransform_matrix(worldtolidar_);
            Eigen::Matrix4d T3 = XYZRPYtotransform_matrix(world2toworld1_);
            Eigen::Matrix4d mix = T1 * T2 * T3 * T1_N;
            XYZRPY xyzrpy = transform_matrixtoXYZRPY(mix) - stead_state_error_; 
            return xyzrpy;
        }



    private:

    Ten::XYZRPY lidartocar_;
    Ten::XYZRPY worldtolidar_;
    Ten::XYZRPY world2toworld1_;
    Ten::XYZRPY stead_state_error_;
    mutable std::mutex mtx_;
    };





    extern Ten_coordinate _COORDINATE_TRANSFORMATION_;


}







#endif

