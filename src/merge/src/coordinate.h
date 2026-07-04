#ifndef __COORDINATE_H_
#define __COORDINATE_H_

#include "method_math_new.h"
#include "./parameter/parameter.h"

// //雷达自己的初始误差
// extern double _lidar_xyzrpy_init_error_xyz_x_;
// extern double _lidar_xyzrpy_init_error_xyz_y_;
// extern double _lidar_xyzrpy_init_error_xyz_z_;
// extern double _lidar_xyzrpy_init_error_rpy_roll_;
// extern double _lidar_xyzrpy_init_error_rpy_pitch_;
// extern double _lidar_xyzrpy_init_error_rpy_yaw_;

namespace Ten
{

    class Ten_coordinate
    {
    public:
        Ten_coordinate()
        {

        }
        ~Ten_coordinate(){}

        void init()
        {
            lidar_state_error_._xyz._x = _lidar_xyzrpy_init_error_xyz_x_;
            lidar_state_error_._xyz._y = _lidar_xyzrpy_init_error_xyz_y_;
            lidar_state_error_._xyz._z = _lidar_xyzrpy_init_error_xyz_z_;
            lidar_state_error_._rpy._roll = _lidar_xyzrpy_init_error_rpy_roll_;
            lidar_state_error_._rpy._pitch = _lidar_xyzrpy_init_error_rpy_pitch_;
            lidar_state_error_._rpy._yaw = _lidar_xyzrpy_init_error_rpy_yaw_;
        }
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


        /**
         * @brief 得到雷达到车的变换
         * @param Eigen::Matrix4d: 雷达到车的变换
         */
        Eigen::Matrix4d get_lidartocar()
        {
            std::lock_guard<std::mutex> lock(mtx_);
            return XYZRPYtotransform_matrix(lidartocar_);
        }

        /**
         * @brief 得到雷达到车的变换
         * @param Eigen::Matrix4d: 雷达到车的变换
         */
        Eigen::Matrix4d get_lidar_state_error()
        {
            std::lock_guard<std::mutex> lock(mtx_);
            return XYZRPYtotransform_matrix(lidar_state_error_);
        }

        /**
         * @brief 得到世界到雷达变换
         * @param Eigen::Matrix4d: 世界到雷达变换
         */
        Eigen::Matrix4d get_world2tolidar()
        {
            std::lock_guard<std::mutex> lock(mtx_);
            Eigen::Matrix4d T1 = XYZRPYtotransform_matrix(lidartocar_);
            Eigen::Matrix4d T1_N = T1.inverse();
            Eigen::Matrix4d T2 = XYZRPYtotransform_matrix(worldtolidar_- stead_state_error_);
            Eigen::Matrix4d T3 = XYZRPYtotransform_matrix(world2toworld1_);
            Eigen::Matrix4d mix;
            if(world2toworld1_ == Ten::XYZRPY())
            {
                mix = T2;
            }
            else
            {
                mix = T2 * T3 * T1_N;
            }
            return mix;
        }

        /**
         * @brief 车相对于世界坐标的位姿 动轴
         * @param Ten::XYZRPY: 车相对于世界坐标的位姿
         */
        Ten::XYZRPY getXYZRPY()
        {
            std::lock_guard<std::mutex> lock(mtx_);
            Eigen::Matrix4d T1 = XYZRPYtotransform_matrix(lidartocar_);
            Eigen::Matrix4d T1_N = T1.inverse();
            Eigen::Matrix4d T2 = XYZRPYtotransform_matrix(worldtolidar_- stead_state_error_);
            Eigen::Matrix4d T3 = XYZRPYtotransform_matrix(world2toworld1_);
            Eigen::Matrix4d mix = T1  * T2 * T3 * T1_N;
            XYZRPY xyzrpy = transform_matrixtoXYZRPY(mix); 
            return xyzrpy;
        }

        /**
         * @brief 车相对于世界坐标的位姿 固定轴
         * @param Ten::XYZRPY: 车相对于世界坐标的位姿
         */
        Ten::XYZRPY getXYZRPY_incline()
        {
            std::lock_guard<std::mutex> lock(mtx_);
            Eigen::Matrix4d T1 = XYZRPYtotransform_matrix(lidartocar_);
            Eigen::Matrix4d T1_N = T1.inverse();
            Eigen::Matrix4d T4 = Ten::math::XYZRPYtotransform_matrix_fixed(stead_state_error_);
            Eigen::Matrix4d T2 = T4 * (Ten::math::XYZRPYtotransform_matrix_fixed(worldtolidar_).inverse());
            Eigen::Matrix4d T3 = XYZRPYtotransform_matrix(world2toworld1_);
            Eigen::Matrix4d mix = T1  * T2 * T3 * T1_N;
            XYZRPY xyzrpy = Ten::transform_matrixtoXYZRPY(mix); 
            //XYZRPY xyzrpy = Ten::math::transform_matrixtoXYZRPY_fixed(mix.inverse()); 
            return xyzrpy;
        }

        // /**
        //  * @brief 车相对于世界坐标的位姿 固定轴 包含雷达误差 
        //  * @param Ten::XYZRPY: 车相对于世界坐标的位姿
        //  */
        // Ten::XYZRPY getXYZRPY_incline_2()
        // {
        //     std::lock_guard<std::mutex> lock(mtx_);
        //     Eigen::Matrix4d lidar_error = XYZRPYtotransform_matrix(lidar_state_error_);
        //     Eigen::Matrix4d lidar_error_inverse = lidar_error.inverse();
        //     Eigen::Matrix4d T1 = XYZRPYtotransform_matrix(lidartocar_);
        //     Eigen::Matrix4d T1_N = T1.inverse();
        //     Eigen::Matrix4d T4 = Ten::math::XYZRPYtotransform_matrix_fixed(stead_state_error_);
        //     Eigen::Matrix4d T2 = T4 * (Ten::math::XYZRPYtotransform_matrix_fixed(worldtolidar_).inverse());
        //     Eigen::Matrix4d T3 = XYZRPYtotransform_matrix(world2toworld1_);
        //     Eigen::Matrix4d mix = T1 * lidar_error * T2 * T3 * lidar_error_inverse * T1_N;
        //     XYZRPY xyzrpy = Ten::transform_matrixtoXYZRPY(mix); 
        //     //XYZRPY xyzrpy = Ten::math::transform_matrixtoXYZRPY_fixed(mix.inverse()); 
        //     return xyzrpy;
        // }


    private:
    Ten::XYZRPY lidar_state_error_;
    Ten::XYZRPY lidartocar_;
    Ten::XYZRPY worldtolidar_;
    Ten::XYZRPY world2toworld1_;
    Ten::XYZRPY stead_state_error_;
    mutable std::mutex mtx_;
    };





    extern Ten_coordinate _COORDINATE_TRANSFORMATION_;


}







#endif

