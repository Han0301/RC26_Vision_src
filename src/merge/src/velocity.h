#ifndef __VELOCITY_H_
#define __VELOCITY_H_

#include "method_math.h"


namespace Ten
{

    class Ten_velocity
    {
    public:
        Ten_velocity(){}
        ~Ten_velocity(){}

        /**
         * @brief 设置lidar坐标系速度和角速度
         * @param lidar: lidar坐标系速度和角速度
         */
        void set_lidar(Ten::XYZRPY lidar)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            lidar_ = lidar;
        }

        /**
         * @brief 设置lidar坐标系到车坐标系的坐标变换
         * @param RT: lidar坐标系到车坐标系的旋转平移
         */
        void set_RT(Ten::XYZRPY RT)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            RT_ = RT;
        }

        /**
         * @brief 获取车坐标系下的速度角速度
         * @return Ten::XYZRPY
         */
        Ten::XYZRPY getvelocity()
        {
            std::lock_guard<std::mutex> lock(mtx_);
            Eigen::Matrix3d R = createRotationMatrix(-RT_._rpy._roll, -RT_._rpy._pitch, -RT_._rpy._yaw);
            Eigen::Vector3d V_L_lidar = R * createTranslationVector(lidar_._xyz._x, lidar_._xyz._y, lidar_._xyz._z);
            Eigen::Vector3d V_A_lidar = R * createTranslationVector(lidar_._rpy._roll, lidar_._rpy._pitch, lidar_._rpy._yaw);
            Eigen::Vector3d R_car_at_lidar = R * createTranslationVector(RT_._xyz._x, RT_._xyz._y, RT_._xyz._z);
            Eigen::Vector3d V_L_car = V_L_lidar + V_A_lidar.cross(R_car_at_lidar);
            car_._xyz._x = V_L_car[0];
            car_._xyz._y = V_L_car[1];
            car_._xyz._z = V_L_car[2];
            car_._rpy._roll = V_A_lidar[0];
            car_._rpy._pitch = V_A_lidar[1];
            car_._rpy._yaw = V_A_lidar[2];
            return car_;
        }

    private:
    Ten::XYZRPY lidar_;
    Ten::XYZRPY car_;
    Ten::XYZRPY RT_;
    std::mutex mtx_;

    };


    extern Ten::Ten_velocity _VELOCITY_TRANSFORMATION_;
}







#endif


