#ifndef __CONTROLR1_H_
#define __CONTROLR1_H_
#include "./../serial.h"
#include "./../openvino.h"
#include "./../threadpool.h"
#include "./../livox_ros_driver2/src/livox_ros_driver.h"
#include "./../point_lio/src/laserMapping2.h"
#include "./../lidar.h"
#include "./../camera.h"
#include "./../relocation.h"
#include "./../coordinate.h"
#include "./../recognition/camera_calibration.h"
#include "./../recognition/world_to_camera.h"
#include "./../velocity.h"
#include "./../calibration.h"
#include "./../log/logger.h"
#include <cctype>     // isdigit() 字符判断（必须包含，否则部分编译器报错）
#include <fstream>    // 文件读写
#include "./../filter.h"
#include "../parameter/parameter.h"


namespace Ten
{
    namespace superstratum
    {

        // #define _r1_xyzrpy_car_xyz_x_ -0.40944
        // #define _r1_xyzrpy_car_xyz_y_ 0.40944 + 0.088 /2
        // #define _r1_xyzrpy_car_xyz_z_ 0
        // #define _r1_xyzrpy_car_rpy_roll_ 0
        // #define _r1_xyzrpy_car_rpy_pitch_ 0
        // #define _r1_xyzrpy_car_rpy_yaw_ -M_PI / 2.0

        // #define _r1_xyzrpy_error_xyz_x_ 0.025
        // #define _r1_xyzrpy_error_xyz_y_ -0.045
        // #define _r1_xyzrpy_error_xyz_z_ 0.10
        // #define _r1_xyzrpy_error_rpy_roll_ 0
        // #define _r1_xyzrpy_error_rpy_pitch_ 0
        // #define _r1_xyzrpy_error_rpy_yaw_ 0

        class controlR1
        {
        public:
            static void serial_send_lidarR1()
            {
                urcu_memb_register_thread();
                Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
                float arr[9] = {0};
                Ten::XYZRPY xyzrpy_error;
                xyzrpy_error._xyz._x = _r1_xyzrpy_init_error_xyz_x_;
                xyzrpy_error._xyz._y = _r1_xyzrpy_init_error_xyz_y_;
                xyzrpy_error._xyz._z = _r1_xyzrpy_init_error_xyz_z_;
                xyzrpy_error._rpy._roll = _r1_xyzrpy_init_error_rpy_roll_;
                xyzrpy_error._rpy._pitch = _r1_xyzrpy_init_error_rpy_pitch_;
                xyzrpy_error._rpy._yaw = _r1_xyzrpy_init_error_rpy_yaw_;
                Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);
                Ten::XYZRPY xyzrpy_car;
                xyzrpy_car._xyz._x = _r1_xyzrpy_car_xyz_x_; //-0.40944
                xyzrpy_car._xyz._y = _r1_xyzrpy_car_xyz_y_;  //0.40944
                xyzrpy_car._xyz._z = _r1_xyzrpy_car_xyz_z_;
                xyzrpy_car._rpy._roll = _r1_xyzrpy_car_rpy_roll_;
                xyzrpy_car._rpy._pitch = _r1_xyzrpy_car_rpy_pitch_;
                xyzrpy_car._rpy._yaw = _r1_xyzrpy_car_rpy_yaw_;
                Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car);  
                Ten::_VELOCITY_TRANSFORMATION_.set_RT(xyzrpy_car);
                //nav_msgs::Odometry odo_n;
                //ros::Rate sl(100);
                while(Ten::_TREADPOOL_FLAG_.read_flag())
                {
                    //位置变化
                    nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
                    Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
                    Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
                    Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();

                    if(result.XYZRPYisnan())
                    {
                        usleep(10*1000);
                        continue;
                    }
                    //速度变化
                    Ten::XYZRPY lidar_LA;
                    lidar_LA._xyz._x = odo.twist.twist.linear.x;
                    lidar_LA._xyz._y = odo.twist.twist.linear.y;
                    lidar_LA._xyz._z = odo.twist.twist.linear.z;
                    lidar_LA._rpy._roll = odo.twist.twist.angular.x;
                    lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
                    lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
                    Ten::_VELOCITY_TRANSFORMATION_.set_lidar(lidar_LA);
                    Ten::XYZRPY car_LA = Ten::_VELOCITY_TRANSFORMATION_.getvelocity();
                    float roll = result._rpy._roll;
                    float pitch = result._rpy._pitch;
                    float yaw = result._rpy._yaw;
                    arr[0] = result._xyz._x;
                    arr[1] = result._xyz._y;
                    arr[2] = result._xyz._z;
                    arr[3] = roll * 180.0 / M_PI;
                    arr[4] = pitch * 180.0 / M_PI;
                    arr[5] = yaw * 180.0 / M_PI;
                    arr[6] = car_LA._xyz._x;
                    arr[7] = car_LA._xyz._y;
                    arr[8] = car_LA._xyz._z;
                    //id为1
                    serial.serial_send(arr, 1, sizeof(arr));       
                    //sl.sleep();
                    usleep(10*1000);
                }
                urcu_memb_unregister_thread();
            }

            static bool calibration2()
            {
                std::string log_path = std::string(ROOT_DIR) + std::string("map/map.pcd");
                //std::string log_path = std::string("/home/maple/study2/maple/map/map.pcd");
                Ten::Ten_relocation<pcl::PointXYZI> rel(log_path);
                Ten::XYZRPY xyzrpy = rel.get_transformation();
                if(xyzrpy == Ten::XYZRPY())
                {
                    return false;
                }

                std::cout << "---------------------------" << std::endl; 
                std::cout << "x: " << xyzrpy._xyz._x << std::endl;
                std::cout << "y: " << xyzrpy._xyz._y << std::endl;
                std::cout << "z: " << xyzrpy._xyz._z << std::endl;
                std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
                std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
                std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;

                Ten::XYZRPY xyzrpy_error;
                xyzrpy_error._xyz._x = _r1_xyzrpy_error_xyz_x_;
                xyzrpy_error._xyz._y = _r1_xyzrpy_error_xyz_y_;
                xyzrpy_error._xyz._z = _r1_xyzrpy_error_xyz_z_;
                xyzrpy_error._rpy._roll = _r1_xyzrpy_error_rpy_roll_;
                xyzrpy_error._rpy._pitch = _r1_xyzrpy_error_rpy_pitch_;
                xyzrpy_error._rpy._yaw = _r1_xyzrpy_error_rpy_yaw_;

                Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::worldtocurrent(xyzrpy._xyz, xyzrpy._rpy) * Ten::worldtocurrent(xyzrpy_error._xyz, xyzrpy_error._rpy) * Ten::_COORDINATE_TRANSFORMATION_.get_lidartocar());

                Ten::_COORDINATE_TRANSFORMATION_.set_world2toworld1(world_origin);
                return true;
            }

            static void serial_receiver()
            {
                urcu_memb_register_thread();
                std::string log_path = std::string(ROOT_DIR) + std::string("log");
                Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
                Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
                uint8_t arr[1000] = {0};
                //ros::Rate sl(10);
                while(Ten::_TREADPOOL_FLAG_.read_flag())
                {
                    uint8_t frame_id = 0;
                    uint8_t length = 0;
                    if(serial.serial_read(arr, frame_id, length))
                    {
                        uint8_t result[1] = {0};
                        if(frame_id == 4) //重定位
                        {
                            result[0] = calibration2();
                            log.record_map(Ten::_Map_GET_.read_data());
                            serial.clearBuffer(1);
                            serial.serial_send(result, 5, sizeof(result));       
                        }
                    }
                    //sl.sleep();
                    usleep(100*1000);
                }
                urcu_memb_unregister_thread();
            }

        private:

        };

    }

}


#endif
