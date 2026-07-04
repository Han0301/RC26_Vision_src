#ifndef __CONTROLR2_H_
#define __CONTROLR2_H_
#include "./controlR1.h"

namespace Ten
{
    namespace superstratum
    {

        // #define _r2_xyzrpy_car_xyz_x_ 0
        // #define _r2_xyzrpy_car_xyz_y_ 0.28
        // #define _r2_xyzrpy_car_xyz_z_ 0
        // #define _r2_xyzrpy_car_rpy_roll_ 0
        // #define _r2_xyzrpy_car_rpy_pitch_ 0
        // #define _r2_xyzrpy_car_rpy_yaw_ 0

        // #define _r2_xyzrpy_error_xyz_x_ 0.025
        // #define _r2_xyzrpy_error_xyz_y_ -0.045
        // #define _r2_xyzrpy_error_xyz_z_ 0.10
        // #define _r2_xyzrpy_error_rpy_roll_ 0
        // #define _r2_xyzrpy_error_rpy_pitch_ 0
        // #define _r2_xyzrpy_error_rpy_yaw_ 0

        class controlR2
        {
        public:
        static void serial_send_lidarR2()
        {
            urcu_memb_register_thread();
            Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
            float arr[4] = {0};
            size_t size = sizeof(arr);
            size_t num = size / sizeof(float);
        
            Ten::XYZRPY xyzrpy_error;
            xyzrpy_error._xyz._x = _r2_xyzrpy_init_error_xyz_x_;
            xyzrpy_error._xyz._y = _r2_xyzrpy_init_error_xyz_y_;
            xyzrpy_error._xyz._z = _r2_xyzrpy_init_error_xyz_z_;
            xyzrpy_error._rpy._roll = _r2_xyzrpy_init_error_rpy_roll_;
            xyzrpy_error._rpy._pitch = _r2_xyzrpy_init_error_rpy_pitch_;
            xyzrpy_error._rpy._yaw = _r2_xyzrpy_init_error_rpy_yaw_;
            Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);
        
            Ten::XYZRPY xyzrpy_car;
            xyzrpy_car._xyz._x = _r2_xyzrpy_car_xyz_x_;
            xyzrpy_car._xyz._y = _r2_xyzrpy_car_xyz_y_;
            xyzrpy_car._xyz._z = _r2_xyzrpy_car_xyz_z_;
            xyzrpy_car._rpy._roll = _r2_xyzrpy_car_rpy_roll_;
            xyzrpy_car._rpy._pitch = _r2_xyzrpy_car_rpy_pitch_;
            xyzrpy_car._rpy._yaw = _r2_xyzrpy_car_rpy_yaw_;
            Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 
            
            //nav_msgs::Odometry odo_n;
            //ros::Rate sl(100);
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
                Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
                Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
                Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();
                
                if(result.XYZRPYisnan())
                {
                    usleep(10*1000);
                    continue;
                }
        
                float roll = result._rpy._roll;
                float pitch = result._rpy._pitch;
                float yaw = result._rpy._yaw;
        
                arr[0] = result._xyz._x;
                arr[1] = result._xyz._y;
                arr[2] = result._xyz._z;
                arr[3] = yaw;
        
                // int flag = 0;
                // for(size_t i = 0; i < num; i++)
                // {
                //     if(std::isnan(arr[i])) 
                //     {
                //         flag = 1;
                //         break;
                //     }
                // }
        
                // if(flag)
                // {
                //     continue;
                // }
                
        
                serial.serial_send(arr, 1, size);
                //std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;
        
        
        
                // odo_n.twist.twist.linear.x = result._xyz._x;
                // odo_n.twist.twist.linear.y = result._xyz._y;
                // odo_n.twist.twist.linear.z = result._xyz._z;
        
                // odo_n.twist.twist.linear.x = car_LA._xyz._x;
                // odo_n.twist.twist.linear.y = car_LA._xyz._y;
                // odo_n.twist.twist.linear.z = car_LA._xyz._z;
        
                //Ten::Ten_logger::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/log").record_odometry(odo_n);
        
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
            xyzrpy_error._xyz._x = _r2_xyzrpy_error_xyz_x_;
            xyzrpy_error._xyz._y = _r2_xyzrpy_error_xyz_y_;
            xyzrpy_error._xyz._z = _r2_xyzrpy_error_xyz_z_;
            xyzrpy_error._rpy._roll = _r2_xyzrpy_error_rpy_roll_;
            xyzrpy_error._rpy._pitch = _r2_xyzrpy_error_rpy_pitch_;
            xyzrpy_error._rpy._yaw = _r2_xyzrpy_error_rpy_yaw_;

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
