#ifndef __CONTROLR2_H_
#define __CONTROLR2_H_
#include "./controlR1.h"
#include "./../PnP/pnp_main.h"

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
            
            Ten::PV pose_and_velocity_now;

            //nav_msgs::Odometry odo_n;
            ros::Rate sl(Ten::_laser_pub_hz_*2);
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                // 位置变化
                nav_msgs::Odometry odo;
                if(!Ten::_TF_GET_.pop(odo))
                {

                    sl.sleep();
                    continue;
                }
                Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
                Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
                Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
                
                if(result.XYZRPYisnan())
                {
                    sl.sleep();
                    continue;
                }

                float roll = result._rpy._roll;
                float pitch = result._rpy._pitch;
                float yaw = result._rpy._yaw;
                arr[0] = result._xyz._x;
                arr[1] = result._xyz._y;
                arr[2] = result._xyz._z;
                arr[3] = yaw;
                serial.serial_send(arr, 1, size);
                pose_and_velocity_now.pose = result;
                publishOdometry(pose_and_velocity_now, odo.header.stamp);
                sl.sleep();
            }
            
            urcu_memb_unregister_thread();
        }

        static void serial_send_lidarR2_ekf()
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
            
            Ten::PoseVelocityKalmanFilter ekf_fliter;
            Ten::PV pose_and_velocity_now;
            double last_time = 0.0;
            double curtime = 0.0;
            //publishOdometry(pose_and_velocity_now, ros::Time::now());
            //nav_msgs::Odometry odo_n;
            ros::Rate sl(Ten::_laser_pub_hz_*2);
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                // 位置变化
                nav_msgs::Odometry odo;
                if(!Ten::_TF_GET_.pop(odo))
                {
                    sl.sleep();
                    continue;
                }
                Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
                
                pose_and_velocity_now.pose = pose;
                //速度变化
                Ten::XYZRPY lidar_LA;
                lidar_LA._xyz._x = odo.twist.twist.linear.x;
                lidar_LA._xyz._y = odo.twist.twist.linear.y;
                lidar_LA._xyz._z = odo.twist.twist.linear.z;
                lidar_LA._rpy._roll = odo.twist.twist.angular.x;
                lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
                lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
                pose_and_velocity_now.velocity = lidar_LA;

                if(pose.XYZRPYisnan())
                {
                    sl.sleep();
                    continue;
                }

                //ekf
                curtime = odo.header.stamp.toSec();
                double dt = curtime - last_time;
                //时间是否为负
                if(dt <= 0.0)
                {
                    sl.sleep();
                    continue;
                }
                Ten::PV pose_and_velocity_ekf = ekf_fliter.process(pose_and_velocity_now, dt);
                last_time = curtime;
                pose = pose_and_velocity_ekf.pose;
                lidar_LA = pose_and_velocity_ekf.velocity;
                //变化
                Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
                Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
                pose_and_velocity_ekf.pose = result;


                float roll = result._rpy._roll;
                float pitch = result._rpy._pitch;
                float yaw = result._rpy._yaw;
                arr[0] = result._xyz._x;
                arr[1] = result._xyz._y;
                arr[2] = result._xyz._z;
                arr[3] = yaw;
                serial.serial_send(arr, 1, size);
                publishOdometry(pose_and_velocity_ekf, odo.header.stamp);
                sl.sleep();
            }
            
            urcu_memb_unregister_thread();
        }

        static void serial_send_lidarR2_ekf_imu()
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
            
            Ten::PoseVelocityKalmanFilter ekf_fliter;
            Ten::PV pose_and_velocity_now;
            double last_time = 0.0;
            double curtime = 0.0;
            //publishOdometry(pose_and_velocity_now, ros::Time::now());
            //nav_msgs::Odometry odo_n;

            //imu
            Ten::ImuOdometry predict;
            Ten::XYZRPY error; 

            ros::Rate sl(Ten::_laser_pub_hz_*2);
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                //插入imu数据
                sensor_msgs::Imu imudata;
                if(Ten::_IMU_GET_.pop(imudata))
                {
                    predict.addImuData(imudata);
                
                }
                // 位置变化
                nav_msgs::Odometry odo;
                if(!Ten::_TF_GET_.pop(odo))
                {
                    sl.sleep();
                    continue;
                }
                Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
                
                pose_and_velocity_now.pose = pose;
                //速度变化
                Ten::XYZRPY lidar_LA;
                lidar_LA._xyz._x = odo.twist.twist.linear.x;
                lidar_LA._xyz._y = odo.twist.twist.linear.y;
                lidar_LA._xyz._z = odo.twist.twist.linear.z;
                lidar_LA._rpy._roll = odo.twist.twist.angular.x;
                lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
                lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
                pose_and_velocity_now.velocity = lidar_LA;

                if(pose.XYZRPYisnan())
                {
                    sl.sleep();
                    continue;
                }
                
                //ekf
                curtime = odo.header.stamp.toSec();
                double dt = curtime - last_time;
                //时间是否为负
                if(dt <= 0.0)
                {
                    sl.sleep();
                    continue;
                }
                //ekf
                Ten::PV pose_and_velocity_ekf = ekf_fliter.process(pose_and_velocity_now, dt);
                //imu
                error = predict.processImu(lidar_LA._xyz, pose_and_velocity_now.pose._rpy);
                pose_and_velocity_ekf.pose += error;

                last_time = curtime;
                pose = pose_and_velocity_ekf.pose;
                lidar_LA = pose_and_velocity_ekf.velocity;
                //变化
                Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
                Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
                pose_and_velocity_ekf.pose = result;

                float roll = result._rpy._roll;
                float pitch = result._rpy._pitch;
                float yaw = result._rpy._yaw;
                arr[0] = result._xyz._x;
                arr[1] = result._xyz._y;
                arr[2] = result._xyz._z;
                arr[3] = yaw;
                serial.serial_send(arr, 1, size);
                publishOdometry(pose_and_velocity_ekf, odo.header.stamp + ros::Duration(0.1));
                sl.sleep();
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

            //Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::worldtocurrent(xyzrpy._xyz, xyzrpy._rpy) * Ten::worldtocurrent(xyzrpy_error._xyz, xyzrpy_error._rpy) * Ten::_COORDINATE_TRANSFORMATION_.get_lidartocar());
            Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::XYZRPYtotransform_matrix(xyzrpy) * Ten::XYZRPYtotransform_matrix(xyzrpy_error) * Ten::_COORDINATE_TRANSFORMATION_.get_lidartocar());
            Ten::_COORDINATE_TRANSFORMATION_.set_world2toworld1(world_origin);
            Ten::_PUB_CLOUD_FLAG_.set_flag(0);
            return true;
        }

        static void serial_receiver()
        {
            urcu_memb_register_thread();
            //定义各种数组
            uint8_t arr[1000] = {0};

            //初始化日志
            std::string log_path = std::string(ROOT_DIR) + std::string("log");
            Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
            Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
            
            //线程调度
            Ten::ThreadPool pool(4);
            int pool_request[10] = {0};
            std::vector<std::function<void()>> func_request;
            func_request.push_back(test_pnp);

            //ros::Rate sl(10);
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                uint8_t frame_id = 0;
                uint8_t length = 0;
                if(serial.serial_read(arr, frame_id, length))
                {
                    
                    if(frame_id == 4) //重定位
                    {
                        uint8_t result[1] = {0};
                        result[0] = calibration2();
                        sensor_msgs::PointCloud2 msg;
                        Ten::_Map_GET_.get_latest(msg);
                        log.record_map(msg);
                        serial.clearBuffer(1);
                        serial.serial_send(result, 5, sizeof(result));       
                    }
                    if(frame_id == 7) //camera_kfs
                    {
                        if(arr[0] == 1)
                        {
                            //如果第一次调用
                            if(pool_request[0] == 0)
                            {
                                pool.enqueue(func_request[0]);
                            }
                            pool_request[0] = 1;
                        }
                        else if(arr[0] == 0)
                        {
                            uint8_t result[1] = {0};
                            Ten::_CAMERA_KFS_FLAG_.set_flag(0);
                            serial.serial_send(result, 7, sizeof(result)); 
                            pool_request[0] = 0;
                        }   
                    }
                }
                //sl.sleep();
                usleep(10*1000);
            }
            urcu_memb_unregister_thread();
        }

        private:


        /**
         * @brief 发布 nav_msgs::Odometry 消息（静态发布者，调用一次发一次）
         * @param pv 输入：自定义位姿+速度结构体 Ten::PV
         * @param stamp 输入：自定义ROS时间戳（odom_msg.header.stamp）
         */
        static void publishOdometry(const Ten::PV& pv, const ros::Time& stamp, std::string topic = "/R2/odom")
        {
            // ====================== 静态发布者（仅初始化一次）======================
            static ros::Publisher odom_pub;
            if (!odom_pub)
            {
                ros::NodeHandle nh;
                odom_pub = nh.advertise<nav_msgs::Odometry>(topic, 10);
            }

            // ====================== 填充 Odometry 消息 ======================
            nav_msgs::Odometry odom_msg;

            // 1. 消息头部：使用外部传入的时间戳 ✅
            odom_msg.header.stamp = stamp; 
            odom_msg.header.frame_id = "odom";
            odom_msg.child_frame_id = "base_link";

            // 2. 位姿部分
            odom_msg.pose.pose.position.x = pv.pose._xyz._x;
            odom_msg.pose.pose.position.y = pv.pose._xyz._y;
            odom_msg.pose.pose.position.z = pv.pose._xyz._z;

            tf::Quaternion quat;
            quat.setRPY(pv.pose._rpy._roll, pv.pose._rpy._pitch, pv.pose._rpy._yaw);
            geometry_msgs::Quaternion q_msg;
            tf::quaternionTFToMsg(quat, q_msg);
            odom_msg.pose.pose.orientation = q_msg;

            // 3. 速度部分
            odom_msg.twist.twist.linear.x = pv.velocity._xyz._x;
            odom_msg.twist.twist.linear.y = pv.velocity._xyz._y;
            odom_msg.twist.twist.linear.z = pv.velocity._xyz._z;
            odom_msg.twist.twist.angular.x = pv.velocity._rpy._roll;
            odom_msg.twist.twist.angular.y = pv.velocity._rpy._pitch;
            odom_msg.twist.twist.angular.z = pv.velocity._rpy._yaw;

            // 4. 协方差矩阵
            odom_msg.pose.covariance.assign(0.0);
            odom_msg.twist.covariance.assign(0.0);

            // ====================== 发布消息 ======================
            odom_pub.publish(odom_msg);
        }
        
        };

    }

}


#endif
