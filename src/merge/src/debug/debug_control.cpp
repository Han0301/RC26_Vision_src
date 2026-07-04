#include "./../rcekf/rcekf.h"
#include "./../serial.h"
#include "./../openvino.h"
#include "./../threadpool.h"
#include "./..//livox_ros_driver2/src/livox_ros_driver.h"
#include "./..//point_lio/src/laserMapping2.h"
#include "./../lidar.h"
#include "./../camera.h"
// #include "test.cpp"
// #include "test2.cpp"
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
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include "./../rcekf/predicted.h"


/**
 * @brief 发布 nav_msgs::Odometry 消息（静态发布者，调用一次发一次）
 * @param pv 输入：自定义位姿+速度结构体 Ten::PV
 * @param stamp 输入：自定义ROS时间戳（odom_msg.header.stamp）
 */
void publishOdometryFromPVraw(const Ten::PV& pv, const ros::Time& stamp)
{
    // ====================== 静态发布者（仅初始化一次）======================
    static ros::Publisher odom_pub;
    if (!odom_pub)
    {
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("/point_lio/raw", 10);
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

/**
 * @brief 发布 nav_msgs::Odometry 消息（静态发布者，调用一次发一次）
 * @param pv 输入：自定义位姿+速度结构体 Ten::PV
 * @param stamp 输入：自定义ROS时间戳（odom_msg.header.stamp）
 */
void publishOdometryFromPVekf(const Ten::PV& pv, const ros::Time& stamp)
{
    // ====================== 静态发布者（仅初始化一次）======================
    static ros::Publisher odom_pub;
    if (!odom_pub)
    {
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("/point_lio/ekf", 10);
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

/**
 * @brief 发布 nav_msgs::Odometry 消息（静态发布者，调用一次发一次）
 * @param pv 输入：自定义位姿+速度结构体 Ten::PV
 * @param stamp 输入：自定义ROS时间戳（odom_msg.header.stamp）
 */
void publishOdometryFromPVfast(const Ten::PV& pv, const ros::Time& stamp)
{
    // ====================== 静态发布者（仅初始化一次）======================
    static ros::Publisher odom_pub;
    if (!odom_pub)
    {
        ros::NodeHandle nh;
        odom_pub = nh.advertise<nav_msgs::Odometry>("/fast/call_back", 10);
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

// 里程计消息回调函数
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    //Ten::_TF_GET2_.write_data(*msg);
    Ten::_TF_GET2_.push(*msg);
    Ten::PV pose_and_velocity_now;
    Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(*msg);
    Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
    Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
    pose_and_velocity_now.pose = result;
    publishOdometryFromPVfast(pose_and_velocity_now, ros::Time::now());
}


void Loopcallback()
{
    ros::NodeHandle nh("/");
    ros::Subscriber odom_sub = nh.subscribe("/fast_lio2/odom", 10, odomCallback);
    ros::Rate loop_rate(20);  
    while (Ten::_TREADPOOL_FLAG_.read_flag())  
    {
        ros::spinOnce();  
        loop_rate.sleep();  
    }
}


void calibration2()
{
    std::string log_path = std::string(ROOT_DIR) + std::string("map/map.pcd");
    //std::string log_path = std::string("/home/maple/study2/maple/map/map.pcd");
    Ten::Ten_relocation<pcl::PointXYZI> rel(log_path);
    Ten::XYZRPY xyzrpy = rel.get_transformation();

    std::cout << "-------------get_transformation--------------" << std::endl; 
    std::cout << "x: " << xyzrpy._xyz._x << std::endl;
    std::cout << "y: " << xyzrpy._xyz._y << std::endl;
    std::cout << "z: " << xyzrpy._xyz._z << std::endl;
    std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
    std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
    std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;

    Ten::XYZRPY xyzrpy_error;
    // xyzrpy_error._xyz._x = Ten::superstratum::_r2_xyzrpy_error_xyz_x_;
    // xyzrpy_error._xyz._y = Ten::superstratum::_r2_xyzrpy_error_xyz_y_;
    // xyzrpy_error._xyz._z = Ten::superstratum::_r2_xyzrpy_error_xyz_z_;
    // xyzrpy_error._rpy._roll = Ten::superstratum::_r2_xyzrpy_error_rpy_roll_;
    // xyzrpy_error._rpy._pitch = Ten::superstratum::_r2_xyzrpy_error_rpy_pitch_;
    // xyzrpy_error._rpy._yaw = Ten::superstratum::_r2_xyzrpy_error_rpy_yaw_;
    Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::XYZRPYtotransform_matrix(xyzrpy) * Ten::XYZRPYtotransform_matrix(xyzrpy_error) * Ten::_COORDINATE_TRANSFORMATION_.get_lidartocar());
    Ten::_COORDINATE_TRANSFORMATION_.set_world2toworld1(world_origin);
}

void calibration3()
{
    std::string log_path = std::string(ROOT_DIR) + std::string("map/map.pcd");
    std::string log_path2 = std::string(ROOT_DIR) + std::string("map/map2.pcd");
    std::string log_path3 = std::string(ROOT_DIR) + std::string("map/map3.pcd");
    //std::string log_path = std::string("/home/maple/study2/maple/map/map.pcd");
    Ten::Ten_relocation<pcl::PointXYZI> rel(log_path, log_path2, log_path3);
    Ten::XYZRPY xyzrpy = rel.get_transformation(2);

    std::cout << "-----------get_transformation----------------" << std::endl; 
    std::cout << "x: " << xyzrpy._xyz._x << std::endl;
    std::cout << "y: " << xyzrpy._xyz._y << std::endl;
    std::cout << "z: " << xyzrpy._xyz._z << std::endl;
    std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
    std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
    std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = Ten::superstratum::_r2_xyzrpy_error_xyz_x_;
    xyzrpy_error._xyz._y = Ten::superstratum::_r2_xyzrpy_error_xyz_y_;
    xyzrpy_error._xyz._z = Ten::superstratum::_r2_xyzrpy_error_xyz_z_;
    xyzrpy_error._rpy._roll = Ten::superstratum::_r2_xyzrpy_error_rpy_roll_;
    xyzrpy_error._rpy._pitch = Ten::superstratum::_r2_xyzrpy_error_rpy_pitch_;
    xyzrpy_error._rpy._yaw = Ten::superstratum::_r2_xyzrpy_error_rpy_yaw_;
    Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::XYZRPYtotransform_matrix(xyzrpy) * Ten::XYZRPYtotransform_matrix(xyzrpy_error) * Ten::_COORDINATE_TRANSFORMATION_.get_lidartocar());
    Ten::_COORDINATE_TRANSFORMATION_.set_world2toworld1(world_origin);
}

void test_input()
{
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        int flag = 0;
        std::cin >> flag;
        if(flag == 1)
        {
            std::cout<< "flag == 1" << std::endl;
            calibration3();
        }
        else if(flag == 0)
        {
            break;
        }
    }
}

void test_lidar_point_lio()
{
    urcu_memb_register_thread();
    //ros::Rate sl(Ten::_laser_pub_hz_);
    //ros::Rate sl(1);
    ros::Rate sll(Ten::_laser_pub_hz_*2); 
    int num = Ten::_laser_pub_hz_;
    std::cout << "num: " << num << std::endl;

    // Ten::XYZRPY xyzrpy_error;
    // xyzrpy_error._xyz._x = Ten::superstratum::_r2_xyzrpy_init_error_xyz_x_;
    // xyzrpy_error._xyz._y = Ten::superstratum::_r2_xyzrpy_init_error_xyz_y_;
    // xyzrpy_error._xyz._z = Ten::superstratum::_r2_xyzrpy_init_error_xyz_z_;
    // xyzrpy_error._rpy._roll = Ten::superstratum::_r2_xyzrpy_init_error_rpy_roll_;
    // xyzrpy_error._rpy._pitch = Ten::superstratum::_r2_xyzrpy_init_error_rpy_pitch_;
    // xyzrpy_error._rpy._yaw = Ten::superstratum::_r2_xyzrpy_init_error_rpy_yaw_;
    // Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    // Ten::XYZRPY xyzrpy_car;
    // xyzrpy_car._xyz._x = Ten::superstratum::_r2_xyzrpy_car_xyz_x_;
    // xyzrpy_car._xyz._y = Ten::superstratum::_r2_xyzrpy_car_xyz_y_;
    // xyzrpy_car._xyz._z = Ten::superstratum::_r2_xyzrpy_car_xyz_z_;
    // xyzrpy_car._rpy._roll = Ten::superstratum::_r2_xyzrpy_car_rpy_roll_;
    // xyzrpy_car._rpy._pitch = Ten::superstratum::_r2_xyzrpy_car_rpy_pitch_;
    // xyzrpy_car._rpy._yaw = Ten::superstratum::_r2_xyzrpy_car_rpy_yaw_;
    // Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 

    Ten::PoseVelocityKalmanFilter ekf_fliter;
    Ten::PV pose_and_velocity_now;
    double last_time = 0.0;
    double curtime = 0.0;

    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo;
        if(!Ten::_TF_GET_.pop(odo))
        {
            sll.sleep();
            continue;
        }

        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);

        if(num <= 0)
        {
            std::cout << "--------------pose----------" << std::endl;

            std::cout << "x: " << pose._xyz._x << std::endl;
            std::cout << "y: " << pose._xyz._y << std::endl;
            std::cout << "z: " << pose._xyz._z << std::endl;

            std::cout << "roll: " << pose._rpy._roll << std::endl;
            std::cout << "pitch: " << pose._rpy._pitch << std::endl;
            std::cout << "yaw: " << pose._rpy._yaw << std::endl;

            std::cout << "--------------pose----------" << std::endl;
        }
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
            sll.sleep();
            continue;
        }

        //ekf
        curtime = odo.header.stamp.toSec();
        double dt = curtime - last_time;
        //时间是否为负
        if(dt <= 0.0)
        {
            sll.sleep();
            continue;
        }

        Ten::PV pose_and_velocity_ekf = ekf_fliter.process(pose_and_velocity_now, dt);
        last_time = curtime;
        pose = pose_and_velocity_ekf.pose;
        lidar_LA = pose_and_velocity_ekf.velocity;

        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();


        pose_and_velocity_ekf.pose = result;
        
        if(num <= 0)
        {
            std::cout << "--------------result----------" << std::endl;

            std::cout << "x: " << result._xyz._x << std::endl;
            std::cout << "y: " << result._xyz._y << std::endl;
            std::cout << "z: " << result._xyz._z << std::endl;

            std::cout << "roll: " << result._rpy._roll << std::endl;
            std::cout << "pitch: " << result._rpy._pitch << std::endl;
            std::cout << "yaw: " << result._rpy._yaw << std::endl;

            std::cout << "--------------result----------" << std::endl;
            num = Ten::_laser_pub_hz_;
        }
        num--;
        publishOdometryFromPVekf(pose_and_velocity_ekf, odo.header.stamp);
        sll.sleep();
    }

    urcu_memb_unregister_thread();
}


void test_lidar_point_lio2()
{
    urcu_memb_register_thread();
    //ros::Rate sl(Ten::_laser_pub_hz_);
    ros::Rate sl(1);
    ros::Rate sll(Ten::_laser_pub_hz_*2); 
 

    // Ten::XYZRPY xyzrpy_error;
    // xyzrpy_error._xyz._x = Ten::superstratum::_r2_xyzrpy_init_error_xyz_x_;
    // xyzrpy_error._xyz._y = Ten::superstratum::_r2_xyzrpy_init_error_xyz_y_;
    // xyzrpy_error._xyz._z = Ten::superstratum::_r2_xyzrpy_init_error_xyz_z_;
    // xyzrpy_error._rpy._roll = Ten::superstratum::_r2_xyzrpy_init_error_rpy_roll_;
    // xyzrpy_error._rpy._pitch = Ten::superstratum::_r2_xyzrpy_init_error_rpy_pitch_;
    // xyzrpy_error._rpy._yaw = Ten::superstratum::_r2_xyzrpy_init_error_rpy_yaw_;
    // Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    // Ten::XYZRPY xyzrpy_car;
    // xyzrpy_car._xyz._x = Ten::superstratum::_r2_xyzrpy_car_xyz_x_;
    // xyzrpy_car._xyz._y = Ten::superstratum::_r2_xyzrpy_car_xyz_y_;
    // xyzrpy_car._xyz._z = Ten::superstratum::_r2_xyzrpy_car_xyz_z_;
    // xyzrpy_car._rpy._roll = Ten::superstratum::_r2_xyzrpy_car_rpy_roll_;
    // xyzrpy_car._rpy._pitch = Ten::superstratum::_r2_xyzrpy_car_rpy_pitch_;
    // xyzrpy_car._rpy._yaw = Ten::superstratum::_r2_xyzrpy_car_rpy_yaw_;
    // Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 

    Ten::PoseVelocityKalmanFilter ekf_fliter;
    Ten::PV pose_and_velocity_now;
    double last_time = 0.0;
    double curtime = 0.0;

    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo;
        if(!Ten::_TF_GET_.pop(odo))
        {
            sll.sleep();
            continue;
        }

        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);


            std::cout << "--------------pose----------" << std::endl;

            std::cout << "x: " << pose._xyz._x << std::endl;
            std::cout << "y: " << pose._xyz._y << std::endl;
            std::cout << "z: " << pose._xyz._z << std::endl;

            std::cout << "roll: " << pose._rpy._roll << std::endl;
            std::cout << "pitch: " << pose._rpy._pitch << std::endl;
            std::cout << "yaw: " << pose._rpy._yaw << std::endl;

            std::cout << "--------------pose----------" << std::endl;

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
            sll.sleep();
            continue;
        }

        //ekf
        curtime = odo.header.stamp.toSec();
        double dt = curtime - last_time;
        //时间是否为负
        if(dt <= 0.0)
        {
            sll.sleep();
            continue;
        }

        Ten::PV pose_and_velocity_ekf = ekf_fliter.process(pose_and_velocity_now, dt);
        last_time = curtime;
        pose = pose_and_velocity_ekf.pose;
        lidar_LA = pose_and_velocity_ekf.velocity;

        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();


        pose_and_velocity_ekf.pose = result;

            std::cout << "--------------result----------" << std::endl;

            std::cout << "x: " << result._xyz._x << std::endl;
            std::cout << "y: " << result._xyz._y << std::endl;
            std::cout << "z: " << result._xyz._z << std::endl;

            std::cout << "roll: " << result._rpy._roll << std::endl;
            std::cout << "pitch: " << result._rpy._pitch << std::endl;
            std::cout << "yaw: " << result._rpy._yaw << std::endl;

            std::cout << "--------------result----------" << std::endl;

        publishOdometryFromPVekf(pose_and_velocity_ekf, odo.header.stamp);
        sl.sleep();
    }

    urcu_memb_unregister_thread();
}

void test_lidar_ekf_of_point_lio()
{
    urcu_memb_register_thread();
    ros::Rate sl(Ten::_laser_pub_hz_*2);
    Ten::PoseVelocityKalmanFilter ekf_fliter;
    Ten::PV pose_and_velocity_now;
    double last_time = 0.0;
    double curtime = 0.0;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo;
        if(!Ten::_TF_GET_.pop(odo))
        {
            //std::cout << "!Ten::_TF_GET_.pop(odo): "<<std::endl;
            sl.sleep();
            continue;
        }
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        // std::cout << "--------------pose----------" << std::endl;

        // std::cout << "x: " << pose._xyz._x << std::endl;
        // std::cout << "y: " << pose._xyz._y << std::endl;
        // std::cout << "z: " << pose._xyz._z << std::endl;

        // std::cout << "roll: " << pose._rpy._roll << std::endl;
        // std::cout << "pitch: " << pose._rpy._pitch << std::endl;
        // std::cout << "yaw: " << pose._rpy._yaw << std::endl;

        // std::cout << "--------------pose----------" << std::endl;
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);

        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
        //数据是否无效
        if(result.XYZRPYisnan())
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
            //std::cout << "nnndt: " << dt <<std::endl;
            sl.sleep();
            continue;
        }
        else
        {
            //std::cout << "dt: " << dt <<std::endl;
        }
        pose_and_velocity_now.pose = result;
        //速度变化
        Ten::XYZRPY lidar_LA;
        lidar_LA._xyz._x = odo.twist.twist.linear.x;
        lidar_LA._xyz._y = odo.twist.twist.linear.y;
        lidar_LA._xyz._z = odo.twist.twist.linear.z;
        lidar_LA._rpy._roll = odo.twist.twist.angular.x;
        lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
        lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
        pose_and_velocity_now.velocity = lidar_LA;

        Ten::PV pose_and_velocity_ekf = ekf_fliter.process(pose_and_velocity_now, dt);
        last_time = curtime;
        //发布调试数据
        // publishOdometryFromPVraw(pose_and_velocity_now, odo.header.stamp);
        // publishOdometryFromPVekf(pose_and_velocity_ekf, odo.header.stamp);
        publishOdometryFromPVraw(pose_and_velocity_now, ros::Time::now());
        publishOdometryFromPVekf(pose_and_velocity_ekf, ros::Time::now());
        sl.sleep();
    }

    urcu_memb_unregister_thread();
}


void test_lidar_fast_lio()
{
    urcu_memb_register_thread();
    ros::Rate sl(10);

    Ten::PV pose_and_velocity_now;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo;
        if(!Ten::_TF_GET2_.pop(odo))
        {
            sl.sleep();
            continue;
        }
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        // std::cout << "--------------pose----------" << std::endl;

        // std::cout << "x: " << pose._xyz._x << std::endl;
        // std::cout << "y: " << pose._xyz._y << std::endl;
        // std::cout << "z: " << pose._xyz._z << std::endl;

        // std::cout << "roll: " << pose._rpy._roll << std::endl;
        // std::cout << "pitch: " << pose._rpy._pitch << std::endl;
        // std::cout << "yaw: " << pose._rpy._yaw << std::endl;

        // std::cout << "--------------pose----------" << std::endl;
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
        //数据是否无效
        if(result.XYZRPYisnan())
        {
            sl.sleep();
            continue;
        }
        pose_and_velocity_now.pose = result;
        // std::cout << "--------------fast_lio_xyzrpy----------" << std::endl;

        // std::cout << "x: " << result._xyz._x << std::endl;
        // std::cout << "y: " << result._xyz._y << std::endl;
        // std::cout << "z: " << result._xyz._z << std::endl;

        // std::cout << "roll: " << result._rpy._roll << std::endl;
        // std::cout << "pitch: " << result._rpy._pitch << std::endl;
        // std::cout << "yaw: " << result._rpy._yaw << std::endl;

        // std::cout << "--------------fast_lio_xyzrpy----------" << std::endl;
        //publishOdometryFromPVfast(pose_and_velocity_now, odo.header.stamp);
        publishOdometryFromPVfast(pose_and_velocity_now, ros::Time::now());
        sl.sleep();
    }

    urcu_memb_unregister_thread();
}



int vision_test_relocation2()
{
    std::string global_pcd_path = "/home/maple/study3/robocon_ws13/src/merge/map/map.pcd";
    std::string local_pcd_path = "/home/maple/study3/maple/log/2026_5_24/7/map/map2.pcd";
    Ten::Ten_relocation<pcl::PointXYZI> rel("/home/maple/study3/robocon_ws13/src/merge/map/map.pcd");
    pcl::PointCloud<pcl::PointXYZI>::Ptr global_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr local_cloud(new pcl::PointCloud<pcl::PointXYZI>); 
    pcl::io::loadPCDFile<pcl::PointXYZI>(global_pcd_path, *global_cloud);
    pcl::io::loadPCDFile<pcl::PointXYZI>(local_pcd_path, *local_cloud);

    // Ten::XYZ xyz;
    // xyz._x = 1 ;
    // xyz._y = 2 ;
    // xyz._z = 3 ;

    // Ten::RPY rpy;
    // rpy._roll = 0.52;
    // rpy._pitch = 0.52;
    // rpy._yaw = 0.52;

    // Eigen::Matrix4d T = Ten::worldtocurrent(xyz, rpy);
    // pcl::transformPointCloud(*global_cloud, *local_cloud, T);

    Ten::XYZRPY xyzrpy = rel.get_transformation(local_cloud);

    std::cout << "---------------------------" << std::endl; 
    std::cout << "x: " << xyzrpy._xyz._x << std::endl;
    std::cout << "y: " << xyzrpy._xyz._y << std::endl;
    std::cout << "z: " << xyzrpy._xyz._z << std::endl;
    std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
    std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
    std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;
    return 0;
}

void test_lidar_point_lio_imu()
{
    urcu_memb_register_thread();
    //ros::Rate sl(Ten::_laser_pub_hz_);
    //ros::Rate sl(1);
    ros::Rate sll(Ten::_laser_pub_hz_*2); 
    int num = Ten::_laser_pub_hz_;
    std::cout << "num: " << num << std::endl;

    // Ten::XYZRPY xyzrpy_error;
    // xyzrpy_error._xyz._x = Ten::superstratum::_r2_xyzrpy_init_error_xyz_x_;
    // xyzrpy_error._xyz._y = Ten::superstratum::_r2_xyzrpy_init_error_xyz_y_;
    // xyzrpy_error._xyz._z = Ten::superstratum::_r2_xyzrpy_init_error_xyz_z_;
    // xyzrpy_error._rpy._roll = Ten::superstratum::_r2_xyzrpy_init_error_rpy_roll_;
    // xyzrpy_error._rpy._pitch = Ten::superstratum::_r2_xyzrpy_init_error_rpy_pitch_;
    // xyzrpy_error._rpy._yaw = Ten::superstratum::_r2_xyzrpy_init_error_rpy_yaw_;
    // Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    // Ten::XYZRPY xyzrpy_car;
    // xyzrpy_car._xyz._x = Ten::superstratum::_r2_xyzrpy_car_xyz_x_;
    // xyzrpy_car._xyz._y = Ten::superstratum::_r2_xyzrpy_car_xyz_y_;
    // xyzrpy_car._xyz._z = Ten::superstratum::_r2_xyzrpy_car_xyz_z_;
    // xyzrpy_car._rpy._roll = Ten::superstratum::_r2_xyzrpy_car_rpy_roll_;
    // xyzrpy_car._rpy._pitch = Ten::superstratum::_r2_xyzrpy_car_rpy_pitch_;
    // xyzrpy_car._rpy._yaw = Ten::superstratum::_r2_xyzrpy_car_rpy_yaw_;
    // Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 

    Ten::PoseVelocityKalmanFilter ekf_fliter;
    Ten::PV pose_and_velocity_now;
    double last_time = 0.0;
    double curtime = 0.0;

    Ten::ImuOdometry predict;
    Ten::XYZRPY error; 

    while (Ten::_TREADPOOL_FLAG_.read_flag())
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
            sll.sleep();
            continue;
        }

        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);

        if(num <= 0)
        {
            std::cout << "--------------pose----------" << std::endl;

            std::cout << "x: " << pose._xyz._x << std::endl;
            std::cout << "y: " << pose._xyz._y << std::endl;
            std::cout << "z: " << pose._xyz._z << std::endl;

            std::cout << "roll: " << pose._rpy._roll << std::endl;
            std::cout << "pitch: " << pose._rpy._pitch << std::endl;
            std::cout << "yaw: " << pose._rpy._yaw << std::endl;

            std::cout << "--------------pose----------" << std::endl;
        }
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
            sll.sleep();
            continue;
        }

        //ekf
        curtime = odo.header.stamp.toSec();
        double dt = curtime - last_time;
        //时间是否为负
        if(dt <= 0.0)
        {
            sll.sleep();
            continue;
        }


        Ten::PV pose_and_velocity_ekf = ekf_fliter.process(pose_and_velocity_now, dt);
        last_time = curtime;
        pose = pose_and_velocity_ekf.pose;
        lidar_LA = pose_and_velocity_ekf.velocity;

        // Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        // Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();

        //pose_and_velocity_ekf.pose = result;

        //测试
        pose_and_velocity_ekf = pose_and_velocity_now;


        error = predict.processImu(lidar_LA._xyz, pose_and_velocity_now.pose._rpy);
        pose_and_velocity_ekf.pose += error;
        
        
        if(num <= 0)
        {
            std::cout << "--------------pose_and_velocity_ekf.pose----------" << std::endl;

            std::cout << "x: " << pose_and_velocity_ekf.pose._xyz._x << std::endl;
            std::cout << "y: " << pose_and_velocity_ekf.pose._xyz._y << std::endl;
            std::cout << "z: " << pose_and_velocity_ekf.pose._xyz._z << std::endl;

            std::cout << "roll: " << pose_and_velocity_ekf.pose._rpy._roll << std::endl;
            std::cout << "pitch: " << pose_and_velocity_ekf.pose._rpy._pitch << std::endl;
            std::cout << "yaw: " << pose_and_velocity_ekf.pose._rpy._yaw << std::endl;

            std::cout << "--------------pose_and_velocity_ekf.pose----------" << std::endl;

            std::cout << "--------------error----------" << std::endl;

            std::cout << "x: " << error._xyz._x << std::endl;
            std::cout << "y: " << error._xyz._y << std::endl;
            std::cout << "z: " << error._xyz._z << std::endl;

            std::cout << "roll: " << error._rpy._roll << std::endl;
            std::cout << "pitch: " << error._rpy._pitch << std::endl;
            std::cout << "yaw: " << error._rpy._yaw << std::endl;

            std::cout << "--------------error----------" << std::endl;

            num = Ten::_laser_pub_hz_;
        }
        num--;


        ros::Time stamp = odo.header.stamp;
        ros::Time new_stamp = stamp + ros::Duration(0.1);

        publishOdometryFromPVraw(pose_and_velocity_now, stamp);
        publishOdometryFromPVekf(pose_and_velocity_ekf, new_stamp);
        sll.sleep();
    }

    urcu_memb_unregister_thread();
}

void test_lidar_point_lio_imu2()
{
    urcu_memb_register_thread();
    //ros::Rate sl(Ten::_laser_pub_hz_);
    //ros::Rate sl(1);
    ros::Rate sll(Ten::_laser_pub_hz_*2); 
    int num = Ten::_laser_pub_hz_;
    std::cout << "num: " << num << std::endl;

    // Ten::XYZRPY xyzrpy_error;
    // xyzrpy_error._xyz._x = Ten::superstratum::_r2_xyzrpy_init_error_xyz_x_;
    // xyzrpy_error._xyz._y = Ten::superstratum::_r2_xyzrpy_init_error_xyz_y_;
    // xyzrpy_error._xyz._z = Ten::superstratum::_r2_xyzrpy_init_error_xyz_z_;
    // xyzrpy_error._rpy._roll = Ten::superstratum::_r2_xyzrpy_init_error_rpy_roll_;
    // xyzrpy_error._rpy._pitch = Ten::superstratum::_r2_xyzrpy_init_error_rpy_pitch_;
    // xyzrpy_error._rpy._yaw = Ten::superstratum::_r2_xyzrpy_init_error_rpy_yaw_;
    // Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    // Ten::XYZRPY xyzrpy_car;
    // xyzrpy_car._xyz._x = Ten::superstratum::_r2_xyzrpy_car_xyz_x_;
    // xyzrpy_car._xyz._y = Ten::superstratum::_r2_xyzrpy_car_xyz_y_;
    // xyzrpy_car._xyz._z = Ten::superstratum::_r2_xyzrpy_car_xyz_z_;
    // xyzrpy_car._rpy._roll = Ten::superstratum::_r2_xyzrpy_car_rpy_roll_;
    // xyzrpy_car._rpy._pitch = Ten::superstratum::_r2_xyzrpy_car_rpy_pitch_;
    // xyzrpy_car._rpy._yaw = Ten::superstratum::_r2_xyzrpy_car_rpy_yaw_;
    // Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 

    Ten::PoseVelocityKalmanFilter ekf_fliter;
    Ten::PV pose_and_velocity_now;
    double last_time = 0.0;
    double curtime = 0.0;

    Ten::ImuOdometry predict;
    Ten::XYZRPY error; 

    while (Ten::_TREADPOOL_FLAG_.read_flag())
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
            sll.sleep();
            continue;
        }

        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);

        if(num <= 0)
        {
            std::cout << "--------------pose----------" << std::endl;

            std::cout << "x: " << pose._xyz._x << std::endl;
            std::cout << "y: " << pose._xyz._y << std::endl;
            std::cout << "z: " << pose._xyz._z << std::endl;

            std::cout << "roll: " << pose._rpy._roll << std::endl;
            std::cout << "pitch: " << pose._rpy._pitch << std::endl;
            std::cout << "yaw: " << pose._rpy._yaw << std::endl;

            std::cout << "--------------pose----------" << std::endl;
        }

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
            sll.sleep();
            continue;
        }

        //ekf
        curtime = odo.header.stamp.toSec();
        double dt = curtime - last_time;
        //时间是否为负
        if(dt <= 0.0)
        {
            sll.sleep();
            continue;
        }


        Ten::PV pose_and_velocity_ekf = ekf_fliter.process(pose_and_velocity_now, dt);
        pose_and_velocity_now = pose_and_velocity_ekf;
        
        last_time = curtime;
        error = predict.processImu(lidar_LA._xyz, pose_and_velocity_now.pose._rpy);
        pose_and_velocity_ekf.pose += error;
        
        pose = pose_and_velocity_ekf.pose;
        lidar_LA = pose_and_velocity_ekf.velocity;

        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();

        pose_and_velocity_ekf.pose = result;

        //测试
        //pose_and_velocity_ekf = pose_and_velocity_now;
 

        
        
        
        if(num <= 0)
        {
            std::cout << "--------------pose_and_velocity_now.pose----------" << std::endl;

            std::cout << "x: " << pose_and_velocity_now.pose._xyz._x << std::endl;
            std::cout << "y: " << pose_and_velocity_now.pose._xyz._y << std::endl;
            std::cout << "z: " << pose_and_velocity_now.pose._xyz._z << std::endl;

            std::cout << "roll: " << pose_and_velocity_now.pose._rpy._roll << std::endl;
            std::cout << "pitch: " << pose_and_velocity_now.pose._rpy._pitch << std::endl;
            std::cout << "yaw: " << pose_and_velocity_now.pose._rpy._yaw << std::endl;

            std::cout << "--------------pose_and_velocity_now.pose----------" << std::endl;

            std::cout << "--------------pose_and_velocity_ekf.pose----------" << std::endl;

            std::cout << "x: " << pose_and_velocity_ekf.pose._xyz._x << std::endl;
            std::cout << "y: " << pose_and_velocity_ekf.pose._xyz._y << std::endl;
            std::cout << "z: " << pose_and_velocity_ekf.pose._xyz._z << std::endl;

            std::cout << "roll: " << pose_and_velocity_ekf.pose._rpy._roll << std::endl;
            std::cout << "pitch: " << pose_and_velocity_ekf.pose._rpy._pitch << std::endl;
            std::cout << "yaw: " << pose_and_velocity_ekf.pose._rpy._yaw << std::endl;

            std::cout << "--------------pose_and_velocity_ekf.pose----------" << std::endl;

            std::cout << "--------------error----------" << std::endl;

            std::cout << "x: " << error._xyz._x << std::endl;
            std::cout << "y: " << error._xyz._y << std::endl;
            std::cout << "z: " << error._xyz._z << std::endl;

            std::cout << "roll: " << error._rpy._roll << std::endl;
            std::cout << "pitch: " << error._rpy._pitch << std::endl;
            std::cout << "yaw: " << error._rpy._yaw << std::endl;

            std::cout << "--------------error----------" << std::endl;

            num = Ten::_laser_pub_hz_;
        }
        num--;


        ros::Time stamp = odo.header.stamp;
        ros::Time new_stamp = stamp + ros::Duration(0.1);

        publishOdometryFromPVraw(pose_and_velocity_now, stamp);
        publishOdometryFromPVekf(pose_and_velocity_ekf, new_stamp);
        sll.sleep();
    }

    urcu_memb_unregister_thread();
}