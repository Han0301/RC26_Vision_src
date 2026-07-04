
#ifndef __TEST_CPP_
#define __TEST_CPP_
#include "openvino.h"
#include "./livox_ros_driver2/src/livox_ros_driver.h"
#include "threadpool.h"
#include "./point_lio/src/laserMapping2.h"
#include "serial.h"
#include "camera.h"
#include "method_math.h"
#include "lidar.h"
#include "recognition/zbuffer_simplify.h"
#include "relocation.h"
#include "coordinate.h"
#include "recognition/camera_calibration.h"
#include "velocity.h"
#include <std_msgs/Float32MultiArray.h>
#include "recognition/world_to_camera.h"
#include "calibration.h"
#include "log/logger.h"

void test()
{
    int arr[31] = {1, 10, 11, 12, 13, 14, 15, 16, 17,18, 19, 2 ,20 ,21 ,22 ,23 ,24, 25 ,26 ,27 ,28 ,29, 3 ,30, 31, 4 ,5 ,6, 7, 8 ,9};
    std::vector<int> map;
    for(int i = 0; i < 31; i++)
    {
        map.push_back(arr[i]);
    }
    //创建检测器
    Ten::Ten_yolo_cls detector("/home/maple/study2/merge_ws11/src/merge/model/yolo11-cls_gazebo/best", map);

    
    //加载图片
    cv::Mat img = cv::imread("/home/maple/study2/merge_ws11/src/merge/image/1.png");
    

    if (img.empty()) {
        std::cout << "无法加载图像！" << std::endl;
        return;
    }

    std::cout << "成功加载" << std::endl;
    std::cout << "图片尺寸: " << img.cols << "x" << img.rows << std::endl;


    
    //调用worker函数
    std::vector<Ten::Detection> results = detector.worker(img);

    
    std::cout << 0 << std::endl;

    
    //输出结果
    if (results.empty()) {
        std::cout << "没检测到目标" << std::endl;
    } 
    else {
        std::cout << "检测到 " << results.size() << " 个目标：" << std::endl;
        Ten::Detection det;
        for (int i = 0; i < results.size(); i++) {
            det = results[i];
            std::cout << "类别" << det.cls_id_ << ": 置信度" << det.conf_ 
                      << ", 位置(" << det.cx_ << "," << det.cy_ 
                      << "), 尺寸" << det.w_ << "x" << det.h_ << std::endl;
        }
    }
}

void test2()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws3/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    while (ros::ok()) { usleep(10000); }

}

void test3()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws7/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(1);
    pool.enqueue(laserMapping);

    while (ros::ok()) { usleep(10000); }

}

void test4()
{
    Ten::Ten_serial::GetInstance("/dev/ttyACM0", 115200);
    //Ten::Ten_serial::GetInstance();
    float arr[9] = {1,2,3,4,5,6,7,8,9};
    ros::Rate sl(100);
    while(ros::ok())
    {
        Ten::Ten_serial::GetInstance().serial_send(arr, 1, sizeof(arr));
        std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;
        sl.sleep();
    }
    
}

void test5()
{
    //Ten::Ten_serial::GetInstance("/dev/ttyUSB0");
    Ten::Ten_serial::GetInstance();
    float arr[10] = {0};
    ros::Rate sl(1);
    uint8_t frameid = 0;
    uint8_t length  = 0;
    while(ros::ok())
    {
        bool ok = Ten::Ten_serial::GetInstance().serial_read(arr, frameid, length);
        if(!ok)
        {
            std::cout<<"no receive! "<<std::endl;
            sl.sleep();
            continue;
        }
        std::cout<<"frameid: "<<(int)frameid<<std::endl;
        std::cout<<"length: "<< (int)length <<std::endl;
        for(int i = 0; i < length; i++)
        {
            std::cout << arr[i] << " ";
        } 
        std::cout<<std::endl;
        sl.sleep();
    }
    
}

void test6(std::string s = "camera")
{
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();

    while(ros::ok())
    {
        cv::Mat* img = camera.camera_read(1);
        // cv::imshow(s.c_str(), *img);
        // cv::waitKey(30);
        delete img;
    }
    

}

void test7()
{
    urcu_memb_register_thread();
    ros::Rate sl(100);
    while(ros::ok())
    {
        Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data());
        std::cout<< "xyz: "<<tf._xyz._x << " " << tf._xyz._y << " " << tf._xyz._z << std::endl;
        std::cout<< "rpy: "<<tf._rpy._roll << " " << tf._rpy._pitch << " " << tf._rpy._yaw << std::endl;
        sl.sleep();
    }
    urcu_memb_unregister_thread();

}

void test8()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws7/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(2);
    pool.enqueue(laserMapping);
    pool.enqueue(test7);
    while (ros::ok()) { usleep(10000); }
}

void test9()
{
    Ten::ThreadPool pool(3);
    pool.enqueue(test6, "camera1");
    pool.enqueue(test6, "camera2");
    pool.enqueue(test6, "camera3");
}

void test10()
{
    Eigen::Matrix3d R = Ten::createRotationMatrix(- M_PI / 2.0, - M_PI / 2.0, 0);
    std::cout<< R << std::endl;
}

void test11()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws7/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    while (ros::ok()) { usleep(10000); }
}

void test12()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws8/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(1);
    pool.enqueue(test7);
    laserMapping();
    //while (ros::ok()) { usleep(10000); }
    Ten::Ten_lidar::GetInstance().~Ten_lidar();
}


void test_yolo()
{

}


void test_han()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws8/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(1);
    pool.enqueue(test_yolo);
    laserMapping();
    Ten::Ten_lidar::GetInstance().~Ten_lidar();

}

void test_serial()
{
    Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws8/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(1);
    pool.enqueue(test7);
    laserMapping();
    Ten::Ten_lidar::GetInstance().~Ten_lidar();
}

void test_change()
{
    Eigen::Matrix3d R;
    R << -0.0947496,  -0.9955,  -0.00140255,
        -0.0426586,  0.00546773,  -0.999075,
        0.994587,  -0.0946021,  -0.0429847;
    cv::Mat rvec = Ten::RotationMatrixtorvec(R);
    std::cout<< "rvec: " << rvec << std::endl;
}



void test_yolo2()
{
    //创建检测器
    ros::NodeHandle nh("~");
    //Ten::Ten_yolo detector("/home/maple/study2/merge_ws10/src/merge/model/houli_1023_v5s/best", "cpu", 1);
    Ten::Ten_yolo detector("/home/maple/study2/merge_ws9/src/merge/model/yolo11-cls_gazebo/best", "cpu");
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    ros::Publisher debug_pub_ = nh.advertise<sensor_msgs::Image>("/yolo/debug_image", 2);

    ros::Rate sl(30);
    while(ros::ok())
    {
        cv::Mat img = camera.camera_read();
        //调用worker函数
        std::vector<Ten::Detection> results = detector.worker(img);
        // for(int i = 0; i < results.size(); i++)
        // {
        //     Ten::Detection best = results[i];
        //     float x1 = best.cx_ - best.w_ / 2;
        //     float x2 = best.cx_ + best.w_ / 2;
        //     float y1 = best.cy_ - best.h_ / 2;
        //     float y2 = best.cy_ + best.h_ / 2;
        //     cv::rectangle(img, 
        //     cv::Point(x1, y1),
        //     cv::Point(x2, y2),
        //     cv::Scalar(0,255,0), 2);
        //     cv::putText(img, "cls: " + std::to_string(best.cls_id_) + "cof: " + std::to_string(best.conf_), cv::Point(best.cx_, best.cy_), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255), 2);
        // }
        if(results.size() <= 0) continue;;
        Ten::Detection best = results[0];
        cv::putText(img, "cls: " + std::to_string(best.cls_id_) + "cof: " + std::to_string(best.conf_), cv::Point(1920/2,1080/2), cv::FONT_HERSHEY_SIMPLEX, 1.5, cv::Scalar(0,0,255), 2);
        std::cout<< "results.size(): " << results.size() << std::endl;

        sensor_msgs::ImagePtr debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
        debug_pub_.publish(debug_msg);
        sl.sleep();
    }
    camera.~Ten_camera();
    std::cout<< "camera.~Ten_camera();" << std::endl;
}




void test_relocation()
{
    std::string global_pcd_path = "/home/maple/study2/mapping/map.pcd";
    std::string local_pcd_path = "/home/maple/study2/mapping/local.pcd";
    Ten::Ten_relocation<pcl::PointXYZI> rel("/home/maple/study2/mapping/map.pcd");
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

}  

// void test_camera_cal()
// {
//     std::cout<< Ten::_D435_CAMERAINFO_.K() << std::endl;
// }



void test_transform()
{
    urcu_memb_register_thread();
    //Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    ros::NodeHandle nh("~");
    ros::Publisher float_array_pub = nh.advertise<std_msgs::Float32MultiArray>("/float_one_d_array", 10);
    std::vector<float> my_float_array;
    my_float_array.resize(9);

    float arr[9] = {0};

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = 0;
    xyzrpy_error._xyz._y = 0;
    xyzrpy_error._xyz._z = 0;
    xyzrpy_error._rpy._roll = 0;
    xyzrpy_error._rpy._pitch = 0;
    xyzrpy_error._rpy._yaw = 0;
    Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    // Ten::XYZRPY xyzrpy_car;
    // xyzrpy_car._xyz._x = -0.40944;
    // xyzrpy_car._xyz._y = 0.40944;
    // xyzrpy_car._xyz._z = 0;
    // xyzrpy_car._rpy._roll = 0;
    // xyzrpy_car._rpy._pitch = 0;
    // xyzrpy_car._rpy._yaw = -M_PI / 2.0;
    // Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 
    Ten::XYZRPY xyzrpy_car;
    xyzrpy_car._xyz._x = -0.28;
    xyzrpy_car._xyz._y = 0;
    xyzrpy_car._xyz._z = 0;
    xyzrpy_car._rpy._roll = 0;
    xyzrpy_car._rpy._pitch = 0;
    xyzrpy_car._rpy._yaw = 0;
    Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 
    
    
    Ten::_VELOCITY_TRANSFORMATION_.set_RT(xyzrpy_car);

    ros::Rate sl(10);
    while(ros::ok())
    {
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();

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

        for(int i = 0; i < 9; i++)
        {
            my_float_array[i] = arr[i];
        }
        std_msgs::Float32MultiArray float_array_msg;
        float_array_msg.data = my_float_array;
        float_array_pub.publish(float_array_msg);

        sl.sleep();
    }
    urcu_memb_unregister_thread();
}

void test_save()
{
    urcu_memb_register_thread();
    Ten::Ten_logger::GetInstance();
    Ten::Ten_logger::GetInstance().record_odometry(Ten::_TF_GET_.read_data());
    urcu_memb_unregister_thread();
}



#endif

