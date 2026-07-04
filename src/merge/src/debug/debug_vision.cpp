#include "./../rcekf/rcekf.h"
#include "./../serial.h"
#include "./../openvino.h"
#include "./../threadpool.h"
#include "./..//livox_ros_driver2/src/livox_ros_driver.h"
#include "./..//point_lio/src/laserMapping2.h"
#include "./../lidar.h"
#include "./../camera.h"
#include "./../usb_cam.h"
#include "./../usb_cam_multi.h"
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
#include "./../mapping/mapping.h"
#include "./../superstratum/controlR2.h"
#include "./../lidar/get_cloud.h"
#include "./../debug/ros_debug.h"
#include "./../selfinspection/inspection.h"
#include "./../nine/mouse.h"
#include "./../nine/segmentation.h"
#include "./../nine/detector.h"
#include <std_msgs/Int32.h>
#include <chrono>
#include <iostream>
#include <iomanip>
#include "./../camera_virtual.h"
#include "./../apriltag_detect_r1/apriltag_detector.cpp"

void test_mapping()
{

    ros::Rate sl(20);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        ros::spinOnce();
        sl.sleep();
    }

}

void test_vision_cam()
{
    Ten::Ten_usb_cam& usbcam = Ten::Ten_usb_cam::GetInstance(Ten::_usb_device_num1_,640,480,30);
    ros::Rate sl(60);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        cv::Mat img = usbcam.camera_read();
        if(img.empty())
        {
            sl.sleep();
            continue;
        }
        cv::imshow("debug", img);
        cv::waitKey(1000 / 60);
    }

}

void test_vision_cam_multi()
{
    std::vector<int> idxs = {Ten::_usb_device_num1_, Ten::_usb_device_num2_, Ten::_usb_device_num3_};
    Ten::Ten_usb_cam_multi& usbcam = Ten::Ten_usb_cam_multi::GetInstance(idxs,640,480,30);

    ros::Rate sl(60 * idxs.size());
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        for(int idx : idxs)
        {
            cv::Mat img = usbcam.camera_read(idx);
            if(img.empty())
            {
                sl.sleep();
                continue;
            }
            if(idx == Ten::_usb_device_num1_)
            {
                cv::imshow("debug1", img);
                cv::waitKey(1000 / 60 / idxs.size());
            }
            else if(idx == Ten::_usb_device_num2_)
            {
                cv::imshow("debug2", img);
                cv::waitKey(1000 / 60 / idxs.size());
            }
            else if(idx == Ten::_usb_device_num3_)
            {
                cv::imshow("debug3", img);
                cv::waitKey(1000 / 60 / idxs.size());
            }
            else
            {
                sl.sleep();
            }
            
        }
        
    }

}


void test_vision_d435()
{
    urcu_memb_register_thread();
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance(640,480,60);
    ros::Rate sl(60);
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        cv::Mat img = camera.camera_read();
        if(img.empty())
        {
            sl.sleep();
            continue;
        }
        cv::imshow("debug", img);
        cv::waitKey(1000 / 60);
    }
}


void vision_calibration()
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
    xyzrpy_error._xyz._x = Ten::superstratum::_r2_xyzrpy_error_xyz_x_;
    xyzrpy_error._xyz._y = Ten::superstratum::_r2_xyzrpy_error_xyz_y_;
    xyzrpy_error._xyz._z = Ten::superstratum::_r2_xyzrpy_error_xyz_z_;
    xyzrpy_error._rpy._roll = Ten::superstratum::_r2_xyzrpy_error_rpy_roll_;
    xyzrpy_error._rpy._pitch = Ten::superstratum::_r2_xyzrpy_error_rpy_pitch_;
    xyzrpy_error._rpy._yaw = Ten::superstratum::_r2_xyzrpy_error_rpy_yaw_;
    Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::XYZRPYtotransform_matrix(xyzrpy) * Ten::XYZRPYtotransform_matrix(xyzrpy_error) * Ten::_COORDINATE_TRANSFORMATION_.get_lidartocar());
    Ten::_COORDINATE_TRANSFORMATION_.set_world2toworld1(world_origin);
}


void test_path()
{
    std::vector<int> path = Ten::readFileToAsciiVector(std::string(ROOT_DIR) + std::string("path/map.txt"));
    
    for(size_t i = 0; i < path.size(); i++)
    {
        std::cout << path[i] << " ";
    }
    std::cout << std::endl;
    
    for(size_t i = 0; i < Ten::_global_path_.size(); i++)
    {
        std::cout << Ten::_global_path_[i] << " ";
    }
    std::cout << std::endl;
}

void test_get_cloud()
{
    Ten::debug::DebugPointCloudPub& cloudpub = Ten::debug::DebugPointCloudPub::GetInstance();

    Ten::cloud::counting_stars cs;

    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        int input = 0;
        std::cin >> input;
        if(input == 1)
        {
            std::vector<bool> is = cs.get_exist();
            for(size_t i = 0; i < is.size(); i++)
            {
                std::cout << "i: " << i << " have_box: " << is[i] << std::endl;
            }
            cloudpub.publish<pcl::PointXYZI>(cs.get_filter_cloud(), "/nine/fliter_cloud");
            cs.clear_cloud();
        }
        else if(input == 2)
        {
            vision_calibration();
        }
    }

}


/**
 * @brief 发布 std_msgs::Int32 消息
 * @param data 输入：需要发布的 int 类型数据
 */
void response(int data)
{
    static ros::Publisher int_pub;
    if (!int_pub)
    {
        ros::NodeHandle nh;
        int_pub = nh.advertise<std_msgs::Int32>("/scripts/response", 3);
    }
    std_msgs::Int32 int_msg;
    int_msg.data = data;
    int_pub.publish(int_msg);
}

void lidarrestartCallback(const std_msgs::Int32::ConstPtr& msg)
{
    static int receive = 0;
    static int last = 0;

    receive = msg->data;
    if(receive != last)
    {
        last = receive;
        Ten::_LASERMAPPING_FLAG_.set_flag(true);
        std::cout << "Ten::_LASERMAPPING_FLAG_.set_flag(true);" << std::endl;
    }
    else
    {
        response(last);
    }
}

/**
 * @brief 读取txt中所有整数（空格/换行任意分隔），返回int数组
 * @param file_path： 参数：txt文件路径
 * @return 存储所有数字的vector<int>
 */
std::vector<int> readIntTxt(const std::string& file_path)
{
    std::vector<int> result;
    // 创建文件输入流
    std::ifstream fin(file_path);

    // 判断文件是否成功打开
    if (!fin.is_open())
    {
        std::cerr << "错误：无法打开文件 " << file_path << std::endl;
        return result;
    }

    int num;
    // while循环持续读取整数，cin/ifstream默认自动忽略空格、换行、制表符
    while (fin >> num)
    {
        result.push_back(num);
    }

    // 关闭文件流（离开作用域会自动析构，手动关闭更规范）
    fin.close();
    return result;
}

void parameterCallback(const std_msgs::Int32::ConstPtr& msg)
{
    static int receive = 0;
    static int last = 0;
    static uint8_t MFparameter[30];

    receive = msg->data;
    if(receive != last)
    {
        last = receive;
        std::vector<int> param = readIntTxt(std::string(ROOT_DIR) + std::string("path/parameter.txt"));
        uint8_t length = param.size();
        if(length)
        {
            std::cout << "------------------" << std::endl;
            for(size_t i = 0; i < param.size() && i < 30; i++)
            {
                MFparameter[i] = (uint8_t)param[i];
                Ten::_input_parameter_[i] = param[i];
                std::cout << " " << param[i];
            }
            std::cout << std::endl;
            std::cout << "------------------" << std::endl;
            response(last);
        }
    }
    else
    {
        response(last);
    }
}

void script_control()
{
    // 1. 创建当前线程私有独立回调队列，和其他线程完全隔离
    ros::CallbackQueue queue_script;
    // 2. 创建句柄并绑定私有队列，该nh下所有订阅消息都进这个私有队列
    ros::NodeHandle nh;
    nh.setCallbackQueue(&queue_script);

    ros::Subscriber restart_lidar = nh.subscribe("/scripts/restart_lidar", 3, lidarrestartCallback);
    ros::Subscriber parameter_path = nh.subscribe("/scripts/send_param", 3, parameterCallback);

    // 循环控频100Hz
    ros::Rate loop_rate(100);
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 只处理当前线程私有队列，不处理全局其他话题回调
        // 超时0.01s，无消息最多阻塞10ms就返回，避免卡死
        queue_script.callAvailable(ros::WallDuration(0.01));
        loop_rate.sleep();
    }
}


void selfinspection()
{
    Ten::inspection in;
    in.cameraside();
}

void nine()
{
    Ten::mouse m;
    Ten::segmentation s;
    Ten::detector d;
    m.calibration();
    Ten::Ten_usb_cam& usbcam = Ten::Ten_usb_cam::GetInstance(Ten::_usb_device_num3_,640,480,30);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        cv::Mat img = usbcam.camera_read();
        if(img.empty())
        {
            cv::waitKey(1000 / 30);
            continue;
        }
        std::vector<cv::Mat> grids = s.getroi(img);
        cv::Mat roi_debug = s.debugAssemble3x3Grid(grids);
        cv::imshow("global", img);
        cv::imshow("roi_debug", roi_debug);
        int key = cv::waitKey(1000 / 30);
        if(key == 'q')
        {
            break;
        }
        else if(key == 's')
        {
            d.saveimg(grids);
        }
    }

}



// 返回秒级时间戳，精度高于1ms
double get_timestamp_s() {
    // 获取当前系统时间点
    auto now = std::chrono::system_clock::now();
    // 转换为以秒为单位的浮点时长
    std::chrono::duration<double> duration_s = now.time_since_epoch();
    return duration_s.count();
}



void test_serial()
{
    //Ten::Ten_serial::GetInstance("/dev/ttyGS0", 115200);
    Ten::Ten_serial& serials = Ten::Ten_serial::GetInstance();
    double arr[1] = {1};
    ros::Rate sl(200);
    double last = 0;
    double now = 0;
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        last = now;
        now = get_timestamp_s();
        arr[0] = now;
        //std::cout << "time: " << now << std::endl;
        // std::cout << "dt: " << now - last << std::endl;
        serials.serial_send(arr, 1, sizeof(arr));
        sl.sleep();
    }
    
}

void test_receiver()
{
    urcu_memb_register_thread();
    Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        double arr[10] = {0};
        uint8_t frame_id = 0;
        uint8_t length = 0;
        if(serial.serial_read2(arr, frame_id, length))
        {
            std::cout << "id: " << (int)frame_id << std::endl;
            std::cout << "length: " << (int)length << std::endl;
            double now = get_timestamp_s();
            //std::cout << "dt: " << now - arr[0] << std::endl;
            printf("dt = %.15lf\n", now - arr[0]);
        }
        usleep(1000);
    }
    urcu_memb_unregister_thread();
}


void test_camera_virtual(Ten::camera_virtual& ccc)
{
    for(int i = 0; i < 30; i++)
    {
        cv::Mat img = ccc.camera_read();
        cv::imshow("debug", img);
        cv::waitKey(30);
    }
}

void test_camerahhh()
{
    std::vector<int> idxs = {Ten::_usb_device_num1_};
    Ten::Ten_usb_cam_multi& usbcam = Ten::Ten_usb_cam_multi::GetInstance(idxs,640,480,30);
    usbcam.set_idx(Ten::_usb_device_num1_);
    test_camera_virtual(usbcam);
    Ten::Ten_camera& deppcamera = Ten::Ten_camera::GetInstance();
    test_camera_virtual(deppcamera);
}

void test_camera_detR1()
{
    Ten::Ten_camera& deppcamera = Ten::Ten_camera::GetInstance();
    deppcamera.reset_camera(640,480,60);
    std::cout << "is_r1: " << Ten::apriltag_detect::is_r1_area(deppcamera) << std::endl;
    std::vector<int> idxs = {Ten::_usb_device_num2_};
    Ten::Ten_usb_cam_multi& usbcam = Ten::Ten_usb_cam_multi::GetInstance(idxs,640,480,60);
    usbcam.set_idx(Ten::_usb_device_num2_);
    std::cout << "is_r1: " << Ten::apriltag_detect::is_r1_area(usbcam) << std::endl;

}