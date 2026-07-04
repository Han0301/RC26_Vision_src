#ifndef __CONTROL_CPP_
#define __CONTROL_CPP_




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

// void test_lidar()
// {
//     urcu_memb_register_thread();
//     float arr[9] = {0};

//     Ten::XYZRPY xyzrpy_error;
//     xyzrpy_error._xyz._x = 0;
//     xyzrpy_error._xyz._y = 0;
//     xyzrpy_error._xyz._z = 0;
//     xyzrpy_error._rpy._roll = 0;
//     xyzrpy_error._rpy._pitch = 0;
//     xyzrpy_error._rpy._yaw = 0;
//     Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

//     Ten::XYZRPY xyzrpy_car;
//     xyzrpy_car._xyz._x = -0.40944; //-0.40944
//     xyzrpy_car._xyz._y = 0.40944 + 0.088 /2;  //0.40944
//     xyzrpy_car._xyz._z = 0;
//     xyzrpy_car._rpy._roll = 0;
//     xyzrpy_car._rpy._pitch = 0;
//     xyzrpy_car._rpy._yaw = -M_PI / 2.0;
//     //Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 
    
    
//     Ten::_VELOCITY_TRANSFORMATION_.set_RT(xyzrpy_car);
//     //nav_msgs::Odometry odo_n;
//     ros::Rate sl(1);
//     while(Ten::_TREADPOOL_FLAG_.read_flag())
//     {
//         //位置变化
//         nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
//         Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
//         Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
//         Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();
//         //速度变化
//         Ten::XYZRPY lidar_LA;
//         lidar_LA._xyz._x = odo.twist.twist.linear.x;
//         lidar_LA._xyz._y = odo.twist.twist.linear.y;
//         lidar_LA._xyz._z = odo.twist.twist.linear.z;
//         lidar_LA._rpy._roll = odo.twist.twist.angular.x;
//         lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
//         lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
//         Ten::_VELOCITY_TRANSFORMATION_.set_lidar(lidar_LA);
//         Ten::XYZRPY car_LA = Ten::_VELOCITY_TRANSFORMATION_.getvelocity();

//         float roll = result._rpy._roll;
//         float pitch = result._rpy._pitch;
//         float yaw = result._rpy._yaw;

//         arr[0] = result._xyz._x;
//         arr[1] = result._xyz._y;
//         arr[2] = result._xyz._z;
//         std::cout<<"x: "<< result._xyz._x <<std::endl;
//         std::cout<<"y: "<< result._xyz._y <<std::endl;
//         std::cout<<"z: "<< result._xyz._z <<std::endl;
//         std::cout<<"roll: "<< result._rpy._roll <<std::endl;
//         std::cout<<"pitch: "<< result._rpy._pitch <<std::endl;
//         std::cout<<"yaw: "<< result._rpy._yaw <<std::endl;

//         arr[3] = roll * 180.0 / M_PI;
//         arr[4] = pitch * 180.0 / M_PI;
//         arr[5] = yaw * 180.0 / M_PI;

//         arr[6] = car_LA._xyz._x;
//         arr[7] = car_LA._xyz._y;
//         arr[8] = car_LA._xyz._z;

//         //std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;



//         // odo_n.twist.twist.linear.x = result._xyz._x;
//         // odo_n.twist.twist.linear.y = result._xyz._y;
//         // odo_n.twist.twist.linear.z = result._xyz._z;

//         // odo_n.twist.twist.linear.x = car_LA._xyz._x;
//         // odo_n.twist.twist.linear.y = car_LA._xyz._y;
//         // odo_n.twist.twist.linear.z = car_LA._xyz._z;

//         //Ten::Ten_logger::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/log").record_odometry(odo_n);

//         sl.sleep();
//     }
    
//     urcu_memb_unregister_thread();
// }


void serial_send_lidarR1()
{
    urcu_memb_register_thread();
    Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    float arr[9] = {0};

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = 0;
    xyzrpy_error._xyz._y = 0;
    xyzrpy_error._xyz._z = 0;
    xyzrpy_error._rpy._roll = 0;
    xyzrpy_error._rpy._pitch = 0;
    xyzrpy_error._rpy._yaw = 0;
    Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    Ten::XYZRPY xyzrpy_car;
    xyzrpy_car._xyz._x = -0.40944; //-0.40944
    xyzrpy_car._xyz._y = 0.40944 + 0.088 /2;  //0.40944
    xyzrpy_car._xyz._z = 0;
    xyzrpy_car._rpy._roll = 0;
    xyzrpy_car._rpy._pitch = 0;
    xyzrpy_car._rpy._yaw = -M_PI / 2.0;
    Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 
    
    
    Ten::_VELOCITY_TRANSFORMATION_.set_RT(xyzrpy_car);
    //nav_msgs::Odometry odo_n;
    ros::Rate sl(100);
    //定义滤波器
    Ten::XYZRPYFilter coordinate_ft;
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        //位置变化
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result_last = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();
        //滤波
        Ten::XYZRPY result = coordinate_ft(result_last);
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

        serial.serial_send(arr, 1, sizeof(arr));
        //std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;



        // odo_n.twist.twist.linear.x = result._xyz._x;
        // odo_n.twist.twist.linear.y = result._xyz._y;
        // odo_n.twist.twist.linear.z = result._xyz._z;

        // odo_n.twist.twist.linear.x = car_LA._xyz._x;
        // odo_n.twist.twist.linear.y = car_LA._xyz._y;
        // odo_n.twist.twist.linear.z = car_LA._xyz._z;

        //Ten::Ten_logger::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/log").record_odometry(odo_n);

        sl.sleep();
    }
    
    urcu_memb_unregister_thread();
}

void serial_send_lidarR2()
{
    urcu_memb_register_thread();
    Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    float arr[4] = {0};
    size_t size = sizeof(arr);
    size_t num = size / sizeof(float);

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = 0;
    xyzrpy_error._xyz._y = 0;
    xyzrpy_error._xyz._z = 0;
    xyzrpy_error._rpy._roll = 0;
    xyzrpy_error._rpy._pitch = 0;
    xyzrpy_error._rpy._yaw = 0;
    Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    Ten::XYZRPY xyzrpy_car;
    xyzrpy_car._xyz._x = 0;
    xyzrpy_car._xyz._y = 0.28;
    xyzrpy_car._xyz._z = 0;
    xyzrpy_car._rpy._roll = 0;
    xyzrpy_car._rpy._pitch = 0;
    xyzrpy_car._rpy._yaw = 0;
    Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 
    
    //nav_msgs::Odometry odo_n;
    ros::Rate sl(100);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();


        float roll = result._rpy._roll;
        float pitch = result._rpy._pitch;
        float yaw = result._rpy._yaw;

        arr[0] = result._xyz._x;
        arr[1] = result._xyz._y;
        arr[2] = result._xyz._z;
        arr[3] = yaw;

        int flag = 0;
        for(size_t i = 0; i < num; i++)
        {
            if(std::isnan(arr[i])) 
            {
                flag = 1;
                break;
            }
        }

        if(flag)
        {
            continue;
        }
        

        serial.serial_send(arr, 1, size);
        //std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;



        // odo_n.twist.twist.linear.x = result._xyz._x;
        // odo_n.twist.twist.linear.y = result._xyz._y;
        // odo_n.twist.twist.linear.z = result._xyz._z;

        // odo_n.twist.twist.linear.x = car_LA._xyz._x;
        // odo_n.twist.twist.linear.y = car_LA._xyz._y;
        // odo_n.twist.twist.linear.z = car_LA._xyz._z;

        //Ten::Ten_logger::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/log").record_odometry(odo_n);

        sl.sleep();
    }
    
    urcu_memb_unregister_thread();
}

void calibration()
{
    std::string log_path = std::string(ROOT_DIR) + std::string("map/map.pcd");
    Ten::Ten_relocation<pcl::PointXYZI> rel(log_path);
    Ten::XYZRPY xyzrpy = rel.get_transformation();

    std::cout << "---------------------------" << std::endl; 
    std::cout << "x: " << xyzrpy._xyz._x << std::endl;
    std::cout << "y: " << xyzrpy._xyz._y << std::endl;
    std::cout << "z: " << xyzrpy._xyz._z << std::endl;
    std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
    std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
    std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;


    Ten::_COORDINATE_TRANSFORMATION_.set_world2toworld1(xyzrpy);
}

void serial_receiver()
{
    urcu_memb_register_thread();
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
    Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    uint8_t arr[1000] = {0};
    ros::Rate sl(10);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        uint8_t frame_id = 0;
        uint8_t length = 0;
        if(serial.serial_read(arr, frame_id, length))
        {
            if(frame_id == 4) //重定位
            {
                calibration();
                log.record_map(Ten::_Map_GET_.read_data());
                serial.clearBuffer(1);
            }
            else if(frame_id == 5) //位置有问题
            {
                log.record_odometry(Ten::_TF_GET_.read_data());
                serial.clearBuffer(1);
            }
        }
        sl.sleep();
    }
    urcu_memb_unregister_thread();
}


// 核心函数：从指定路径的文件读取第一行数字串，按位存入vector<int>引用
// 入参：file_path - 文件绝对/相对路径；num_vec - 存储结果的vector引用（会被修改）
// 返回：bool - true=读取成功，false=读取失败（文件不存在/打开失败/空文件等）
bool readFirstLineDigitsToVec(const std::string& file_path, std::vector<int>& num_vec) {
    // 第一步：清空vector，避免外部传入的容器有旧数据残留（关键！）
    num_vec.clear();

    // 第二步：打开文件（只读模式）
    std::ifstream in_file(file_path);
    // 校验文件是否成功打开
    if (!in_file.is_open()) {
        std::cerr << "错误：无法打开文件 -> " << file_path << std::endl;
        return false;
    }

    // 第三步：读取文件第一行内容
    std::string first_line;
    if (!std::getline(in_file, first_line)) {  // 校验是否读取到行（空文件会触发此错误）
        std::cerr << "错误：文件为空，或读取第一行失败 -> " << file_path << std::endl;
        in_file.close();  // 失败也要关闭文件，释放资源
        return false;
    }

    // 第四步：遍历第一行字符，拆解数字存入vector
    for (char c : first_line) {
        if (std::isdigit(c)) {  // 只处理数字字符，自动过滤空格/符号/换行等
            int num = c - '0';  // 字符转整型数字（核心）
            num_vec.push_back(num);
            //std::cout << num << std::endl;
        }
    }

    // 第五步：关闭文件，释放资源
    in_file.close();

    // 额外校验：如果第一行无任何数字，也视为「读取成功但无数据」（非错误，仅提示）
    if (num_vec.empty()) {
        std::cout << "提示：文件第一行无有效数字 -> " << file_path << std::endl;
    }

    if(num_vec.size() != 12)
    {
        std::cout<< "num_vec.size() != 12" << std::endl;
        return false;
    }
    return true;  // 所有步骤执行完成，返回成功
}






void serial_send_map_and_path_R2()
{
    urcu_memb_register_thread();
    //Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    
    std::vector<int> map;
    std::string map_path = string(ROOT_DIR) + string("path/map.txt");
    std::cout<< map_path << std::endl;
    if(!readFirstLineDigitsToVec(map_path, map))
    {
        std::cout<< "❌ can not read the path" << std::endl;;
        return;
    }

    std::string path_path = string(ROOT_DIR) + string("path/placements_and_paths_blue_0.2m_simplied_sorted.txt");
    std::vector<int> path;
    Ten::getpath(path_path, map, path);

    ros::Rate sl(10);
    int total_num = 0;
    while(Ten::_TREADPOOL_FLAG_.read_flag() && total_num < 10)
    {
        uint8_t msgs_map[map.size()] = {0};
        std::cout<< " : ";
        for (size_t i = 0; i < map.size(); ++i) 
        {
            msgs_map[i] = (uint8_t)map[i];
            std::cout<<(int)msgs_map[i];
        }
        std::cout<<std::endl;
        //serial.serial_send(msgs_map, 2, map.size());

        uint8_t msgs_path[path.size()] = {0};
        std::cout<< " : ";
        for (size_t i = 0; i < path.size(); ++i) 
        {
            msgs_path[i] = (uint8_t)path[i];
            std::cout<<(int)msgs_path[i] << " ";
        }
        std::cout<<std::endl;
        //serial.serial_send(msgs_path, 3, path.size());
        sl.sleep();
        total_num++;
    }
    
    urcu_memb_unregister_thread();
}



void calibration2()
{
    std::string log_path = std::string(ROOT_DIR) + std::string("map/map.pcd");
    //std::string log_path = std::string("/home/maple/study2/maple/map/map.pcd");
    Ten::Ten_relocation<pcl::PointXYZI> rel(log_path);
    Ten::XYZRPY xyzrpy = rel.get_transformation();

    std::cout << "---------------------------" << std::endl; 
    std::cout << "x: " << xyzrpy._xyz._x << std::endl;
    std::cout << "y: " << xyzrpy._xyz._y << std::endl;
    std::cout << "z: " << xyzrpy._xyz._z << std::endl;
    std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
    std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
    std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = 0.025;
    xyzrpy_error._xyz._y = -0.045;
    xyzrpy_error._xyz._z = 0.10;
    xyzrpy_error._rpy._roll = 0;
    xyzrpy_error._rpy._pitch = 0;
    xyzrpy_error._rpy._yaw = 0;

    Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::worldtocurrent(xyzrpy._xyz, xyzrpy._rpy) * Ten::worldtocurrent(xyzrpy_error._xyz, xyzrpy_error._rpy) * Ten::_COORDINATE_TRANSFORMATION_.get_lidartocar());

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
            calibration2();
        }
        else if(flag == 0)
        {
            break;
        }
    }
}


void test_lidar()
{
    urcu_memb_register_thread();
    float arr[9] = {0};

    Ten::XYZRPY xyzrpy_error;
    xyzrpy_error._xyz._x = 0;
    xyzrpy_error._xyz._y = 0;
    xyzrpy_error._xyz._z = 0;
    xyzrpy_error._rpy._roll = 0;
    xyzrpy_error._rpy._pitch = 0;
    xyzrpy_error._rpy._yaw = 0;
    //Ten::_COORDINATE_TRANSFORMATION_.set_stead_state_error(xyzrpy_error);

    Ten::XYZRPY xyzrpy_car;
    xyzrpy_car._xyz._x = -0.40944;            //-0.40944
    xyzrpy_car._xyz._y = 0.40944 + 0.088 / 2; // 0.40944
    xyzrpy_car._xyz._z = 0;
    xyzrpy_car._rpy._roll = 0;
    xyzrpy_car._rpy._pitch = 0;
    xyzrpy_car._rpy._yaw = -M_PI / 2.0;
    //Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car);

    //Ten::_VELOCITY_TRANSFORMATION_.set_RT(xyzrpy_car);
    // nav_msgs::Odometry odo_n;
    ros::Rate sl(1);
    Ten::XYZRPYFilter coordinate_ft;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        // 位置变化
        nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
        Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
        Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
        Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY();
        //Ten::XYZRPY result = coordinate_ft(result_last);
        // 速度变化
        // Ten::XYZRPY lidar_LA;
        // lidar_LA._xyz._x = odo.twist.twist.linear.x;
        // lidar_LA._xyz._y = odo.twist.twist.linear.y;
        // lidar_LA._xyz._z = odo.twist.twist.linear.z;
        // lidar_LA._rpy._roll = odo.twist.twist.angular.x;
        // lidar_LA._rpy._pitch = odo.twist.twist.angular.y;
        // lidar_LA._rpy._yaw = odo.twist.twist.angular.z;
        // Ten::_VELOCITY_TRANSFORMATION_.set_lidar(lidar_LA);
        // Ten::XYZRPY car_LA = Ten::_VELOCITY_TRANSFORMATION_.getvelocity();

        // float roll = result._rpy._roll;
        // float pitch = result._rpy._pitch;
        // float yaw = result._rpy._yaw;

        // arr[0] = result._xyz._x;
        // arr[1] = result._xyz._y;
        // arr[2] = result._xyz._z;

        std::cout << "x: " << result._xyz._x << std::endl;
        std::cout << "y: " << result._xyz._y << std::endl;
        std::cout << "z: " << result._xyz._z << std::endl;

        std::cout << "roll: " << result._rpy._roll << std::endl;
        std::cout << "pitch: " << result._rpy._pitch << std::endl;
        std::cout << "yaw: " << result._rpy._yaw << std::endl;

        std::cout << "--------------xyz----------" << std::endl;
        std::cout << result._xyz._x << " " << result._xyz._y << " " << result._xyz._z << std::endl;

        // arr[3] = roll * 180.0 / M_PI;
        // arr[4] = pitch * 180.0 / M_PI;
        // arr[5] = yaw * 180.0 / M_PI;

        // arr[6] = car_LA._xyz._x;
        // arr[7] = car_LA._xyz._y;
        // arr[8] = car_LA._xyz._z;



        // std::cout<<"sizeof(arr)"<<sizeof(arr)<<std::endl;

        // odo_n.twist.twist.linear.x = result._xyz._x;
        // odo_n.twist.twist.linear.y = result._xyz._y;
        // odo_n.twist.twist.linear.z = result._xyz._z;

        // odo_n.twist.twist.linear.x = car_LA._xyz._x;
        // odo_n.twist.twist.linear.y = car_LA._xyz._y;
        // odo_n.twist.twist.linear.z = car_LA._xyz._z;

        // Ten::Ten_logger::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/log").record_odometry(odo_n);

        sl.sleep();
    }

    urcu_memb_unregister_thread();
}


void serial_receiver2()
{
    urcu_memb_register_thread();
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
    Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    uint8_t arr[1000] = {0};
    ros::Rate sl(10);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        uint8_t frame_id = 0;
        uint8_t length = 0;
        if(serial.serial_read(arr, frame_id, length))
        {
            if(frame_id == 4) //重定位
            {
                calibration2();
                log.record_map(Ten::_Map_GET_.read_data());
                serial.clearBuffer(1);
            }
            else if(frame_id == 5) //位置有问题
            {
                log.record_odometry(Ten::_TF_GET_.read_data());
                serial.clearBuffer(1);
            }
        }
        sl.sleep();
    }
    urcu_memb_unregister_thread();
}

void serial_send_test1()
{
    urcu_memb_register_thread();
    Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    float arr[9] = {0,1,2,3,4,5,6,7,8};
    ros::Rate sl(1);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        if(serial.serial_send(arr, 1, sizeof(arr)))
        {
            std::cout << "ok" << std::endl;
        }
        else
        {
            std::cout << "false" << std::endl;
        }
        sl.sleep();
    }
    urcu_memb_unregister_thread();
}










#endif


