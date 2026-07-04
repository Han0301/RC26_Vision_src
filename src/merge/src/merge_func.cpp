#ifndef __MERGE_FUNC_CPP_
#define __MERGE_FUNC_CPP_




#include "serial.h"
#include "openvino.h"
#include "threadpool.h"
#include "./livox_ros_driver2/src/livox_ros_driver.h"
#include "./point_lio/src/laserMapping2.h"
#include "lidar.h"
#include "camera.h"
#include "test.cpp"
#include "test2.cpp"
#include "recognition/zbuffer_simplify.h"
#include "relocation.h"
#include "coordinate.h"
#include "recognition/camera_calibration.h"
#include "recognition/world_to_camera.h"
#include "velocity.h"
#include "calibration.h"
#include "log/logger.h"

// void yolo()
// {
//     urcu_memb_register_thread();
//     ros::NodeHandle nh("~");
//     image_transport::ImageTransport it(nh);
//     image_transport::Publisher debug_roi_pub = it.advertise("pub_debug_roi",2);
//     image_transport::Publisher zbuffer_pub = it.advertise("pub_image_topic", 2);

//     Ten::Ten_zbuffer zbuffer_handler;
//     Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();


//     int arr[31] = {1, 10, 11, 12, 13, 14, 15, 16, 17,18, 19, 2 ,20 ,21 ,22 ,23 ,24, 25 ,26 ,27 ,28 ,29, 3 ,30, 31, 4 ,5 ,6, 7, 8 ,9};
//     std::vector<int> map;
//     for(int i = 0; i < 31; i++)
//     {
//         map.push_back(arr[i]);
//     }    
//     Ten::Ten_yolo_cls detector("/home/maple/study2/merge_ws11/src/merge/model/yolo11-cls_gazebo/best", map);
//     Ten::Ten_map map_s;

//     ros::Rate sl(10);
//     while(ros::ok())
//     {
//         // Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(odo);
//         // std::cout<< "xyz: "<<tf._xyz._x << " " << tf._xyz._y << " " << tf._xyz._z << std::endl;
//         // std::cout<< "rpy: "<<tf._rpy._roll << " " << tf._rpy._pitch << " " << tf._rpy._yaw << std::endl;

//         cv::Mat img = camera.camera_read();
//         sl.sleep();
//         nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();

//         std::unordered_map<int, cv::Mat> best_roi_image_ = zbuffer_handler.manage_odom_zbuffer_roi(odo, img);
//         std::cout << "std::unordered_map<int, cv::Mat> best_roi_image_.size(): " << best_roi_image_.size() << std::endl;
//         for (auto& x: best_roi_image_)
//         {
//             std::vector<Ten::Detection> results = detector.worker(x.second);
//             if(results.empty())
//             {
//                 continue;;
//             }
//             std::cout<< "x.first: " << x.first << "results[0].conf_: " << results[0].conf_ << std::endl;
//             if(map_s.object_confidence_[x.first - 1] < results[0].conf_)
//             {
//                 map_s.object_[x.first - 1] = results[0].cls_id_;
//                 map_s.object_confidence_[x.first - 1] = results[0].conf_;
//                 std::cout<< "map.object_[x.first]" << map_s.object_[x.first - 1] <<std::endl;
//             }
//         }
//         cv::Mat debug = Ten::roi_best_zbuffer_debug2(best_roi_image_, map_s.object_);
//         sensor_msgs::ImagePtr pub_debug_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug).toImageMsg();
//         debug_roi_pub.publish(pub_debug_msg);
//         // sensor_msgs::ImagePtr pub_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();
//         // zbuffer_pub.publish(pub_msg);

        
//     }

//     camera.~Ten_camera();
//     urcu_memb_unregister_thread();

// }




void serial_send_lidar()
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
    xyzrpy_car._xyz._x = -0.40944;
    xyzrpy_car._xyz._y = 0.40944;
    xyzrpy_car._xyz._z = 0;
    xyzrpy_car._rpy._roll = 0;
    xyzrpy_car._rpy._pitch = 0;
    xyzrpy_car._rpy._yaw = -M_PI / 2.0;
    Ten::_COORDINATE_TRANSFORMATION_.set_lidartocar(xyzrpy_car); 
    
    
    Ten::_VELOCITY_TRANSFORMATION_.set_RT(xyzrpy_car);
    nav_msgs::Odometry odo_n;
    ros::Rate sl(100);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
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

void calibration()
{
    Ten::Ten_relocation<pcl::PointXYZI> rel("/home/rc/RC_2026/mapping/map.pcd");

    std::cout << "Ten::Ten_relocation<pcl::PointXYZI> rel(//home/rc/RC_2026/mapping/map.pcd);" << std::endl; 

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
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/log");
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



void zbuffer()
{

    urcu_memb_register_thread();
    //初始化ros调试
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher debug_roi_pub = it.advertise("pub_debug_roi",2);
    image_transport::Publisher pub_img = it.advertise("pub_image_topic", 2);
    //初始化相机
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance("/home/h/RC2026/merge_ws21/src/merge/log");
    //初始化yolo11-cls
    int arr[31] = {1, 10, 11, 12, 13, 14, 15, 16, 17,18, 19, 2 ,20 ,21 ,22 ,23 ,24, 25 ,26 ,27 ,28 ,29, 3 ,30, 31, 4 ,5 ,6, 7, 8 ,9};
    std::vector<int> map;
    for(int i = 0; i < 31; i++)
    {
        map.push_back(arr[i]);
    }    
    Ten::Ten_yolo_cls detector("/home/h/package/hou_li_11cls_1224/best", map);
    Ten::Ten_map map_s;
    sleep(5);
    //打印sleep()
    std::cout<< "----------sleep(10)------------" << std::endl;

    //设置相机内外参
    cv::Mat K = (cv::Mat_<double>(3,3) <<
    1380.4350, 0, 974.0183,
    0,  1385.0788, 541.4301,
    0, 0, 1);
    Eigen::Matrix4d transform_matrix;
    transform_matrix << 
    -0.0520106,  -0.998646,  0.000310088,  0.0206641,  
    -0.0471048,  0.00214311,  -0.998888,  0.396765,  
    0.997535,  -0.0519674,  -0.0471525,  0.50734,  
    0.0         ,  0.0        ,  0.0        ,  1.0;       
    Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_K(K);
    Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_Extrinsic_Matrix(transform_matrix);

    //打印内参
    std::cout<< "revc: " << Ten::_CAMERA_TRANSFORMATION_.camerainfo_.revc() << std::endl;
    std::cout<< "tevc: " << Ten::_CAMERA_TRANSFORMATION_.camerainfo_.tevc() << std::endl;

    //重定位
    std::cout << "-----------------------是否重定位--------------------------"<<std::endl;
    //sleep(20);
    string input;
    std::cin >> input;
    if(input == "r")
    {
        Ten::Ten_relocation<pcl::PointXYZI> rel("/home/maple/study2/mapping/map.pcd");

        std::cout << "Ten::Ten_relocation<pcl::PointXYZI> rel(/home/maple/study2/mapping/map.pcd);" << std::endl; 
    
        Ten::XYZRPY xyzrpy = rel.get_transformation();
    
        std::cout << "---------------------------" << std::endl; 
        std::cout << "x: " << xyzrpy._xyz._x << std::endl;
        std::cout << "y: " << xyzrpy._xyz._y << std::endl;
        std::cout << "z: " << xyzrpy._xyz._z << std::endl;
        std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
        std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
        std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;
        //Ten::_COORDINATE_TRANSFORMATION_.set_world2toworld1(xyzrpy);
        Ten::_CAMERA_TRANSFORMATION_.set_world2toworld1(xyzrpy);
    }

    //设置世界到雷达
    //读取图片
    //第一次标定
    cv::Mat image = camera.camera_read();
    Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data());
    Ten::XYZRPY error;
    error._xyz._x = -0.0037;
    error._xyz._y = -0.0012;
    error._xyz._z = -0.0026;
    error._rpy._roll = -0.007;
    error._rpy._pitch = 0.01;
    error._rpy._yaw = 0.0003;
    //打印tf信息
    std::cout << "---------------------------" << std::endl; 
    std::cout << "x: " << tf._xyz._x << std::endl;
    std::cout << "y: " << tf._xyz._y << std::endl;
    std::cout << "z: " << tf._xyz._z << std::endl;
    std::cout << "roll: " << tf._rpy._roll << std::endl;
    std::cout << "pitch: " << tf._rpy._pitch << std::endl;
    std::cout << "yaw: " << tf._rpy._yaw << std::endl;

    Ten::_CAMERA_TRANSFORMATION_.set_worldtolidar(tf);
    Ten::_CAMERA_TRANSFORMATION_.set_error(error);

    Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(Ten::_INIT_3D_BOX_.pcl_LS_plum_object_points_, 
        Ten::_INIT_3D_BOX_.pcl_C_plum_object_points_, Ten::_INIT_3D_BOX_.object_plum_points);
    Ten::_INIT_3D_BOX_.split_2d_points();

    Ten::_ZBUFFER_SIMPLIFY_.set_box_lists_init(Ten::_INIT_3D_BOX_.C_object_points_, Ten::_INIT_3D_BOX_.C_plum_points_, 
        Ten::_INIT_3D_BOX_.object_2d_points, Ten::_INIT_3D_BOX_.plum_2d_points,
        Ten::_INIT_3D_BOX_.object_2d_points_, Ten::_INIT_3D_BOX_.plum_2d_points_);

    Ten::_ZBUFFER_SIMPLIFY_.set_box_lists_(image,  Ten::_INIT_3D_BOX_.object_2d_points_, 
        Ten::_INIT_3D_BOX_.plum_2d_points_ ,Ten::_INIT_3D_BOX_.box_lists_);

    //模型初步筛选
    int _debug_count_zbuffer_flag_z_size = 0;
    int _debug_count_zbuffer_flag_o_size = 0;
    std::vector<Ten::box>& box_lists = Ten::_INIT_3D_BOX_.box_lists_;
    for(int i = 0; i < box_lists.size(); i++)
    {
        if(box_lists[i].zbuffer_flag == 1)
        {
            _debug_count_zbuffer_flag_o_size++;
            std::vector<Ten::Detection> result = detector.worker(box_lists[i].roi_image);
            if(result[0].conf_ <= 0.8)
            {
                _debug_count_zbuffer_flag_z_size++;
                box_lists[i].zbuffer_flag = 0;
            }
            if(map_s.object_confidence_[box_lists[i].idx] < result[0].conf_)
            {
                map_s.object_[box_lists[i].idx] = result[0].cls_id_;
                map_s.object_confidence_[box_lists[i].idx] = result[0].conf_;
                box_lists[i].cls = result[0].cls_id_;
                box_lists[i].confidence = result[0].conf_;
                std::cout<< "update where" << box_lists[i].idx <<std::endl;
            }
        }
    }
    std::cout<< "_debug_count_zbuffer_flag_z_size: "<< _debug_count_zbuffer_flag_z_size << std::endl;
    std::cout<< "_debug_count_zbuffer_flag_o_size: "<< _debug_count_zbuffer_flag_o_size << std::endl;
    //用比较好的图像初始化standard_hsv
    Ten::_ZBUFFER_SIMPLIFY_.set_standard_hsv_(box_lists);
    log.record_image(Ten::_INIT_3D_BOX_.box_lists_);
    //sleep(3);
    ros::Rate sl(10);
    bool status = true;

    int have_debug = 0;


    while(status)
    {
        status = Ten::_TREADPOOL_FLAG_.read_flag();
        std::cout << "status: " << status << std::endl;
        //重置感兴趣区域和存在
        Ten::_ZBUFFER_SIMPLIFY_.set_interested_boxes(map_s.interested_boxes_);
        Ten::_ZBUFFER_SIMPLIFY_.set_exist_boxes(map_s.exist_boxes_);

        //读取图片
        cv::Mat image_in = camera.camera_read();
        sl.sleep();
        Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data());
        //设置世界到雷达
        Ten::_CAMERA_TRANSFORMATION_.set_worldtolidar(tf);
        //坐标变换
        Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(Ten::_INIT_3D_BOX_.pcl_LS_plum_object_points_, 
            Ten::_INIT_3D_BOX_.pcl_C_plum_object_points_, Ten::_INIT_3D_BOX_.object_plum_points);
        //拆分
        Ten::_INIT_3D_BOX_.split_2d_points();



        //初始化zb用的数据
        Ten::_ZBUFFER_SIMPLIFY_.set_box_lists_init(Ten::_INIT_3D_BOX_.C_object_points_, Ten::_INIT_3D_BOX_.C_plum_points_, 
            Ten::_INIT_3D_BOX_.object_2d_points, Ten::_INIT_3D_BOX_.plum_2d_points,
            Ten::_INIT_3D_BOX_.object_2d_points_, Ten::_INIT_3D_BOX_.plum_2d_points_);

        //zb
        Ten::_ZBUFFER_SIMPLIFY_.set_box_lists_(image_in,  Ten::_INIT_3D_BOX_.object_2d_points_, 
            Ten::_INIT_3D_BOX_.plum_2d_points_ ,Ten::_INIT_3D_BOX_.box_lists_);
        std::vector<Ten::box>& box_lists = Ten::_INIT_3D_BOX_.box_lists_;

        //判断空
        Ten::_ZBUFFER_SIMPLIFY_.set_HSV_exist_boxes_(box_lists);

        


        //模型初步筛选
        for(int i = 0; i < box_lists.size(); i++)
        {
            if(box_lists[i].exist_flag == 1 && box_lists[i].zbuffer_flag == 1)
            {
                std::vector<Ten::Detection> result = detector.worker(box_lists[i].roi_image);
                if(result[0].conf_ > 0.98)
                {
                    map_s.interested_boxes_[box_lists[i].idx - 1] = 0;
                    map_s.exist_boxes_[box_lists[i].idx - 1] = 1;
                    box_lists[i].zbuffer_flag = 0;
                }
                else
                {
                    box_lists[i].exist_flag = 0;
                }

                if(map_s.object_confidence_[box_lists[i].idx] < result[0].conf_)
                {
                    map_s.object_[box_lists[i].idx] = result[0].cls_id_;
                    map_s.object_confidence_[box_lists[i].idx] = result[0].conf_;
                    box_lists[i].cls = result[0].cls_id_;
                    box_lists[i].confidence = result[0].conf_;
                    std::cout<< "update where" << box_lists[i].idx <<std::endl;
                }
            }
            else if(box_lists[i].zbuffer_flag == 1)
            {
                std::vector<Ten::Detection> result = detector.worker(box_lists[i].roi_image);
                if(map_s.object_confidence_[box_lists[i].idx] < result[0].conf_)
                {
                    map_s.object_[box_lists[i].idx] = result[0].cls_id_;
                    map_s.object_confidence_[box_lists[i].idx] = result[0].conf_;
                    box_lists[i].cls = result[0].cls_id_;
                    box_lists[i].confidence = result[0].conf_;
                    std::cout<< "update where" << box_lists[i].idx <<std::endl;
                }
            }

            // if(box_lists[i].exist_flag == 0)
            // {
            //     std::vector<Ten::Detection> result = detector.worker(box_lists[i].roi_image);
            //     if(result[0].conf_ < 0.3)
            //     {
            //         map_s.exist_boxes_[box_lists[i].idx - 1] = 0;
            //         box_lists[i].zbuffer_flag = 0;
            //     }
            // }
        }

        //发布调试图像
        cv::Mat debug_best_roi_image = cv::Mat::zeros(480, 640, CV_8UC3);
        Ten::_ZBUFFER_SIMPLIFY_.set_debug_roi_image(box_lists, debug_best_roi_image);
        sensor_msgs::ImagePtr pub_debug_roi_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_best_roi_image).toImageMsg();
        //调试图像
        //Ten::debug_draw_img(image_in, Ten::_INIT_3D_BOX_.object_2d_points);
        cv::Mat debug_raw_img = Ten::_ZBUFFER_SIMPLIFY_.update_debug_image(image_in, Ten::_INIT_3D_BOX_.object_2d_points_);
        sensor_msgs::ImagePtr pub_debug_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_raw_img).toImageMsg();
        debug_roi_pub.publish(pub_debug_roi_msg);
        pub_img.publish(pub_debug_img_msg);

        // cv::imshow("img", debug_raw_img);
        // cv::waitkey(30);

        int cout = 0;
        for(int i = 0; i < 12; i++)
        {
            if(map_s.exist_boxes_[i] == 1)
            {
                cout++;
            }
        }

        have_debug++;
        if(have_debug >= 30 && cout < 8)
        {
            log.record_image(Ten::_INIT_3D_BOX_.box_lists_);
            have_debug = 0;
        }

    }


    //打印最终结果
    for(int i = 1; i <= 12; i++)
    {
        std::cout<< "map_s.object_[" << i << "]"<< map_s.object_[i] <<std::endl;
    }

    log.record_image(Ten::_INIT_3D_BOX_.box_lists_);
    log.record_map(Ten::_Map_GET_.read_data());
    log.record_odometry(Ten::_TF_GET_.read_data());

    //camera.~Ten_camera();
    urcu_memb_unregister_thread();



}



// 里程计订阅回调函数
void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
{
    nav_msgs::Odometry odo = *odom_msg;
    Ten::_TF_CAMERA_GET_.write_data(odo);
}

void odom()
{
    urcu_memb_register_thread();

    ros::NodeHandle nh;

    // 订阅里程计话题：/ORB_SLAM3/odometry
    ros::Subscriber odom_sub = nh.subscribe("/ORB_SLAM3/odometry", 2, OdomCallback);
    ros::Publisher pub_img = nh.advertise<sensor_msgs::Image>("/camera/image_raw", 2);

    ROS_INFO("里程计订阅节点已启动，正在订阅 /ORB_SLAM3/odometry ...");


    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance(640,480,30);
    //ros::spin();
    ros::Rate sl(30);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        cv::Mat image_in = camera.camera_read();
        // 核心：cv_bridge 转换 OpenCV(Mat) → ROS(sensor_msgs/Image)
        // 编码格式：bgr8 （ORB-SLAM3完美兼容，OpenCV读取的图像默认就是BGR顺序）
        cv_bridge::CvImage cv_img;
        cv_img.header.stamp = ros::Time::now(); // 设置时间戳
        cv_img.header.frame_id = "camera";      // 设置坐标系（可选）
        cv_img.encoding = "bgr8";               // 关键：指定图像编码，适配ORB
        cv_img.image = image_in;                    // 赋值OpenCV图像矩阵

        // 转换为ROS消息并发布
        sensor_msgs::ImagePtr ros_img = cv_img.toImageMsg();
        pub_img.publish(ros_img);
        //std::cout<<"pub_img.publish(ros_img)"<<std::endl;
        ros::spinOnce();
        sl.sleep();
    }


    urcu_memb_unregister_thread();
}

void calibration_merge()
{
    urcu_memb_register_thread();
    ros::Rate sl(10);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        string input;
        std::cin >> input;
        if(input == "s")
        {
            Ten::XYZRPY lidar_tf = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data());
            Ten::XYZRPY camera_tf = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_CAMERA_GET_.read_data());
            Ten::_CALCULATE_LIDAR_TO_CAMERA_TRANSFORM_.set_lidar_and_camera_T(Ten::XYZRPYtotransform_matrix(lidar_tf), Ten::XYZRPYtotransform_matrix(camera_tf));
            std::cout<< "------------------设置变换矩阵----------------------------------"<<std::endl;
        }
        else if(input == "c")
        {
            Eigen::Matrix4d T = Ten::_CALCULATE_LIDAR_TO_CAMERA_TRANSFORM_.get_calibration();
            std::cout<< "------------------相机变换矩阵----------------------------------"<<std::endl;
            std::cout << T << std::endl;
            std::cout<< "------------------xxxxxxxxxx----------------------------------"<<std::endl;
            return;
        }
        
        sl.sleep();
    }


    urcu_memb_unregister_thread();
}




// int main(int argc, char **argv)
// {
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//     }

//     ros::init(argc, argv, "merge_node");
//     ros::NodeHandle nh;

//     //test4();
//     Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws11/src/merge/src/livox_ros_driver2/config/MID360_config.json");
//     Ten::ThreadPool pool(1);
//     pool.enqueue(serial_send_lidar);

//     laserMapping();
//     Ten::Ten_lidar::GetInstance().~Ten_lidar();
//     return 0;
// }

// int main(int argc, char **argv)
// {
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//     }

//     ros::init(argc, argv, "merge_node");
//     ros::NodeHandle nh;

//     test_relocation();
//     return 0;
// }


// int main(int argc, char **argv)
// {
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//     ros::console::notifyLoggerLevelsChanged();
//     }

//     ros::init(argc, argv, "merge_node");
//     ros::NodeHandle nh;

//     Ten::Ten_lidar::GetInstance("/home/maple/study2/merge_ws13/src/merge/src/livox_ros_driver2/config/MID360_config.json");
//     // Ten::ThreadPool pool(1);
//     // pool.enqueue(test_speed);
//     laserMapping();
//     Ten::Ten_lidar::GetInstance().~Ten_lidar();

//     return 0;
// }














#endif


