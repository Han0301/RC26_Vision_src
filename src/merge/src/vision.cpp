#ifndef __VISION_CPP_
#define __VISION_CPP_




#include "./superstratum/super.h"
#include "test.cpp"
#include "test2.cpp"
#include "./lidar/lidar_recognition.h"
#include "./lidar/lidar_getttf.h"

void orb_test()
{
    urcu_memb_register_thread();
    //orb
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);

    // int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
    // cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
    // cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
    // cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
    // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
    // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);
    
    // cv::Mat img = cv::imread("/home/maple/study2/hou/image1/img1.jpg");
    // cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.36314436, -1.01831353,  0.96407981);
    // cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.58769593,  0.78453107,  0.6021223);
    // cv::Mat img2 = cv::imread("/home/maple/study2/hou/image1/img2.jpg");
    // cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.19823447, -1.19880829,  1.22625982);
    // cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -2.99413314,  0.72745418, -2.86621733);

    cv::Mat img = cv::imread("/home/maple/study2/hou/image1/2.jpg");
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.15538523, -1.06445562,  1.18891875);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -3.1360852,   0.65997508, -1.54297095);
    cv::Mat img2 = cv::imread("/home/maple/study2/hou/image1/1.jpg");
    cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.39449508, -1.03322981,  0.93960988);
    cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -4.7881104,   0.77966552,  0.75880949);

    Ten::ORB::orb_optimize_exhaust ooe("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);



    std::vector<Ten::ORB::orb_exhaust_element> oees;
    Ten::ORB::orb_exhaust_element oee;
    oee.image_ = img;
    oee.rvec_ = rvec;
    oee.tvec_ = tvec;
    oees.push_back(oee);
    oee.image_ = img2;
    oee.rvec_ = rvec2;
    oee.tvec_ = tvec2;
    oees.push_back(oee);
    std::vector<int> place = ooe.getplace(oees);



    std::cout << "place: ";
    for(auto it : place)
    {
        std::cout << it << ",";
    }
    std::cout << std::endl;


    std::vector<double> losses = ooe.get_loss();
    for(size_t i = 0; i < losses.size(); i++)
    {
        std::cout << "loss[" << i << "]: " << losses[i] << std::endl;
    }

    std::vector<Ten::ORB::orb_exhaust_element> oprt = ooe.get_RT(oees);
    log.record_imageRT(oees);
    log.record_imageRT(oprt);

    //检验
    int exist_box[12] = {0};
    for(size_t k = 0; k < 12 && k < place.size(); k++)
    {
        exist_box[k] = place[k];
    }

    {
        //zubbfer
        Ten::init_3d_box world_point;
        Ten::Ten_occlusion_handing zbuffer;
        for(size_t i = 0; i < oees.size(); i++)
        {
            Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_RT(oees[i].rvec_, oees[i].tvec_);
            Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
            world_point.pcl_to_C();
            //zbuffer.set_exist_boxes(exist_box);
            //zbuffer.set_interested_boxes(exist_box);
            zbuffer.set_box_lists_(oees[i].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
            cv::Mat debug1  = cv::Mat::zeros(480, 640, CV_8UC3); 
            zbuffer.set_debug_roi_image(world_point.box_lists_, debug1);
            cv::imshow("debug1", debug1);
            cv::waitKey(0);
        }
        log.record_image(world_point.box_lists_);
    }

    {
        //zubbfer
        Ten::init_3d_box world_point;
        Ten::Ten_occlusion_handing zbuffer;
        for(size_t i = 0; i < oprt.size(); i++)
        {
            Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_RT(oprt[i].rvec_, oprt[i].tvec_);
            Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
            world_point.pcl_to_C();
            zbuffer.set_exist_boxes(exist_box);
            zbuffer.set_box_lists_(oprt[i].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
            cv::Mat debug2  = cv::Mat::zeros(480, 640, CV_8UC3); 
            zbuffer.set_debug_roi_image(world_point.box_lists_, debug2);
            cv::imshow("debug2", debug2);
            cv::waitKey(0); 
        }
        log.record_image(world_point.box_lists_);
    }
    urcu_memb_unregister_thread();
}


void test_calibration_lidar_camera()
{
    urcu_memb_register_thread();
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
    int flag = 0;
    std::vector<Ten::ORB::orb_exhaust_element> imageRTs;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        Ten::ORB::orb_exhaust_element imageRT;
        cv::Mat image = camera.camera_read();
        cv::imshow("debug", image);
        flag = cv::waitKey(0);
        if(flag == 's')
        {
            cv::Mat image = camera.camera_read();
            imageRT.image_ = image;
            imageRTs.push_back(imageRT);
            log.record_odometry(Ten::_TF_GET_.read_data());
        }
        else if(flag == 'x')
        {
            break;
        }

    }
    log.record_imageRT(imageRTs);
    urcu_memb_unregister_thread();
}
void vision_test1()
{
    urcu_memb_register_thread();
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
    log.record_odometry(nav_msgs::Odometry());
    log.record_odometry(nav_msgs::Odometry());
    log.record_odometry(nav_msgs::Odometry());
    urcu_memb_unregister_thread();
}

// void vision_test2()
// {
//     urcu_memb_register_thread();
//     std::vector<Ten::Ten_camerainfo> vc = Ten::readCameraInfosFromDir("/home/maple/study2/merge_ws32/src/merge/log/2026_3_9/11/imageRT/imagert2");
//     for(size_t i = 0; i < vc.size(); i++)
//     {
//         std::cout << i << std::endl;
//         std::cout << vc[i].rvec() << std::endl;
//         std::cout << vc[i].tvec() << std::endl;
//         std::cout << vc[i].extrinsic() << std::endl;
//     }
//     std::vector<Ten::XYZRPY> vpose = Ten::readPoseFromTxt("/home/maple/study2/merge_ws32/src/merge/log/2026_3_11/17/odometry.txt");
//     std::cout << "vpose.size(): " << vpose.size() << std::endl;
//     for(auto tf : vpose)
//     {
//         std::cout << "---------------------------" << std::endl; 
//         std::cout << "x: " << tf._xyz._x << std::endl;
//         std::cout << "y: " << tf._xyz._y << std::endl;
//         std::cout << "z: " << tf._xyz._z << std::endl;
//         std::cout << "roll: " << tf._rpy._roll << std::endl;
//         std::cout << "pitch: " << tf._rpy._pitch << std::endl;
//         std::cout << "yaw: " << tf._rpy._yaw << std::endl;
//     }
//     urcu_memb_unregister_thread();
// }

void vision_test3()
{
    urcu_memb_register_thread();
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
    int flag = 0;
    std::vector<Ten::ORB::orb_exhaust_element> imageRTs;
    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        Ten::ORB::orb_exhaust_element imageRT;
        //cv::Mat image = camera.camera_read();
        std::cin >> flag;
        if(flag == 1)
        {
            std::cout << "save_img" << std::endl;
            cv::Mat image = camera.camera_read();
            image.copyTo(imageRT.image_);
            imageRTs.push_back(imageRT);
            log.record_odometry(Ten::_TF_GET_.read_data());
            Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data());
            std::cout << "---------------------------" << std::endl; 
            std::cout << "x: " << tf._xyz._x << std::endl;
            std::cout << "y: " << tf._xyz._y << std::endl;
            std::cout << "z: " << tf._xyz._z << std::endl;
            std::cout << "roll: " << tf._rpy._roll << std::endl;
            std::cout << "pitch: " << tf._rpy._pitch << std::endl;
            std::cout << "yaw: " << tf._rpy._yaw << std::endl;
        }
        else if(flag == 0)
        {
            std::cout << "out" << std::endl;
            break;
        }
    }
    log.record_imageRT(imageRTs);
    urcu_memb_unregister_thread();
}


//测试标定代码
void test_calibration()
{
    urcu_memb_register_thread();
    
    //用于调试rt转换器
    
    //初始化ros调试
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_img = it.advertise("pub_image_topic", 2);
    //初始化相机
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    //设置相机内外参
    cv::Mat K = (cv::Mat_<double>(3,3) <<
    1380.4350, 0, 974.0183,
    0,  1385.0788, 541.4301,
    0, 0, 1);
    Eigen::Matrix4d transform_matrix;
    // transform_matrix << 
    // -0.0794166,  -0.996838,  -0.00276348,  0.0197102,  
    // -0.045364,  0.00638342,  -0.99895,  0.36517,  
    // 0.995809,  -0.0792079,  -0.0457275,  0.511095,  
    // 0.0         ,  0.0        ,  0.0        ,  1.0;
    // transform_matrix << 
    // -0.0520106,  -0.998646,  0.000310088,  0.0206641,  
    // -0.0471048,  0.00214311,  -0.998888,  0.396765,  
    // 0.997535,  -0.0519674,  -0.0471525,  0.50734,  
    // 0.0         ,  0.0        ,  0.0        ,  1.0;       
    transform_matrix << 
    -0.0293067,  -0.999359,  -0.0205564,  0.0519757,  
    0.0195515,  0.0199882,  -0.999609,  0.47424,  
    0.999379,  -0.0296971,  0.0189532,  0.336381,  
    0.0         ,  0.0        ,  0.0        ,  1.0; 

    Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_K(K);
    Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_Extrinsic_Matrix(transform_matrix);
    ros::Rate sl(10);
    size_t debug_num = 0;
    //zb
    Ten::init_3d_box world_point;
    Ten::Ten_occlusion_handing zbuffer;
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        //读取图片
        cv::Mat image_in = camera.camera_read();
        Ten::XYZRPY tf = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data());
        //打印tf信息
        // std::cout << "---------------------------" << std::endl; 
        // std::cout << "x: " << tf._xyz._x << std::endl;
        // std::cout << "y: " << tf._xyz._y << std::endl;
        // std::cout << "z: " << tf._xyz._z << std::endl;
        // std::cout << "roll: " << tf._rpy._roll << std::endl;
        // std::cout << "pitch: " << tf._rpy._pitch << std::endl;
        // std::cout << "yaw: " << tf._rpy._yaw << std::endl;
        //设置世界到雷达
        Ten::_CAMERA_TRANSFORMATION_.set_worldtolidar(tf);
        Eigen::Matrix4d world_to_camera_test =  Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
        if(debug_num >= 10)
        {
            std::cout << "world_to_camera_test: " << std::endl;
            std::cout << world_to_camera_test << std::endl;
            debug_num = 0;
        }
        debug_num++;
        world_point.pcl_to_C();
        zbuffer.set_box_lists_(image_in, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
        cv::Mat debug = zbuffer.update_debug_image(image_in, world_point.object_plum_2d_points_);
        sensor_msgs::ImagePtr pub_debug_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug).toImageMsg();
        pub_img.publish(pub_debug_img_msg);
        sl.sleep();
        
    }
    urcu_memb_unregister_thread();
}


//视觉代码
void vision_code()
{
    urcu_memb_register_thread();
    int flag = 0;
    Ten::superstratum::super vision_controller;
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        std::cout << "wait for input" << std::endl;
        std::cin >> flag;
        if(flag == 1)
        {
            std::cout << "flag: " << flag << std::endl;
            vision_controller.set_image_for_orb();
        }
        else if(flag == 2)
        {
            std::cout << "flag: "  << flag << std::endl;
            vision_controller.set_image_for_yolo();
        }
        else if(flag == 3)
        {
            std::cout << "flag: "  << flag << std::endl;
            double loss = vision_controller.estimate_square_position();
            if(loss > 1000)
            {
                std::cout << "place can't find" << std::endl;
            }
            else
            {
                std::vector<int> cls = vision_controller.estimate_classifier();
                std::cout << "cls: ";
                for(auto it : cls)
                {
                    std::cout << it << ",";
                }
                std::cout << std::endl;
            }
        }
        else if(flag == 0)
        {
            std::cout << "flag: "  << flag << std::endl;
            return;
        }
    }
    urcu_memb_unregister_thread();
}

void publishimg()
{
    urcu_memb_register_thread();
    ros::NodeHandle nh("~");
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub_img = it.advertise("pub_image_topic", 2);
    //初始化相机
    Ten::Ten_camera& camera =  Ten::Ten_camera::GetInstance();
    ros::Rate sl(5);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        //读取图片
        cv::Mat image_in = camera.camera_read();
        sensor_msgs::ImagePtr pub_debug_img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_in).toImageMsg();
        pub_img.publish(pub_debug_img_msg);
        sl.sleep();
    }
    urcu_memb_unregister_thread();
}

// void vision_calibration_of_tf_imagert()
// {
//     urcu_memb_register_thread();
//     std::vector<Eigen::Matrix4d> camera;
//     std::vector<Eigen::Matrix4d> lidar;
//     Eigen::Matrix4d camera_first;
//     Eigen::Matrix4d lidar_first;

//     std::vector<Ten::Ten_camerainfo> vc = Ten::readCameraInfosFromDir("/home/maple/study2/li/imagert1/imagert1");
//     for(size_t i = 0; i < vc.size(); i++)
//     {
//         //std::cout << i << std::endl;
//         // std::cout << vc[i].rvec() << std::endl;
//         // std::cout << vc[i].tvec() << std::endl;
//         // std::cout << vc[i].extrinsic() << std::endl; 
//         if(i == 0)
//         {
//             camera_first = vc[i].extrinsic();
//         }
//         else
//         {
//             Eigen::Matrix4d relative = vc[i].extrinsic() * camera_first.inverse();
//             Ten::XYZRPY tf = Ten::transform_matrixtoXYZRPY(relative);
//             std::cout << "-------------camera--------------" << std::endl; 
//             std::cout << "x: " << tf._xyz._x << std::endl;
//             std::cout << "y: " << tf._xyz._y << std::endl;
//             std::cout << "z: " << tf._xyz._z << std::endl;
//             std::cout << "roll: " << tf._rpy._roll << std::endl;
//             std::cout << "pitch: " << tf._rpy._pitch << std::endl;
//             std::cout << "yaw: " << tf._rpy._yaw << std::endl;
//             camera.push_back(relative);
//         }
//     }

//     std::vector<Ten::XYZRPY> vpose = Ten::readPoseFromTxt("/home/maple/study2/merge_ws35/src/merge/log/2026_3_20/3/odometry.txt");
//     std::cout << "vpose.size(): " << vpose.size() << std::endl;
//     for(size_t i = 0; i < vpose.size(); i++)
//     {
//         Ten::XYZRPY tfp = vpose[i];
//         // std::cout << "---------------------------" << std::endl; 
//         // std::cout << "x: " << tf._xyz._x << std::endl;
//         // std::cout << "y: " << tf._xyz._y << std::endl;
//         // std::cout << "z: " << tf._xyz._z << std::endl;
//         // std::cout << "roll: " << tf._rpy._roll << std::endl;
//         // std::cout << "pitch: " << tf._rpy._pitch << std::endl;
//         // std::cout << "yaw: " << tf._rpy._yaw << std::endl;
//         if(i == 0)
//         {
//             lidar_first = Ten::worldtocurrent(tfp._xyz, tfp._rpy);
//         }
//         else
//         {
//             Eigen::Matrix4d relative = Ten::worldtocurrent(tfp._xyz, tfp._rpy) * lidar_first.inverse();
//             Ten::XYZRPY tf = Ten::transform_matrixtoXYZRPY(relative);
//             std::cout << "-------------lidar--------------" << std::endl; 
//             std::cout << "x: " << tf._xyz._x << std::endl;
//             std::cout << "y: " << tf._xyz._y << std::endl;
//             std::cout << "z: " << tf._xyz._z << std::endl;
//             std::cout << "roll: " << tf._rpy._roll << std::endl;
//             std::cout << "pitch: " << tf._rpy._pitch << std::endl;
//             std::cout << "yaw: " << tf._rpy._yaw << std::endl;
//             lidar.push_back(relative);
//         }
//     }


//     std::cout << "------------开始标定数据-----------------" << std::endl;
//     Ten::Ten_calibration tcl;

//     for(size_t i = 0; i < lidar.size() && i < camera.size(); i++)
//     {
//         tcl.set_lidar_and_camera_T(lidar[i], camera[i]);
//     }

    
//     Eigen::Matrix4d transform_matrix;
//     transform_matrix << 
//     -0.0794166,  -0.996838,  -0.00276348,  0.0197102,  
//     -0.045364,  0.00638342,  -0.99895,  0.36517,  
//     0.995809,  -0.0792079,  -0.0457275,  0.511095,  
//     0.0         ,  0.0        ,  0.0        ,  1.0;

//     tcl.set_c(transform_matrix);

//     Eigen::Matrix4d transform_matrix2 = tcl.get_calibration();

//     std::cout << "----------------标定结果-------------"<< std::endl;
//     std::cout << transform_matrix2 << std::endl;
//     std::cout << transform_matrix2.inverse() << std::endl;

//     urcu_memb_unregister_thread();
// }


void test_performance()
{
    urcu_memb_register_thread();
    int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
    cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
    cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
    cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
    cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);
    std::vector<Ten::ORB::orb_exhaust_element> oees;
    Ten::ORB::orb_exhaust_element oee;
    oee.image_ = img;
    oee.rvec_ = rvec;
    oee.tvec_ = tvec;
    oees.push_back(oee);
    oee.image_ = img2;
    oee.rvec_ = rvec2;
    oee.tvec_ = tvec2;
    oees.push_back(oee);
    //zubbfer
    auto start = std::chrono::steady_clock::now();
    Ten::init_3d_box world_point;
    Ten::Ten_occlusion_handing zbuffer;
    for(size_t i = 0; i < 495 && Ten::_TREADPOOL_FLAG_.read_flag(); i++)
    {
        for(size_t j = 0; j < oees.size(); j++)
        {
            Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_RT(oees[j].rvec_, oees[j].tvec_);
            Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
            world_point.pcl_to_C();
            //zbuffer.set_exist_boxes(exist_box);
            //zbuffer.set_interested_boxes(exist_box);
            zbuffer.set_box_lists_(oees[j].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
            cv::Mat debug1  = cv::Mat::zeros(480, 640, CV_8UC3); 
            zbuffer.set_debug_roi_image(world_point.box_lists_, debug1);
        }
    }
    // 记录结束时间
    auto end = std::chrono::steady_clock::now();
    // 计算耗时（转换为毫秒）
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "1--简单计时 - 耗时：" << duration_ms << " 毫秒" << std::endl;
    urcu_memb_unregister_thread();
}

void test_performance2()
{
    //Ten::bind_thread_to_cpu(15);
    urcu_memb_register_thread();
    int exist_box[12] = {1,1,1,0,0,1,1,1,0,0,1,1};
    cv::Mat img = cv::imread("/home/maple/study2/hou/image.png");
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.406094, -0.900495, 0.902471);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -4.384002, 1.098546, 1.598849);
    cv::Mat img2 = cv::imread("/home/maple/study2/hou/image2.png");
    cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.273942, -1.317019, 1.148317);
    cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -3.018514, 1.472847, -1.608665);
    std::vector<Ten::ORB::orb_exhaust_element> oees;
    Ten::ORB::orb_exhaust_element oee;
    oee.image_ = img;
    oee.rvec_ = rvec;
    oee.tvec_ = tvec;
    oees.push_back(oee);
    oee.image_ = img2;
    oee.rvec_ = rvec2;
    oee.tvec_ = tvec2;
    oees.push_back(oee);
    //zubbfer
    // 记录开始时间（steady_clock 避免系统时间干扰）
    auto start = std::chrono::steady_clock::now();

    Ten::init_3d_box world_point;
    Ten::Ten_occlusion_handing zbuffer;
    for(size_t i = 0; i < 495 && Ten::_TREADPOOL_FLAG_.read_flag(); i++)
    {
        for(size_t j = 0; j < oees.size(); j++)
        {
            Ten::_CAMERA_TRANSFORMATION_.camerainfo_.set_RT(oees[j].rvec_, oees[j].tvec_);
            Ten::_CAMERA_TRANSFORMATION_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
            world_point.pcl_to_C();
            //zbuffer.set_exist_boxes(exist_box);
            //zbuffer.set_interested_boxes(exist_box);
            zbuffer.set_box_lists_(oees[j].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
            cv::Mat debug1  = cv::Mat::zeros(480, 640, CV_8UC3); 
            zbuffer.set_debug_roi_image(world_point.box_lists_, debug1);
        }
    }
    // 记录结束时间
    auto end = std::chrono::steady_clock::now();
    // 计算耗时（转换为毫秒）
    auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    std::cout << "2--简单计时 - 耗时：" << duration_ms << " 毫秒" << std::endl;
    urcu_memb_unregister_thread();
}


void vision_calibration2()
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

    Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::worldtocurrent(xyzrpy._xyz, xyzrpy._rpy) * Ten::worldtocurrent(xyzrpy_error._xyz, xyzrpy_error._rpy));

    Ten::_CAMERA_TRANSFORMATION_.set_world2toworld1(world_origin);
}

void vision_test_input()
{
    urcu_memb_register_thread();
    Ten::superstratum::super sp_controller;
    Ten::XYZRPY last;
    Ten::XYZRPY now;
    int input = 0;
    ros::Rate sl(3);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        int flag = 0;
        std::cout << "input: " << std::endl;
        std::cout << "flag: " << std::endl;
        if(input == 0)
        {
            //flag = 0;
            std::cin >> flag;
        }
        
        if(flag == 1)
        {
            std::cout<< "flag == 1" << std::endl;
            sp_controller.use_relocation2();
        }
        else if(input != 0 || flag == 2)
        {
            input = 3;
            //flag = 2;
            now = Ten::Nav_Odometrytoxyzrpy(Ten::_TF_GET_.read_data());
            if(now.Eclidean_distance(last) >= 0.15 || std::abs(now._rpy._yaw - last._rpy._yaw) > 0.1)
            {
                last = now;
                sp_controller.set_image_for_yolo();
                std::cout << "save" << std::endl;
            }
            else
            {
                std::cout<< "please-move" << std::endl;
            }
            
        }
        else if(flag == 0)
        {
            break;
        }
        sl.sleep();
    }
    sp_controller.set_image_for_yolo();
    sp_controller.save_logRT();
    urcu_memb_unregister_thread();
}

void test_vision_save()
{
    std::string log_path = std::string(ROOT_DIR) + std::string("log");
    std::string map_path = std::string(ROOT_DIR) + std::string("path/map.txt");
    
    Ten::Ten_logger& log = Ten::Ten_logger::GetInstance(log_path);
    std::vector<int> map = Ten::readNumberFile(map_path);
    cv::Mat img = cv::imread("/home/maple/study2/hou/image1/2.jpg");
    cv::Mat rvec = (cv::Mat_<double>(3, 1) << 1.15538523, -1.06445562,  1.18891875);
    cv::Mat tvec = (cv::Mat_<double>(3, 1) << -3.1360852,   0.65997508, -1.54297095);
    cv::Mat img2 = cv::imread("/home/maple/study2/hou/image1/1.jpg");
    cv::Mat rvec2 = (cv::Mat_<double>(3, 1) << 1.39449508, -1.03322981,  0.93960988);
    cv::Mat tvec2 = (cv::Mat_<double>(3, 1) << -4.7881104,   0.77966552,  0.75880949);

    Ten::ORB::orb_optimize_exhaust ooe("/home/maple/study2/hou/corner2_v5s_1000_260202/best", "cpu", 0.3, 0.3, 0);



    std::vector<Ten::ORB::orb_exhaust_element> oees;
    Ten::ORB::orb_exhaust_element oee;
    oee.image_ = img;
    oee.rvec_ = rvec;
    oee.tvec_ = tvec;
    oees.push_back(oee);
    oee.image_ = img2;
    oee.rvec_ = rvec2;
    oee.tvec_ = tvec2;
    oees.push_back(oee);


    log.record_imageRT(oees, map);
}

void vision_lidarR1()
{
    urcu_memb_register_thread();
    ros::NodeHandle nh;
    // 创建TF广播器
    tf2_ros::TransformBroadcaster tf_broadcaster;

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
    


    ros::Rate sl(10);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        //位置变化
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

        arr[3] = roll * 180.0 / M_PI;
        arr[4] = pitch * 180.0 / M_PI;
        arr[5] = yaw * 180.0 / M_PI;

        geometry_msgs::TransformStamped tf_msg;
        // 1. 消息头配置
        tf_msg.header.stamp = ros::Time::now();
        tf_msg.header.frame_id = "test_lidar";       // 父坐标系
        tf_msg.child_frame_id = "car_true";    // 子坐标系（动态运动）

    
        tf_msg.transform.translation.x = result._xyz._x;

        tf_msg.transform.translation.y = result._xyz._y;

        tf_msg.transform.translation.z = result._xyz._z / 2.0;

        // 4. 设置旋转：默认值 0（无旋转，四元数 0,0,0,1）
        tf2::Quaternion q;
        q.setRPY(0, 0, 0);  // 横滚/俯仰/偏航 全部默认0
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();


        // 发布TF变换
        tf_broadcaster.sendTransform(tf_msg);

        sl.sleep();
    }
    
    urcu_memb_unregister_thread();
}
void vision_calibration3()
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

void vision_test_input_lidar()
{
    urcu_memb_register_thread();
    Ten::lidar::lidar_recogniton lidar_r;
    int input = 0;
    ros::Rate sl(10);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        int flag = 0;
        // std::cout << "input: " << std::endl;
        // std::cout << "flag: " << std::endl;
        if(input == 0)
        {
            //flag = 0;
            std::cin >> flag;
        }
        if(flag == 1)
        {
            std::cout<< "flag == 1" << std::endl;
            vision_calibration3();
        }
        else if(input != 0 || flag == 2)
        {
            input = 3;
            lidar_r.get_current_cloud();
            lidar_r.point_cloud_debug();
        }
        else if(flag == 0)
        {
            break;
        }
        sl.sleep();
    }
    urcu_memb_unregister_thread();
}

void vision_test_lidar_getttf()
{
    Ten::lidar::lidar_getttf lg;
    ros::Rate sl(10);
    while(Ten::_TREADPOOL_FLAG_.read_flag())
    {
        ros::spinOnce();
        sl.sleep();
    }
}



#endif


