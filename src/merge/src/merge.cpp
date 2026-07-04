//#include "merge_func.cpp"
#include "control.cpp"
#include "vision.cpp"


/**
 * @brief 自定义SIGINT信号处理函数（捕获Ctrl+C）
 * @param sig 接收到的信号值（SIGINT对应值为2）
 */
void sigintHandler(int sig)
{
    // 1. 打印退出提示（可选）
    //ROS_INFO("收到Ctrl+C信号，开始优雅退出...");
    std::cout<< "收到Ctrl+C信号,开始优雅退出..."<< std::endl;

    Ten::_TREADPOOL_FLAG_.set_flag(false);

    // 4. 关闭ROS节点（必须调用，否则ROS资源不会释放）
    

    // sleep(10);
    // std::cout<<"exit(0);"<< std::endl;
    // ros::shutdown();
    // exit(0);
    // 注意：这里不要直接exit(0)，交给main函数执行return 0更优雅
}


// int main(int argc, char **argv)
// {
//     if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
//         ros::console::notifyLoggerLevelsChanged();
//     }

//     //ros::init(argc, argv, "merge_node");
//     //关键：NoSigintHandler → 禁用ROS默认的SIGINT处理，改用自定义逻辑
    
//     ros::init(argc, argv, "merge_node", ros::init_options::NoSigintHandler);



//     // ========== 第二步：创建NodeHandle（必须！否则ros::ok()无效） ==========
//     ros::NodeHandle nh;
//     //3. 注册自定义SIGINT信号处理函数（替换默认处理）
//     signal(SIGINT, sigintHandler);




//     //Ten::Ten_lidar::GetInstance("/home/rc/RC_2026/merge_ws21/src/merge/src/livox_ros_driver2/config/MID360_config.json");
//     Ten::ThreadPool pool(2);
//     pool.enqueue(test_relocation);
//     //pool.enqueue(serial_receiver);
//     //laserMapping();

//     return 0;
// }




#ifdef _R1_R1_
    int main(int argc, char **argv)
    {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }

        //ros::init(argc, argv, "merge_node");
        //关键：NoSigintHandler → 禁用ROS默认的SIGINT处理，改用自定义逻辑
        
        ros::init(argc, argv, "merge_node", ros::init_options::NoSigintHandler);



        // ========== 第二步：创建NodeHandle（必须！否则ros::ok()无效） ==========
        ros::NodeHandle nh;
        //3. 注册自定义SIGINT信号处理函数（替换默认处理）
        signal(SIGINT, sigintHandler);

        std::cout << "🐋: " << "R1" << std::endl;
        // std::string lidar_path = std::string(ROOT_DIR) + std::string("src/livox_ros_driver2/config/MID360_config.json");
        // Ten::Ten_lidar::GetInstance(lidar_path);
        // Ten::ThreadPool pool(2);
        // pool.enqueue(serial_send_lidarR1);
        // pool.enqueue(serial_receiver);
        // laserMapping(); 
        return 0;
    }
#elif defined(_R2_R2_)
    int main(int argc, char **argv)
    {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }

        //ros::init(argc, argv, "merge_node");
        //关键：NoSigintHandler → 禁用ROS默认的SIGINT处理，改用自定义逻辑
        
        ros::init(argc, argv, "merge_node", ros::init_options::NoSigintHandler);



        // ========== 第二步：创建NodeHandle（必须！否则ros::ok()无效） ==========
        ros::NodeHandle nh;
        //3. 注册自定义SIGINT信号处理函数（替换默认处理）
        signal(SIGINT, sigintHandler);

        std::cout << "🐀: " << "R2" << std::endl;
        // std::string lidar_path = std::string(ROOT_DIR) + std::string("src/livox_ros_driver2/config/MID360_config.json");
        // Ten::Ten_lidar::GetInstance(lidar_path);
        // Ten::ThreadPool pool(2);
        // pool.enqueue(serial_send_lidarR2);
        // pool.enqueue(serial_receiver);
        // laserMapping(); 
        return 0;
    }
#else
    int main(int argc, char **argv)
    {
        if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
            ros::console::notifyLoggerLevelsChanged();
        }

        //ros::init(argc, argv, "merge_node");
        //关键：NoSigintHandler → 禁用ROS默认的SIGINT处理，改用自定义逻辑
        
        ros::init(argc, argv, "merge_node", ros::init_options::NoSigintHandler);



        // ========== 第二步：创建NodeHandle（必须！否则ros::ok()无效） ==========
        ros::NodeHandle nh;
        //3. 注册自定义SIGINT信号处理函数（替换默认处理）
        signal(SIGINT, sigintHandler);

        std::cout << "🐅: " << "R0" << std::endl;
        // std::string lidar_path = std::string(ROOT_DIR) + std::string("src/livox_ros_driver2/config/MID360_config.json");
        // Ten::Ten_lidar::GetInstance(lidar_path);
        Ten::ThreadPool pool(1);
        //pool.enqueue(orb_test);
        pool.enqueue(vision_test2);
        //laserMapping();
 
        
        return 0;
    }
#endif