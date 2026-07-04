#include "merge_func.cpp"





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

//     test5();
//     return 0;
// }


int main(int argc, char **argv)
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug)) {
    ros::console::notifyLoggerLevelsChanged();
    }

    ros::init(argc, argv, "merge_node");
    ros::NodeHandle nh;

    Ten::Ten_lidar::GetInstance("/home/h/RC2026/merge_ws20/src/merge/src/livox_ros_driver2/config/MID360_config.json");
    Ten::ThreadPool pool(1);
    pool.enqueue(zbuffer);
    //pool.enqueue(test_speed);
    
    laserMapping();
    Ten::Ten_lidar::GetInstance().~Ten_lidar();

    return 0;
}
