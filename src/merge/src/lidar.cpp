#ifndef __LIDAR_CPP_
#define __LIDAR_CPP_

#include "lidar.h"

namespace Ten
{


    
Ten_one_write_multiple_read<nav_msgs::Odometry> _TF_GET_;
Ten_one_write_multiple_read<nav_msgs::Odometry> _TF_CAMERA_GET_;
// Ten_one_write_multiple_read<livox_ros_driver::CustomMsg> _LIVOX_GET_;
// Ten_one_write_multiple_read<sensor_msgs::Imu> _IMU_GET_;
Ten_one_write_multiple_read<sensor_msgs::PointCloud2> _Map_GET_;


}





#endif



