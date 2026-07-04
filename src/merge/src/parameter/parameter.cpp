#include "parameter.h"




namespace Ten
{
    namespace parameter
    {
        std::mutex parameter_mtx_;
    }

    namespace superstratum
    {
        // #define _r1_xyzrpy_car_xyz_x_ -0.40944
        // #define _r1_xyzrpy_car_xyz_y_ 0.40944 + 0.088 /2
        // #define _r1_xyzrpy_car_xyz_z_ 0
        // #define _r1_xyzrpy_car_rpy_roll_ 0
        // #define _r1_xyzrpy_car_rpy_pitch_ 0
        // #define _r1_xyzrpy_car_rpy_yaw_ -M_PI / 2.0

        // #define _r1_xyzrpy_error_xyz_x_ 0.025
        // #define _r1_xyzrpy_error_xyz_y_ -0.045
        // #define _r1_xyzrpy_error_xyz_z_ 0.10
        // #define _r1_xyzrpy_error_rpy_roll_ 0
        // #define _r1_xyzrpy_error_rpy_pitch_ 0
        // #define _r1_xyzrpy_error_rpy_yaw_ 0
        
        //r1车相对雷达
        double _r1_xyzrpy_car_xyz_x_ = 0;
        double _r1_xyzrpy_car_xyz_y_ = 0;
        double _r1_xyzrpy_car_xyz_z_ = 0;
        double _r1_xyzrpy_car_rpy_roll_ = 0;
        double _r1_xyzrpy_car_rpy_pitch_ = 0;
        double _r1_xyzrpy_car_rpy_yaw_ = 0;
        //r1建图误差
        double _r1_xyzrpy_error_xyz_x_ = 0;
        double _r1_xyzrpy_error_xyz_y_ = 0;
        double _r1_xyzrpy_error_xyz_z_ = 0;
        double _r1_xyzrpy_error_rpy_roll_ = 0;
        double _r1_xyzrpy_error_rpy_pitch_ = 0;
        double _r1_xyzrpy_error_rpy_yaw_ = 0;

        //r1雷达初始误差
        std::atomic<double> _r1_xyzrpy_init_error_xyz_x_;
        std::atomic<double> _r1_xyzrpy_init_error_xyz_y_;
        std::atomic<double> _r1_xyzrpy_init_error_xyz_z_;
        std::atomic<double> _r1_xyzrpy_init_error_rpy_roll_;
        std::atomic<double> _r1_xyzrpy_init_error_rpy_pitch_;
        std::atomic<double> _r1_xyzrpy_init_error_rpy_yaw_;

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

        //r2车相对雷达
        double _r2_xyzrpy_car_xyz_x_ = 0;
        double _r2_xyzrpy_car_xyz_y_ = 0;
        double _r2_xyzrpy_car_xyz_z_ = 0;
        double _r2_xyzrpy_car_rpy_roll_ = 0;
        double _r2_xyzrpy_car_rpy_pitch_ = 0;
        double _r2_xyzrpy_car_rpy_yaw_ = 0;

        //r2建图误差
        double _r2_xyzrpy_error_xyz_x_ = 0;
        double _r2_xyzrpy_error_xyz_y_ = 0;
        double _r2_xyzrpy_error_xyz_z_ = 0;
        double _r2_xyzrpy_error_rpy_roll_ = 0;
        double _r2_xyzrpy_error_rpy_pitch_ = 0;
        double _r2_xyzrpy_error_rpy_yaw_ = 0;

        //r2雷达初始误差
        std::atomic<double> _r2_xyzrpy_init_error_xyz_x_;
        std::atomic<double> _r2_xyzrpy_init_error_xyz_y_;
        std::atomic<double> _r2_xyzrpy_init_error_xyz_z_;
        std::atomic<double> _r2_xyzrpy_init_error_rpy_roll_;
        std::atomic<double> _r2_xyzrpy_init_error_rpy_pitch_;
        std::atomic<double> _r2_xyzrpy_init_error_rpy_yaw_;

        // #define _coner_path_ "/home/robocon/rc2026/model/corner5/best"
        // #define _juanzhou_path_ "/home/robocon/rc2026/model/juanZhou_cls1/best"
        // #define _box_num_ 5  //8
        //识别卷轴
        std::string _coner_path_;
        std::string _juanzhou_path_;
        int _box_num_ = 0;


        
        // #define _roi12_path_ "/home/h/下载/yolo11s_roi12_atten_17/yolo11s_roi12_atten_17"
        // //#define _juanzhou_path_ "/home/h/下载/卷轴分类1_仿真+现实_32类/best"
        // //#define _box_num_ 8
        // #define _limit_count_1_ 3
        // #define _limit_count_2_ 3
        // #define _limit_count_3_ 2
        // #define _limit_count_4_ 4
        //识别卷轴2
        std::string _roi12_path_;
        int _limit_count_1_ = 0;
        int _limit_count_2_ = 0;
        int _limit_count_3_ = 0;
        int _limit_count_4_ = 0;

        //雷达到相机 4x4 变换矩阵
        Eigen::Matrix4d _lidar_to_camera_transform_matrix_;
    }

    //#define _max_serial_num_ 10
    //串口最大遍历数值
    int _max_serial_num_ = 0;

    // #define _voxeldownsample_threshold_for_teaser_   0.3  //0.6
    // #define _voxeldownsample_threshold_for_icp_   0.5  //0.6
    // #define _setmaxcorrespondencedistance_nano_gicp_  0.55 //0.65
    // #define _min_num_of_point_cloud_for_relocation_ 20000
    //重定位参数
    double _voxeldownsample_threshold_for_teaser_ = 0.0;
    double _voxeldownsample_threshold_for_icp_ = 0.0;
    double _setmaxcorrespondencedistance_nano_gicp_ = 0.0;
    size_t _min_num_of_point_cloud_for_relocation_ = 0;
    size_t _min_num_of_point_cloud_for_relocation2_ = 0;
    
    //#define _voxeldownsample_threshold_ 0.3
    //point_lio建图下采样深度
    double _voxeldownsample_threshold_ = 0.0;

    // 640x480
    double camera_fx_640 = 0;
    double camera_fy_640 = 0;
    double camera_cx_640 = 0;
    double camera_cy_640 = 0;

    // 1920x1080
    double camera_fx_1080 = 0;
    double camera_fy_1080 = 0;
    double camera_cx_1080 = 0;
    double camera_cy_1080 = 0;
    //发布频率
    double _laser_pub_hz_ = 200.0;

    //ekf
    double _q_pos_   = 0;   // 位置过程噪声系数
    double _q_att_   = 0;   // 姿态过程噪声系数
    double _q_vel_   = 0;   // 线速度过程噪声系数
    double _q_ang_   = 0;   // 角速度过程噪声系数


    //雷达自己的初始误差
    double _lidar_xyzrpy_init_error_xyz_x_ = 0;
    double _lidar_xyzrpy_init_error_xyz_y_ = 0;
    double _lidar_xyzrpy_init_error_xyz_z_ = 0;
    double _lidar_xyzrpy_init_error_rpy_roll_ = 0;
    double _lidar_xyzrpy_init_error_rpy_pitch_ = 0;
    double _lidar_xyzrpy_init_error_rpy_yaw_ = 0.0;

    // //camera_kfs
    // double _camera_x_bias_ = 0;
    // double _camera_y_bias_ = 0;
    // double _camera_z_bias_ = 0;
    // double _max_bias_  = 0.2;

    // ==============================================
    // AprilTag 全局变量声明
    // ==============================================
    double APRILTAG_FX = 0.0;
    double APRILTAG_FY = 0.0;
    double APRILTAG_CX = 0.0;
    double APRILTAG_CY = 0.0;
    double APRILTAG_K1 = 0.0;
    double APRILTAG_K2 = 0.0;
    double APRILTAG_P1 = 0.0;
    double APRILTAG_P2 = 0.0;
    double APRILTAG_K3 = 0.0;

    int APRILTAG_CAMERA_WIDTH = 0;
    int APRILTAG_CAMERA_HEIGHT = 0;
    int APRILTAG_CAMERA_FPS = 0;

    double APRILTAG_TAG_SIZE = 0.0;
    double APRILTAG_TAG_SPACING = 0.0;
    double APRILTAG_QUAD_DECIMATE = 0.0;
    double APRILTAG_QUAD_SIGMA = 0.0;
    int APRILTAG_NTHREADS = 0;
    bool APRILTAG_REFINE_EDGES = false;

    int APRILTAG_PRIMARY_ID = 0;
    int APRILTAG_AUX_ID = 0;
    bool APRILTAG_SINGLE_TAG_MODE = false;
    int APRILTAG_SINGLE_TAG_ID = 0;
    double APRILTAG_FUSION_BIAS_X = 0.0;
    double APRILTAG_FUSION_BIAS_Y = 0.0;
    double APRILTAG_FUSION_BIAS_Z = 0.0;

    double APRILTAG_WORLD_BIAS_X = 0.0;
    double APRILTAG_WORLD_BIAS_Y = 0.0;
    double APRILTAG_WORLD_BIAS_Z = 0.0;

    int APRILTAG_JUMP_WINDOW = 0;
    double APRILTAG_YAW_EMA_ALPHA = 0.0;

    bool APRILTAG_ENABLE_SAW_WINDOW = false;
    double APRILTAG_DRAW_FONT_SCALE = 0.0;
    int APRILTAG_DRAW_THICKNESS = 0;
    std::string APRILTAG_SAW_WINDOW_NAME = "";

    int _usb_device_num1_ = 0;
    int _usb_device_num2_ = 0;
    int _usb_device_num3_ = 0;

    //imuekf
    double _imu_q_pos_   = 0;   // 位置过程噪声系数
    double _imu_q_att_   = 0;   // 姿态过程噪声系数
    double _imu_q_vel_   = 0;   // 线速度过程噪声系数

    std::vector<int> _global_path_;

    std::string _kfs_path_;

    //输入参数
    int _input_parameter_[30] = {0};

    double _max_z_ = 0.0;
    
}