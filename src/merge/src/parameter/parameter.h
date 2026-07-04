#ifndef __PARAMETER_H_
#define __PARAMETER_H_
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <string>
#include <Eigen/Dense>  

namespace Ten
{    
    namespace superstratum
    {
        //r1车相对雷达
        extern double _r1_xyzrpy_car_xyz_x_;
        extern double _r1_xyzrpy_car_xyz_y_;
        extern double _r1_xyzrpy_car_xyz_z_;
        extern double _r1_xyzrpy_car_rpy_roll_;
        extern double _r1_xyzrpy_car_rpy_pitch_;
        extern double _r1_xyzrpy_car_rpy_yaw_;
        //r1建图误差
        extern double _r1_xyzrpy_error_xyz_x_;
        extern double _r1_xyzrpy_error_xyz_y_;
        extern double _r1_xyzrpy_error_xyz_z_;
        extern double _r1_xyzrpy_error_rpy_roll_;
        extern double _r1_xyzrpy_error_rpy_pitch_;
        extern double _r1_xyzrpy_error_rpy_yaw_;
        //r1雷达初始误差
        extern double _r1_xyzrpy_init_error_xyz_x_;
        extern double _r1_xyzrpy_init_error_xyz_y_;
        extern double _r1_xyzrpy_init_error_xyz_z_;
        extern double _r1_xyzrpy_init_error_rpy_roll_;
        extern double _r1_xyzrpy_init_error_rpy_pitch_;
        extern double _r1_xyzrpy_init_error_rpy_yaw_;
        //r2车相对雷达
        extern double _r2_xyzrpy_car_xyz_x_;
        extern double _r2_xyzrpy_car_xyz_y_;
        extern double _r2_xyzrpy_car_xyz_z_;
        extern double _r2_xyzrpy_car_rpy_roll_;
        extern double _r2_xyzrpy_car_rpy_pitch_;
        extern double _r2_xyzrpy_car_rpy_yaw_;
        //r2建图误差
        extern double _r2_xyzrpy_error_xyz_x_;
        extern double _r2_xyzrpy_error_xyz_y_;
        extern double _r2_xyzrpy_error_xyz_z_ ;
        extern double _r2_xyzrpy_error_rpy_roll_;
        extern double _r2_xyzrpy_error_rpy_pitch_;
        extern double _r2_xyzrpy_error_rpy_yaw_;
        //r2雷达初始误差
        extern double _r2_xyzrpy_init_error_xyz_x_;
        extern double _r2_xyzrpy_init_error_xyz_y_;
        extern double _r2_xyzrpy_init_error_xyz_z_;
        extern double _r2_xyzrpy_init_error_rpy_roll_;
        extern double _r2_xyzrpy_init_error_rpy_pitch_;
        extern double _r2_xyzrpy_init_error_rpy_yaw_;
        //识别卷轴
        extern std::string _coner_path_;
        extern std::string _juanzhou_path_;
        extern int _box_num_;
        //识别卷轴2
        extern std::string _roi12_path_;
        extern int _limit_count_1_;
        extern int _limit_count_2_;
        extern int _limit_count_3_;
        extern int _limit_count_4_;

        //雷达到相机 4x4 变换矩阵
        extern Eigen::Matrix4d _lidar_to_camera_transform_matrix_;
    }
    //串口最大遍历数值
    extern int _max_serial_num_;
    //重定位参数
    extern double _voxeldownsample_threshold_for_teaser_;
    extern double _voxeldownsample_threshold_for_icp_;
    extern double _setmaxcorrespondencedistance_nano_gicp_;
    extern size_t _min_num_of_point_cloud_for_relocation_;
    extern size_t _min_num_of_point_cloud_for_relocation2_;
    //point_lio建图下采样深度
    extern double _voxeldownsample_threshold_;
    // 640x480
    extern double camera_fx_640;
    extern double camera_fy_640;
    extern double camera_cx_640;
    extern double camera_cy_640;
    // 1920x1080
    extern double camera_fx_1080;
    extern double camera_fy_1080;
    extern double camera_cx_1080;
    extern double camera_cy_1080;
    extern double _laser_pub_hz_;

    //ekf
    extern double _q_pos_;   // 位置过程噪声系数
    extern double _q_att_;   // 姿态过程噪声系数
    extern double _q_vel_;   // 线速度过程噪声系数
    extern double _q_ang_;   // 角速度过程噪声系数

    //雷达自己的初始误差
    extern double _lidar_xyzrpy_init_error_xyz_x_;
    extern double _lidar_xyzrpy_init_error_xyz_y_;
    extern double _lidar_xyzrpy_init_error_xyz_z_;
    extern double _lidar_xyzrpy_init_error_rpy_roll_;
    extern double _lidar_xyzrpy_init_error_rpy_pitch_;
    extern double _lidar_xyzrpy_init_error_rpy_yaw_;

    // //camera_kfs
    // extern double _camera_x_bias_;
    // extern double _camera_y_bias_;
    // extern double _camera_z_bias_;
    // extern double _max_bias_;

    // ==============================================
    // AprilTag extern 声明
    // ==============================================
    extern double APRILTAG_FX;
    extern double APRILTAG_FY;
    extern double APRILTAG_CX;
    extern double APRILTAG_CY;
    extern double APRILTAG_K1;
    extern double APRILTAG_K2;
    extern double APRILTAG_P1;
    extern double APRILTAG_P2;
    extern double APRILTAG_K3;

    extern int APRILTAG_CAMERA_WIDTH;
    extern int APRILTAG_CAMERA_HEIGHT;
    extern int APRILTAG_CAMERA_FPS;

    extern double APRILTAG_TAG_SIZE;
    extern double APRILTAG_TAG_SPACING;
    extern double APRILTAG_QUAD_DECIMATE;
    extern double APRILTAG_QUAD_SIGMA;
    extern int APRILTAG_NTHREADS;
    extern bool APRILTAG_REFINE_EDGES;

    extern int APRILTAG_PRIMARY_ID;
    extern int APRILTAG_AUX_ID;
    extern bool APRILTAG_SINGLE_TAG_MODE;
    extern int APRILTAG_SINGLE_TAG_ID;
    extern double APRILTAG_FUSION_BIAS_X;
    extern double APRILTAG_FUSION_BIAS_Y;
    extern double APRILTAG_FUSION_BIAS_Z;

    extern double APRILTAG_WORLD_BIAS_X;
    extern double APRILTAG_WORLD_BIAS_Y;
    extern double APRILTAG_WORLD_BIAS_Z;

    extern int APRILTAG_JUMP_WINDOW;
    extern double APRILTAG_YAW_EMA_ALPHA;

    extern bool APRILTAG_ENABLE_SAW_WINDOW;
    extern double APRILTAG_DRAW_FONT_SCALE;
    extern int APRILTAG_DRAW_THICKNESS;
    extern std::string APRILTAG_SAW_WINDOW_NAME;

    extern int _usb_device_num1_;
    extern int _usb_device_num2_;
    extern int _usb_device_num3_;

    //imuekf
    extern double _imu_q_pos_;   // 位置过程噪声系数
    extern double _imu_q_att_;   // 姿态过程噪声系数
    extern double _imu_q_vel_;   // 线速度过程噪声系数

    namespace parameter
    {

        class loadyaml
        {
        public:
        loadyaml()
        {
            // 直接读取 yaml 文件
            YAML::Node config = YAML::LoadFile(std::string(ROOT_DIR) + std::string("src/parameter/config.yaml"));

            // ==============================================
            // 【superstratum 命名空间】所有参数赋值
            // ==============================================
            superstratum::_r1_xyzrpy_car_xyz_x_                = config["_r1_xyzrpy_car_xyz_x_"].as<double>();
            superstratum::_r1_xyzrpy_car_xyz_y_                = config["_r1_xyzrpy_car_xyz_y_"].as<double>();
            superstratum::_r1_xyzrpy_car_xyz_z_                = config["_r1_xyzrpy_car_xyz_z_"].as<double>();
            superstratum::_r1_xyzrpy_car_rpy_roll_             = config["_r1_xyzrpy_car_rpy_roll_"].as<double>();
            superstratum::_r1_xyzrpy_car_rpy_pitch_            = config["_r1_xyzrpy_car_rpy_pitch_"].as<double>();
            superstratum::_r1_xyzrpy_car_rpy_yaw_              = config["_r1_xyzrpy_car_rpy_yaw_"].as<double>();

            superstratum::_r1_xyzrpy_error_xyz_x_              = config["_r1_xyzrpy_error_xyz_x_"].as<double>();
            superstratum::_r1_xyzrpy_error_xyz_y_              = config["_r1_xyzrpy_error_xyz_y_"].as<double>();
            superstratum::_r1_xyzrpy_error_xyz_z_              = config["_r1_xyzrpy_error_xyz_z_"].as<double>();
            superstratum::_r1_xyzrpy_error_rpy_roll_           = config["_r1_xyzrpy_error_rpy_roll_"].as<double>();
            superstratum::_r1_xyzrpy_error_rpy_pitch_          = config["_r1_xyzrpy_error_rpy_pitch_"].as<double>();
            superstratum::_r1_xyzrpy_error_rpy_yaw_            = config["_r1_xyzrpy_error_rpy_yaw_"].as<double>();

            superstratum::_r1_xyzrpy_init_error_xyz_x_              = config["_r1_xyzrpy_init_error_xyz_x_"].as<double>();
            superstratum::_r1_xyzrpy_init_error_xyz_y_              = config["_r1_xyzrpy_init_error_xyz_y_"].as<double>();
            superstratum::_r1_xyzrpy_init_error_xyz_z_              = config["_r1_xyzrpy_init_error_xyz_z_"].as<double>();
            superstratum::_r1_xyzrpy_init_error_rpy_roll_           = config["_r1_xyzrpy_init_error_rpy_roll_"].as<double>();
            superstratum::_r1_xyzrpy_init_error_rpy_pitch_          = config["_r1_xyzrpy_init_error_rpy_pitch_"].as<double>();
            superstratum::_r1_xyzrpy_init_error_rpy_yaw_            = config["_r1_xyzrpy_init_error_rpy_yaw_"].as<double>();

            superstratum::_r2_xyzrpy_car_xyz_x_                = config["_r2_xyzrpy_car_xyz_x_"].as<double>();
            superstratum::_r2_xyzrpy_car_xyz_y_                = config["_r2_xyzrpy_car_xyz_y_"].as<double>();
            superstratum::_r2_xyzrpy_car_xyz_z_                = config["_r2_xyzrpy_car_xyz_z_"].as<double>();
            superstratum::_r2_xyzrpy_car_rpy_roll_             = config["_r2_xyzrpy_car_rpy_roll_"].as<double>();
            superstratum::_r2_xyzrpy_car_rpy_pitch_            = config["_r2_xyzrpy_car_rpy_pitch_"].as<double>();
            superstratum::_r2_xyzrpy_car_rpy_yaw_              = config["_r2_xyzrpy_car_rpy_yaw_"].as<double>();

            superstratum::_r2_xyzrpy_error_xyz_x_              = config["_r2_xyzrpy_error_xyz_x_"].as<double>();
            superstratum::_r2_xyzrpy_error_xyz_y_              = config["_r2_xyzrpy_error_xyz_y_"].as<double>();
            superstratum::_r2_xyzrpy_error_xyz_z_              = config["_r2_xyzrpy_error_xyz_z_"].as<double>();
            superstratum::_r2_xyzrpy_error_rpy_roll_           = config["_r2_xyzrpy_error_rpy_roll_"].as<double>();
            superstratum::_r2_xyzrpy_error_rpy_pitch_          = config["_r2_xyzrpy_error_rpy_pitch_"].as<double>();
            superstratum::_r2_xyzrpy_error_rpy_yaw_            = config["_r2_xyzrpy_error_rpy_yaw_"].as<double>();

            superstratum::_r2_xyzrpy_init_error_xyz_x_              = config["_r2_xyzrpy_init_error_xyz_x_"].as<double>();
            superstratum::_r2_xyzrpy_init_error_xyz_y_              = config["_r2_xyzrpy_init_error_xyz_y_"].as<double>();
            superstratum::_r2_xyzrpy_init_error_xyz_z_              = config["_r2_xyzrpy_init_error_xyz_z_"].as<double>();
            superstratum::_r2_xyzrpy_init_error_rpy_roll_           = config["_r2_xyzrpy_init_error_rpy_roll_"].as<double>();
            superstratum::_r2_xyzrpy_init_error_rpy_pitch_          = config["_r2_xyzrpy_init_error_rpy_pitch_"].as<double>();
            superstratum::_r2_xyzrpy_init_error_rpy_yaw_            = config["_r2_xyzrpy_init_error_rpy_yaw_"].as<double>();

            superstratum::_coner_path_                         = config["_coner_path_"].as<std::string>();
            superstratum::_juanzhou_path_                      = config["_juanzhou_path_"].as<std::string>();
            superstratum::_box_num_                            = config["_box_num_"].as<int>();

            superstratum::_roi12_path_                         = config["_roi12_path_"].as<std::string>();
            superstratum::_limit_count_1_                      = config["_limit_count_1_"].as<int>();
            superstratum::_limit_count_2_                      = config["_limit_count_2_"].as<int>();
            superstratum::_limit_count_3_                      = config["_limit_count_3_"].as<int>();
            superstratum::_limit_count_4_                      = config["_limit_count_4_"].as<int>();

            // ==============================================
            // 全局变量赋值
            // ==============================================
            _max_serial_num_                                   = config["_max_serial_num_"].as<int>();
            _voxeldownsample_threshold_for_teaser_             = config["_voxeldownsample_threshold_for_teaser_"].as<double>();
            _voxeldownsample_threshold_for_icp_                = config["_voxeldownsample_threshold_for_icp_"].as<double>();
            _setmaxcorrespondencedistance_nano_gicp_           = config["_setmaxcorrespondencedistance_nano_gicp_"].as<double>();
            _min_num_of_point_cloud_for_relocation_            = config["_min_num_of_point_cloud_for_relocation_"].as<size_t>();
            _min_num_of_point_cloud_for_relocation2_            = config["_min_num_of_point_cloud_for_relocation2_"].as<size_t>();
            _voxeldownsample_threshold_                        = config["_voxeldownsample_threshold_"].as<double>();

            // 相机内参
            // ==============================================
            Ten::camera_fx_640                                 = config["camera_fx_640"].as<double>();
            Ten::camera_fy_640                                 = config["camera_fy_640"].as<double>();
            Ten::camera_cx_640                                 = config["camera_cx_640"].as<double>();
            Ten::camera_cy_640                                 = config["camera_cy_640"].as<double>();

            Ten::camera_fx_1080                                = config["camera_fx_1080"].as<double>();
            Ten::camera_fy_1080                                = config["camera_fy_1080"].as<double>();
            Ten::camera_cx_1080                                = config["camera_cx_1080"].as<double>();
            Ten::camera_cy_1080                                = config["camera_cy_1080"].as<double>();

            // ==============================================
            // 雷达到相机 4x4 变换矩阵
            // ==============================================
            Ten::superstratum::_lidar_to_camera_transform_matrix_(0,0) = config["lidar_to_camera_00"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(0,1) = config["lidar_to_camera_01"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(0,2) = config["lidar_to_camera_02"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(0,3) = config["lidar_to_camera_03"].as<double>();

            Ten::superstratum::_lidar_to_camera_transform_matrix_(1,0) = config["lidar_to_camera_10"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(1,1) = config["lidar_to_camera_11"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(1,2) = config["lidar_to_camera_12"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(1,3) = config["lidar_to_camera_13"].as<double>();

            Ten::superstratum::_lidar_to_camera_transform_matrix_(2,0) = config["lidar_to_camera_20"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(2,1) = config["lidar_to_camera_21"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(2,2) = config["lidar_to_camera_22"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(2,3) = config["lidar_to_camera_23"].as<double>();

            Ten::superstratum::_lidar_to_camera_transform_matrix_(3,0) = config["lidar_to_camera_30"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(3,1) = config["lidar_to_camera_31"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(3,2) = config["lidar_to_camera_32"].as<double>();
            Ten::superstratum::_lidar_to_camera_transform_matrix_(3,3) = config["lidar_to_camera_33"].as<double>();

            Ten::_laser_pub_hz_                                = config["_laser_pub_hz_"].as<double>();


            Ten::_q_pos_                                = config["_q_pos_"].as<double>();
            Ten::_q_att_                                = config["_q_att_"].as<double>();
            Ten::_q_vel_                                = config["_q_vel_"].as<double>();
            Ten::_q_ang_                                = config["_q_ang_"].as<double>();

            Ten::_lidar_xyzrpy_init_error_xyz_x_              = config["_lidar_xyzrpy_init_error_xyz_x_"].as<double>();
            Ten::_lidar_xyzrpy_init_error_xyz_y_              = config["_lidar_xyzrpy_init_error_xyz_y_"].as<double>();
            Ten::_lidar_xyzrpy_init_error_xyz_z_              = config["_lidar_xyzrpy_init_error_xyz_z_"].as<double>();
            Ten::_lidar_xyzrpy_init_error_rpy_roll_           = config["_lidar_xyzrpy_init_error_rpy_roll_"].as<double>();
            Ten::_lidar_xyzrpy_init_error_rpy_pitch_          = config["_lidar_xyzrpy_init_error_rpy_pitch_"].as<double>();
            Ten::_lidar_xyzrpy_init_error_rpy_yaw_            = config["_lidar_xyzrpy_init_error_rpy_yaw_"].as<double>();


            //  //camera_kfs
            // Ten::_camera_x_bias_                                = config["_camera_x_bias_"].as<double>();
            // Ten::_camera_y_bias_                                = config["_camera_y_bias_"].as<double>();
            // Ten::_camera_z_bias_                                = config["_camera_z_bias_"].as<double>();
            // Ten::_max_bias_                                     = config["_max_bias_"].as<double>();

            // ==============================================
            // 全局变量赋值
            // ==============================================
            APRILTAG_FX                            = config["APRILTAG_FX"].as<double>();
            APRILTAG_FY                            = config["APRILTAG_FY"].as<double>();
            APRILTAG_CX                            = config["APRILTAG_CX"].as<double>();
            APRILTAG_CY                            = config["APRILTAG_CY"].as<double>();
            APRILTAG_K1                            = config["APRILTAG_K1"].as<double>();
            APRILTAG_K2                            = config["APRILTAG_K2"].as<double>();
            APRILTAG_P1                            = config["APRILTAG_P1"].as<double>();
            APRILTAG_P2                            = config["APRILTAG_P2"].as<double>();
            APRILTAG_K3                            = config["APRILTAG_K3"].as<double>();

            APRILTAG_CAMERA_WIDTH                   = config["APRILTAG_CAMERA_WIDTH"].as<int>();
            APRILTAG_CAMERA_HEIGHT                  = config["APRILTAG_CAMERA_HEIGHT"].as<int>();
            APRILTAG_CAMERA_FPS                     = config["APRILTAG_CAMERA_FPS"].as<int>();

            APRILTAG_TAG_SIZE                       = config["APRILTAG_TAG_SIZE"].as<double>();
            APRILTAG_TAG_SPACING                    = config["APRILTAG_TAG_SPACING"].as<double>();
            APRILTAG_QUAD_DECIMATE                  = config["APRILTAG_QUAD_DECIMATE"].as<double>();
            APRILTAG_QUAD_SIGMA                     = config["APRILTAG_QUAD_SIGMA"].as<double>();
            APRILTAG_NTHREADS                       = config["APRILTAG_NTHREADS"].as<int>();
            APRILTAG_REFINE_EDGES                   = config["APRILTAG_REFINE_EDGES"].as<bool>();

            APRILTAG_PRIMARY_ID                     = config["APRILTAG_PRIMARY_ID"].as<int>();
            APRILTAG_AUX_ID                         = config["APRILTAG_AUX_ID"].as<int>();
            APRILTAG_SINGLE_TAG_MODE                = config["APRILTAG_SINGLE_TAG_MODE"].as<bool>();
            APRILTAG_SINGLE_TAG_ID                  = config["APRILTAG_SINGLE_TAG_ID"].as<int>();
            APRILTAG_FUSION_BIAS_X                  = config["APRILTAG_FUSION_BIAS_X"].as<double>();
            APRILTAG_FUSION_BIAS_Y                  = config["APRILTAG_FUSION_BIAS_Y"].as<double>();
            APRILTAG_FUSION_BIAS_Z                  = config["APRILTAG_FUSION_BIAS_Z"].as<double>();

            APRILTAG_WORLD_BIAS_X                   = config["APRILTAG_WORLD_BIAS_X"].as<double>();
            APRILTAG_WORLD_BIAS_Y                   = config["APRILTAG_WORLD_BIAS_Y"].as<double>();
            APRILTAG_WORLD_BIAS_Z                   = config["APRILTAG_WORLD_BIAS_Z"].as<double>();

            APRILTAG_JUMP_WINDOW                    = config["APRILTAG_JUMP_WINDOW"].as<int>();
            APRILTAG_YAW_EMA_ALPHA                  = config["APRILTAG_YAW_EMA_ALPHA"].as<double>();

            APRILTAG_ENABLE_SAW_WINDOW              = config["APRILTAG_ENABLE_SAW_WINDOW"].as<bool>();
            APRILTAG_DRAW_FONT_SCALE                = config["APRILTAG_DRAW_FONT_SCALE"].as<double>();
            APRILTAG_DRAW_THICKNESS                 = config["APRILTAG_DRAW_THICKNESS"].as<int>();
            APRILTAG_SAW_WINDOW_NAME                = config["APRILTAG_SAW_WINDOW_NAME"].as<std::string>();

            _usb_device_num1_                 = config["_usb_device_num1_"].as<int>();
            _usb_device_num2_                 = config["_usb_device_num2_"].as<int>();
            _usb_device_num3_                 = config["_usb_device_num3_"].as<int>();


            Ten::_imu_q_pos_                                = config["_imu_q_pos_"].as<double>();
            Ten::_imu_q_att_                                = config["_imu_q_att_"].as<double>();
            Ten::_imu_q_vel_                                = config["_imu_q_vel_"].as<double>();


            // ==============================================
            // 【全部打印】加载完成后输出所有参数
            // ==============================================
            std::cout << "\n==================== 加载 YAML 参数完成 ====================\n" << std::endl;

            std::cout << "--- r1 车相对雷达 ---" << std::endl;
            std::cout << "_r1_xyzrpy_car_xyz_x_:               " << superstratum::_r1_xyzrpy_car_xyz_x_ << std::endl;
            std::cout << "_r1_xyzrpy_car_xyz_y_:               " << superstratum::_r1_xyzrpy_car_xyz_y_ << std::endl;
            std::cout << "_r1_xyzrpy_car_xyz_z_:               " << superstratum::_r1_xyzrpy_car_xyz_z_ << std::endl;
            std::cout << "_r1_xyzrpy_car_rpy_roll_:            " << superstratum::_r1_xyzrpy_car_rpy_roll_ << std::endl;
            std::cout << "_r1_xyzrpy_car_rpy_pitch_:           " << superstratum::_r1_xyzrpy_car_rpy_pitch_ << std::endl;
            std::cout << "_r1_xyzrpy_car_rpy_yaw_:             " << superstratum::_r1_xyzrpy_car_rpy_yaw_ << std::endl;

            std::cout << "\n--- r1 建图误差 ---" << std::endl;
            std::cout << "_r1_xyzrpy_error_xyz_x_:             " << superstratum::_r1_xyzrpy_error_xyz_x_ << std::endl;
            std::cout << "_r1_xyzrpy_error_xyz_y_:             " << superstratum::_r1_xyzrpy_error_xyz_y_ << std::endl;
            std::cout << "_r1_xyzrpy_error_xyz_z_:             " << superstratum::_r1_xyzrpy_error_xyz_z_ << std::endl;
            std::cout << "_r1_xyzrpy_error_rpy_roll_:          " << superstratum::_r1_xyzrpy_error_rpy_roll_ << std::endl;
            std::cout << "_r1_xyzrpy_error_rpy_pitch_:         " << superstratum::_r1_xyzrpy_error_rpy_pitch_ << std::endl;
            std::cout << "_r1_xyzrpy_error_rpy_yaw_:           " << superstratum::_r1_xyzrpy_error_rpy_yaw_ << std::endl;

            std::cout << "\n--- r1 雷达初始误差 ---" << std::endl;
            std::cout << "_r1_xyzrpy_init_error_xyz_x_:             " << superstratum::_r1_xyzrpy_init_error_xyz_x_ << std::endl;
            std::cout << "_r1_xyzrpy_init_error_xyz_y_:             " << superstratum::_r1_xyzrpy_init_error_xyz_y_ << std::endl;
            std::cout << "_r1_xyzrpy_init_error_xyz_z_:             " << superstratum::_r1_xyzrpy_init_error_xyz_z_ << std::endl;
            std::cout << "_r1_xyzrpy_init_error_rpy_roll_:          " << superstratum::_r1_xyzrpy_init_error_rpy_roll_ << std::endl;
            std::cout << "_r1_xyzrpy_init_error_rpy_pitch_:         " << superstratum::_r1_xyzrpy_init_error_rpy_pitch_ << std::endl;
            std::cout << "_r1_xyzrpy_init_error_rpy_yaw_:           " << superstratum::_r1_xyzrpy_init_error_rpy_yaw_ << std::endl;

            std::cout << "\n--- r2 车相对雷达 ---" << std::endl;
            std::cout << "_r2_xyzrpy_car_xyz_x_:               " << superstratum::_r2_xyzrpy_car_xyz_x_ << std::endl;
            std::cout << "_r2_xyzrpy_car_xyz_y_:               " << superstratum::_r2_xyzrpy_car_xyz_y_ << std::endl;
            std::cout << "_r2_xyzrpy_car_xyz_z_:               " << superstratum::_r2_xyzrpy_car_xyz_z_ << std::endl;
            std::cout << "_r2_xyzrpy_car_rpy_roll_:            " << superstratum::_r2_xyzrpy_car_rpy_roll_ << std::endl;
            std::cout << "_r2_xyzrpy_car_rpy_pitch_:           " << superstratum::_r2_xyzrpy_car_rpy_pitch_ << std::endl;
            std::cout << "_r2_xyzrpy_car_rpy_yaw_:             " << superstratum::_r2_xyzrpy_car_rpy_yaw_ << std::endl;

            std::cout << "\n--- r2 建图误差 ---" << std::endl;
            std::cout << "_r2_xyzrpy_error_xyz_x_:             " << superstratum::_r2_xyzrpy_error_xyz_x_ << std::endl;
            std::cout << "_r2_xyzrpy_error_xyz_y_:             " << superstratum::_r2_xyzrpy_error_xyz_y_ << std::endl;
            std::cout << "_r2_xyzrpy_error_xyz_z_:             " << superstratum::_r2_xyzrpy_error_xyz_z_ << std::endl;
            std::cout << "_r2_xyzrpy_error_rpy_roll_:          " << superstratum::_r2_xyzrpy_error_rpy_roll_ << std::endl;
            std::cout << "_r2_xyzrpy_error_rpy_pitch_:         " << superstratum::_r2_xyzrpy_error_rpy_pitch_ << std::endl;
            std::cout << "_r2_xyzrpy_error_rpy_yaw_:           " << superstratum::_r2_xyzrpy_error_rpy_yaw_ << std::endl;

            std::cout << "\n--- r2 雷达初始误差 ---" << std::endl;
            std::cout << "_r2_xyzrpy_init_error_xyz_x_:             " << superstratum::_r2_xyzrpy_init_error_xyz_x_ << std::endl;
            std::cout << "_r2_xyzrpy_init_error_xyz_y_:             " << superstratum::_r2_xyzrpy_init_error_xyz_y_ << std::endl;
            std::cout << "_r2_xyzrpy_init_error_xyz_z_:             " << superstratum::_r2_xyzrpy_init_error_xyz_z_ << std::endl;
            std::cout << "_r2_xyzrpy_init_error_rpy_roll_:          " << superstratum::_r2_xyzrpy_init_error_rpy_roll_ << std::endl;
            std::cout << "_r2_xyzrpy_init_error_rpy_pitch_:         " << superstratum::_r2_xyzrpy_init_error_rpy_pitch_ << std::endl;
            std::cout << "_r2_xyzrpy_init_error_rpy_yaw_:           " << superstratum::_r2_xyzrpy_init_error_rpy_yaw_ << std::endl;

            std::cout << "\n--- 识别卷轴 ---" << std::endl;
            std::cout << "_coner_path_:                        " << superstratum::_coner_path_ << std::endl;
            std::cout << "_juanzhou_path_:                     " << superstratum::_juanzhou_path_ << std::endl;
            std::cout << "_box_num_:                           " << superstratum::_box_num_ << std::endl;

            std::cout << "\n--- 识别卷轴2 ---" << std::endl;
            std::cout << "_roi12_path_:                        " << superstratum::_roi12_path_ << std::endl;
            std::cout << "_limit_count_1_:                     " << superstratum::_limit_count_1_ << std::endl;
            std::cout << "_limit_count_2_:                     " << superstratum::_limit_count_2_ << std::endl;
            std::cout << "_limit_count_3_:                     " << superstratum::_limit_count_3_ << std::endl;
            std::cout << "_limit_count_4_:                     " << superstratum::_limit_count_4_ << std::endl;

            std::cout << "\n--- 串口 & 重定位 ---" << std::endl;
            std::cout << "_max_serial_num_:                    " << _max_serial_num_ << std::endl;
            std::cout << "_voxeldownsample_threshold_for_teaser_:   " << _voxeldownsample_threshold_for_teaser_ << std::endl;
            std::cout << "_voxeldownsample_threshold_for_icp_:      " << _voxeldownsample_threshold_for_icp_ << std::endl;
            std::cout << "_setmaxcorrespondencedistance_nano_gicp_:  " << _setmaxcorrespondencedistance_nano_gicp_ << std::endl;
            std::cout << "_min_num_of_point_cloud_for_relocation_:   " << _min_num_of_point_cloud_for_relocation_ << std::endl;
            std::cout << "_min_num_of_point_cloud_for_relocation2_:   " << _min_num_of_point_cloud_for_relocation2_ << std::endl;
            std::cout << "\n--- point_lio建图下采样深度 ---" << std::endl;
            std::cout << "_voxeldownsample_threshold_:   " << _voxeldownsample_threshold_ << std::endl;

            std::cout << "\n--- 相机内参 640x480 ---" << std::endl;
            std::cout << "fx_640:    " << Ten::camera_fx_640  << std::endl;
            std::cout << "fy_640:    " << Ten::camera_fy_640  << std::endl;
            std::cout << "cx_640:    " << Ten::camera_cx_640  << std::endl;
            std::cout << "cy_640:    " << Ten::camera_cy_640  << std::endl;

            std::cout << "\n--- 相机内参 1920x1080 ---" << std::endl;
            std::cout << "fx_1080:   " << Ten::camera_fx_1080 << std::endl;
            std::cout << "fy_1080:   " << Ten::camera_fy_1080 << std::endl;
            std::cout << "cx_1080:   " << Ten::camera_cx_1080 << std::endl;
            std::cout << "cy_1080:   " << Ten::camera_cy_1080 << std::endl;

            std::cout << "\n--- 雷达到相机外参变换矩阵 (4x4) ---" << std::endl;
            std::cout << Ten::superstratum::_lidar_to_camera_transform_matrix_ << std::endl;

            std::cout << "\n--- 雷达发布频率 ---" << std::endl;
            std::cout << "_laser_pub_hz_:   " << Ten::_laser_pub_hz_ << std::endl;

            std::cout << "\n--- ekf ---" << std::endl;
            std::cout << "_q_pos_:   " << Ten::_q_pos_ << std::endl;
            std::cout << "_q_att_:   " << Ten::_q_att_ << std::endl;
            std::cout << "_q_vel_:   " << Ten::_q_vel_ << std::endl;
            std::cout << "_q_ang_:   " << Ten::_q_ang_ << std::endl;

            std::cout << "\n--- 雷达自己的初始误差 ---" << std::endl;
            std::cout << "_lidar_xyzrpy_init_error_xyz_x_:             " << Ten::_lidar_xyzrpy_init_error_xyz_x_ << std::endl;
            std::cout << "_lidar_xyzrpy_init_error_xyz_y_:             " << Ten::_lidar_xyzrpy_init_error_xyz_y_ << std::endl;
            std::cout << "_lidar_xyzrpy_init_error_xyz_z_:             " << Ten::_lidar_xyzrpy_init_error_xyz_z_ << std::endl;
            std::cout << "_lidar_xyzrpy_init_error_rpy_roll_:          " << Ten::_lidar_xyzrpy_init_error_rpy_roll_ << std::endl;
            std::cout << "_lidar_xyzrpy_init_error_rpy_pitch_:         " << Ten::_lidar_xyzrpy_init_error_rpy_pitch_ << std::endl;
            std::cout << "_lidar_xyzrpy_init_error_rpy_yaw_:           " << Ten::_lidar_xyzrpy_init_error_rpy_yaw_ << std::endl;

            // std::cout << "\n--- camera_kfs ---" << std::endl;
            // std::cout << "_camera_x_bias_:   " << Ten::_camera_x_bias_ << std::endl;
            // std::cout << "_camera_y_bias_:   " << Ten::_camera_y_bias_ << std::endl;
            // std::cout << "_camera_z_bias_:   " << Ten::_camera_z_bias_ << std::endl;
            // std::cout << "_max_bias_:   " << Ten::_max_bias_ << std::endl;


            std::cout << "\n---  AprilTag 参数 ---" << std::endl;
            std::cout << "APRILTAG_FX:   " << APRILTAG_FX << std::endl;
            std::cout << "APRILTAG_FY:   " << APRILTAG_FY << std::endl;
            std::cout << "APRILTAG_CX:   " << APRILTAG_CX << std::endl;
            std::cout << "APRILTAG_CY:   " << APRILTAG_CY << std::endl;
            std::cout << "APRILTAG_K1:   " << APRILTAG_K1 << std::endl;
            std::cout << "APRILTAG_K2:   " << APRILTAG_K2 << std::endl;
            std::cout << "APRILTAG_P1:   " << APRILTAG_P1 << std::endl;
            std::cout << "APRILTAG_P2:   " << APRILTAG_P2 << std::endl;
            std::cout << "APRILTAG_K3:   " << APRILTAG_K3 << std::endl;
            std::cout << "APRILTAG_CAMERA_WIDTH:   " << APRILTAG_CAMERA_WIDTH << std::endl;
            std::cout << "APRILTAG_CAMERA_HEIGHT:   " << APRILTAG_CAMERA_HEIGHT << std::endl;
            std::cout << "APRILTAG_CAMERA_FPS:   " << APRILTAG_CAMERA_FPS << std::endl;
            std::cout << "APRILTAG_TAG_SIZE:   " << APRILTAG_TAG_SIZE << std::endl;
            std::cout << "APRILTAG_TAG_SPACING:   " << APRILTAG_TAG_SPACING << std::endl;
            std::cout << "APRILTAG_QUAD_DECIMATE:   " << APRILTAG_QUAD_DECIMATE << std::endl;
            std::cout << "APRILTAG_QUAD_SIGMA:   " << APRILTAG_QUAD_SIGMA << std::endl;
            std::cout << "APRILTAG_NTHREADS:   " << APRILTAG_NTHREADS << std::endl;
            std::cout << "APRILTAG_REFINE_EDGES:   " << APRILTAG_REFINE_EDGES << std::endl;
            std::cout << "APRILTAG_PRIMARY_ID:   " << APRILTAG_PRIMARY_ID << std::endl;
            std::cout << "APRILTAG_AUX_ID:   " << APRILTAG_AUX_ID << std::endl;
            std::cout << "APRILTAG_SINGLE_TAG_MODE:   " << APRILTAG_SINGLE_TAG_MODE << std::endl;
            std::cout << "APRILTAG_SINGLE_TAG_ID:   " << APRILTAG_SINGLE_TAG_ID << std::endl;
            std::cout << "APRILTAG_FUSION_BIAS_X:   " << APRILTAG_FUSION_BIAS_X << std::endl;
            std::cout << "APRILTAG_FUSION_BIAS_Y:   " << APRILTAG_FUSION_BIAS_Y << std::endl;
            std::cout << "APRILTAG_FUSION_BIAS_Z:   " << APRILTAG_FUSION_BIAS_Z << std::endl;
            std::cout << "APRILTAG_WORLD_BIAS_X:   " << APRILTAG_WORLD_BIAS_X << std::endl;
            std::cout << "APRILTAG_WORLD_BIAS_Y:   " << APRILTAG_WORLD_BIAS_Y << std::endl;
            std::cout << "APRILTAG_WORLD_BIAS_Z:   " << APRILTAG_WORLD_BIAS_Z << std::endl;
            std::cout << "APRILTAG_JUMP_WINDOW:   " << APRILTAG_JUMP_WINDOW << std::endl;
            std::cout << "APRILTAG_YAW_EMA_ALPHA:   " << APRILTAG_YAW_EMA_ALPHA << std::endl;
            std::cout << "APRILTAG_ENABLE_SAW_WINDOW:   " << APRILTAG_ENABLE_SAW_WINDOW << std::endl;
            std::cout << "APRILTAG_DRAW_FONT_SCALE:   " << APRILTAG_DRAW_FONT_SCALE << std::endl;
            std::cout << "APRILTAG_DRAW_THICKNESS:   " << APRILTAG_DRAW_THICKNESS << std::endl;
            std::cout << "APRILTAG_SAW_WINDOW_NAME:   " << APRILTAG_SAW_WINDOW_NAME << std::endl;

            std::cout << "\n---  usbcam 参数 ---" << std::endl;
            std::cout << "_usb_device_num1_:   " << _usb_device_num1_ << std::endl;
            std::cout << "_usb_device_num2_:   " << _usb_device_num2_ << std::endl;
            std::cout << "_usb_device_num3_:   " << _usb_device_num3_ << std::endl;

            std::cout << "\n--- imu ekf ---" << std::endl;
            std::cout << "_imu_q_pos_:   " << Ten::_imu_q_pos_ << std::endl;
            std::cout << "_imu_q_att_:   " << Ten::_imu_q_att_ << std::endl;
            std::cout << "_imu_q_vel_:   " << Ten::_imu_q_vel_ << std::endl;

            
            std::cout << "\n==========================================================\n" << std::endl;
        }

        
        private:

        };

    }

}


#endif