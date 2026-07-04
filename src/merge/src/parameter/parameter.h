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

    //camera_kfs
    extern double _camera_x_bias_;
    extern double _camera_y_bias_;
    extern double _camera_z_bias_;
    extern double _max_bias_;

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


             //camera_kfs
            Ten::_camera_x_bias_                                = config["_camera_x_bias_"].as<double>();
            Ten::_camera_y_bias_                                = config["_camera_y_bias_"].as<double>();
            Ten::_camera_z_bias_                                = config["_camera_z_bias_"].as<double>();
            Ten::_max_bias_                                     = config["_max_bias_"].as<double>();


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

            std::cout << "\n--- camera_kfs ---" << std::endl;
            std::cout << "_camera_x_bias_:   " << Ten::_camera_x_bias_ << std::endl;
            std::cout << "_camera_y_bias_:   " << Ten::_camera_y_bias_ << std::endl;
            std::cout << "_camera_z_bias_:   " << Ten::_camera_z_bias_ << std::endl;
            std::cout << "_max_bias_:   " << Ten::_max_bias_ << std::endl;

            std::cout << "\n==========================================================\n" << std::endl;
        }

        
        private:

        };

    }

}


#endif