#ifndef __FAST_CAMERA_CALIBRATION_H_
#define __FAST_CAMERA_CALIBRATION_H_

#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <algorithm>
#include <cstring>

// ROS
#include "ros/ros.h"

// OpenCV
#include "opencv2/opencv.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// X11
#include <X11/Xlib.h>
#include <yaml-cpp/yaml.h> 
#include <regex>  


// 自定义头文件
#include "./../threadpool.h"
#include "../parameter/parameter.h"

namespace Ten
{
    namespace camera_calibration
    {
    /**
     * @brief 相机快速标定类：通过鼠标选点实现3D-2D标定，计算雷达到相机外参并写入配置文件
     */
    class FastCalibration
    {
    public:
        /**
         * @brief 构造函数：初始化标定所需的3D点、相机内参、显示参数
         */
        FastCalibration()
        {

            // points_3d_.emplace_back(4.4, -1.2, 0.4);
            // points_3d_.emplace_back(4.4, -2.4, 0.4);
            // points_3d_.emplace_back(3.2, -2.4, 0.4);
            // points_3d_.emplace_back(4.4, -3.6, 0.4);
            // points_3d_.emplace_back(4.4, -3.6, 0.6);
            // points_3d_.emplace_back(4.4, -4.8, 0.6);
            // points_3d_.emplace_back(5.6, -3.6, 0.6);
            // points_3d_.emplace_back(5.6, -2.4, 0.6);
            // points_3d_.emplace_back(5.6, -1.2, 0.2);
            // 初始化标定参考3D点
            points_3d_.emplace_back(4.4, -4.8, 0.4);
            points_3d_.emplace_back(4.4, -4.8, 0.6);
            points_3d_.emplace_back(4.4, -3.6, 0.6);
            points_3d_.emplace_back(5.6, -3.6, 0.6);
            points_3d_.emplace_back(5.6, -4.8, 0.6);
            points_3d_.emplace_back(6.8, -4.8, 0.4);
            points_3d_.emplace_back(8.0, -4.8, 0.2);
            points_3d_.emplace_back(6.8, -3.6, 0.6);
            points_3d_.emplace_back(8.0, -3.6, 0.4);

            // 初始化相机内参矩阵(1080P)
            K_ = (cv::Mat_<double>(3, 3) << camera_fx_1080, 0, camera_cx_1080,
                                            0, camera_fy_1080, camera_cy_1080,
                                            0, 0, 1);

            // 畸变系数初始化为0
            dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

            // 显示参数初始化
            window_scale_ = 1.0;
            mouse_point_ = cv::Point2d(-1, -1);
            screen_width_ = 1920;
            screen_height_ = 1080;
            // 参考图像路径
            example_img_path_ = std::string(ROOT_DIR) + std::string("src/calibration/example.png");
            std::cout << "example_img_path_ : " << example_img_path_ << std::endl;
        }

        /**
         * @brief 执行快速标定主函数
         * @param img 标定图像（直接传入cv::Mat）
         * @param world_to_lidar 世界到雷达的变换矩阵(Eigen格式)
         * @return 成功返回Eigen格式雷达到相机矩阵，失败返回Eigen::Matrix4d()
         */
        Eigen::Matrix4d useFastCalibration(const cv::Mat& img, const Eigen::Matrix4d& world_to_lidar)
        {
            // 保存传入参数
            world_to_lidar_ = world_to_lidar;

            // 1. 校验输入图像
            if (img.empty())
            {
                std::cerr << "错误：输入的标定图像为空！" << std::endl;
                return Eigen::Matrix4d();
            }

            // 加载参考标定图像
            cv::Mat example_img = cv::imread(example_img_path_);
            if (example_img.empty())
            {
                std::cerr << "错误：无法加载参考标定图像！" << std::endl;
                return Eigen::Matrix4d();
            }

            // 缩放参考图像
            cv::resize(example_img, example_img, cv::Size(screen_width_ / 4, screen_height_ / 4));

            // 2. 获取屏幕分辨率并创建窗口
            getScreenResolution();
            // 支持自由缩放的窗口设置
            cv::namedWindow("img", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            cv::moveWindow("img", 0, 0);
            cv::resizeWindow("img", 1280, 720);

            cv::namedWindow("example", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            cv::moveWindow("example", 1300, 0);
            cv::resizeWindow("example", 480, 270);

            // 3. 绑定鼠标回调函数
            cv::setMouseCallback("img", mouseCallback, this);

            // 4. 鼠标选点循环（采集9个标定点）
            collectMousePoints(img, example_img);

            // 5. 校验点数量
            if (points_2d_.size() != points_3d_.size())
            {
                std::cout << "错误：2D点数量与3D点数量不匹配！需要 " << points_3d_.size() << " 个点" << std::endl;
                cv::destroyAllWindows();
                points_2d_.clear();
                return Eigen::Matrix4d();
            }

            // 6. 执行PnP求解相机外参
            cv::Mat rvec, tvec;
            cv::solvePnP(points_3d_, points_2d_, K_, dist_coeffs_, rvec, tvec, false, cv::SOLVEPNP_ITERATIVE);

            // 7. 计算重投影误差
            double reproj_error = calculateReprojectionError(rvec, tvec);
            std::cout << "平均重投影误差：" << reproj_error << std::endl;

            // 8. 构建世界到相机的变换矩阵
            cv::Mat world2camera = buildTransformMatrix(rvec, tvec);

            // 9. 计算雷达到相机外参 (OpenCV格式)
            cv::Mat lidar2camera_cv = computeLidarToCamera(world2camera);
            std::cout << std::fixed << std::setprecision(6);
            std::cout << "雷达到相机变换矩阵：\n" << lidar2camera_cv << std::endl;

            // 10. 写入配置文件（保留注释）
            writeCalibResultToYaml(lidar2camera_cv);
            std::cout << "标定完成，结果已写入配置文件！注释与格式均保留" << std::endl;

            // 11. OpenCV矩阵 转换为 Eigen矩阵
            Eigen::Matrix4d lidar2camera_eigen;
            for (int i = 0; i < 4; ++i)
            {
                for (int j = 0; j < 4; ++j)
                {
                    lidar2camera_eigen(i, j) = lidar2camera_cv.at<double>(i, j);
                }
            }

            // 释放资源
            cv::destroyAllWindows();
            points_2d_.clear();

            // 12. 成功返回Eigen格式矩阵
            return lidar2camera_eigen;
        }

    private:
        // ===================== 成员变量 =====================
        std::vector<cv::Point2f> points_2d_;
        std::vector<cv::Point2f> points_inv_;
        double window_scale_;
        cv::Point2d mouse_point_;
        std::string example_img_path_;
        std::vector<cv::Point3f> points_3d_;
        cv::Mat K_;
        Eigen::Matrix4d world_to_lidar_;
        cv::Mat dist_coeffs_;
        int screen_width_;
        int screen_height_;

        // ===================== 私有函数 =====================
        /**
         * @brief 获取屏幕实际分辨率
         */
        void getScreenResolution()
        {
            Display* d = XOpenDisplay(NULL);
            if (d)
            {
                Screen* s = DefaultScreenOfDisplay(d);
                screen_width_ = s->width;
                screen_height_ = s->height;
                XCloseDisplay(d);
            }
        }

        /**
         * @brief 鼠标选点：左键打点，右键撤销，ESC退出
         */
        void collectMousePoints(const cv::Mat& img, const cv::Mat& example)
        {
            while (points_2d_.size() < 9 && Ten::_TREADPOOL_FLAG_.read_flag())
            {
                cv::Mat display_img;
                img.copyTo(display_img);

                // 绘制已标记的点
                for (size_t i = 0; i < points_2d_.size(); ++i)
                {
                    cv::Point p = points_2d_[i] * window_scale_;
                    cv::circle(display_img, p, 4, cv::Scalar(0, 0, 255), -1);
                    cv::putText(display_img, std::to_string(i + 1), p + cv::Point(5, 5),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                }

                // 显示 + 支持缩放
                cv::imshow("img", display_img);
                cv::imshow("example", example);

                char key = static_cast<char>(cv::waitKey(1));
                if (key == 27) break;
            }
        }

        /**
         * @brief 计算重投影误差
         */
        double calculateReprojectionError(const cv::Mat& rvec, const cv::Mat& tvec)
        {
            std::vector<cv::Point2f> proj_points;
            cv::projectPoints(points_3d_, rvec, tvec, K_, dist_coeffs_, proj_points);

            double total_error = 0.0;
            for (size_t i = 0; i < proj_points.size(); ++i)
            {
                total_error += cv::norm(proj_points[i] - points_2d_[i]);
            }
            return total_error / proj_points.size();
        }

        /**
         * @brief 构建4x4变换矩阵
         */
        cv::Mat buildTransformMatrix(const cv::Mat& rvec, const cv::Mat& tvec)
        {
            cv::Mat R;
            cv::Rodrigues(rvec, R);
            cv::Mat T = cv::Mat::eye(4, 4, CV_64F);
            R.copyTo(T(cv::Rect(0, 0, 3, 3)));
            tvec.copyTo(T(cv::Rect(3, 0, 1, 3)));
            return T;
        }

        /**
         * @brief 计算雷达到相机外参
         */
        cv::Mat computeLidarToCamera(const cv::Mat& world2camera)
        {
            cv::Mat world2lidar_cv = cv::Mat::eye(4, 4, CV_64F);
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    world2lidar_cv.at<double>(i, j) = world_to_lidar_(i, j);

            return world2camera * world2lidar_cv.inv(cv::DECOMP_LU);
        }

        /**
         * @brief 文本替换写入YAML，100%保留注释、空行、格式
         */
        void writeCalibResultToYaml(const cv::Mat& T)
        {
            const std::string yaml_path = std::string(ROOT_DIR) + "src/parameter/config.yaml";
            
            std::ifstream ifs(yaml_path);
            if (!ifs.is_open()) {
                std::cerr << "打开YAML文件失败！" << std::endl;
                return;
            }

            // 读取全部内容
            std::string content((std::istreambuf_iterator<char>(ifs)), std::istreambuf_iterator<char>());
            ifs.close();

            // 矩阵16个参数名称
            const char* keys[16] = {
                "lidar_to_camera_00", "lidar_to_camera_01", "lidar_to_camera_02", "lidar_to_camera_03",
                "lidar_to_camera_10", "lidar_to_camera_11", "lidar_to_camera_12", "lidar_to_camera_13",
                "lidar_to_camera_20", "lidar_to_camera_21", "lidar_to_camera_22", "lidar_to_camera_23",
                "lidar_to_camera_30", "lidar_to_camera_31", "lidar_to_camera_32", "lidar_to_camera_33"
            };

            // 逐行替换
            for (int i = 0; i < 4; ++i) {
                for (int j = 0; j < 4; ++j) {
                    int idx = i * 4 + j;
                    double val = T.at<double>(i, j);
                    char pattern[128], replace[128];
                    snprintf(pattern, sizeof(pattern), "%s:\\s*[-+]?[0-9]*\\.?[0-9]+", keys[idx]);
                    snprintf(replace, sizeof(replace), "%s: %.15f", keys[idx], val);

                    // 正则替换
                    std::regex reg(pattern);
                    content = std::regex_replace(content, reg, replace);
                }
            }

            // 保存回文件
            std::ofstream ofs(yaml_path);
            ofs << content;
            ofs.close();
            std::cout << "✅ YAML写入成功：注释、空行、格式全部保留！" << std::endl;
        }

        /**
         * @brief 静态鼠标回调
         */
        static void mouseCallback(int event, int x, int y, int flags, void* userdata)
        {
            FastCalibration* self = static_cast<FastCalibration*>(userdata);
            if (!self) return;

            float x_fix = x / self->window_scale_;
            float y_fix = y / self->window_scale_;

            if (event == cv::EVENT_LBUTTONDOWN)
            {
                self->points_2d_.emplace_back(x_fix, y_fix);
                std::cout << "标记第" << self->points_2d_.size() << "个点：" << x_fix << ", " << y_fix << std::endl;
            }
            else if (event == cv::EVENT_RBUTTONDOWN && !self->points_2d_.empty())
            {
                cv::Point2f p = self->points_2d_.back();
                self->points_2d_.pop_back();
                std::cout << "撤回点：" << p << std::endl;
            }
        }
    };

    } 
} 

#endif
