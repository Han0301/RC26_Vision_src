#ifndef __MOUSE_H_
#define __MOUSE_H_


#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include "./../method_math.h"
#include "./../usb_cam_multi.h"
#include "./../usb_cam.h"
#include "../parameter/parameter.h"
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <vector>
#include <string>
#include <opencv2/opencv.hpp>
#include <X11/Xlib.h> 

namespace Ten
{

    class PointCollector {
    public:
        explicit PointCollector(const cv::Mat& img) : original_img_(img.clone()) {
            // 缩放参数初始化
            last_window_scale_ = 1.0;
            window_scale_ = 1.0;
            min_window_scale_ = 1.0;  // 最小缩放比例，可改为0.5支持缩小
            max_window_scale_ = 4.0;  // 最大缩放比例
            scale_change_ = false;
            mouse_point_ = cv::Point2d(-1, -1);
            window_point_ = cv::Point2d(0, 0);

            // 计算窗口初始大小
            getScreenResolution();
            setShowWindowSize(original_img_.rows, original_img_.cols);

            // 初始缩放：让原图完整适配窗口
            double scale_x = static_cast<double>(window_width_) / original_img_.cols;
            double scale_y = static_cast<double>(window_height_) / original_img_.rows;
            initial_scale_ = std::min(scale_x, scale_y);
        }

        // 运行选点交互，返回true表示用户确认，false表示取消
        bool run() {
            if (original_img_.empty()) {
                std::cerr << "错误：输入图像为空" << std::endl;
                return false;
            }

            // 创建可缩放窗口
            cv::namedWindow("PointSelect", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
            cv::resizeWindow("PointSelect", window_width_, window_height_);
            cv::moveWindow("PointSelect", 0, 0);
            cv::setMouseCallback("PointSelect", mouseCallback, this);

            // 选点主循环
            while (true) {
                cv::Mat display_img = calculateShowImg();

                // 绘制已选点与序号
                for (size_t i = 0; i < points_.size(); ++i) {
                    // 原图坐标转换为窗口显示坐标
                    cv::Point p;
                    p.x = static_cast<int>((points_[i].x * initial_scale_ - window_point_.x) * window_scale_);
                    p.y = static_cast<int>((points_[i].y * initial_scale_ - window_point_.y) * window_scale_);

                    if (p.x >= 0 && p.x < window_width_ && p.y >= 0 && p.y < window_height_) {
                        cv::circle(display_img, p, 4, cv::Scalar(0, 0, 255), -1);
                        cv::putText(display_img, std::to_string(i + 1), p + cv::Point(5, 5),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
                    }
                }

                cv::imshow("PointSelect", display_img);
                char key = static_cast<char>(cv::waitKey(1));

                if (key == 13) { // Enter键：确认选点
                    cv::destroyWindow("PointSelect");
                    return true;
                }
                if (key == 27) { // ESC键：取消选点
                    points_.clear();
                    cv::destroyWindow("PointSelect");
                    return false;
                }
            }
        }

        // 获取采集到的点集
        const std::vector<cv::Point2f>& getPoints() const {
            return points_;
        }

    private:
        cv::Mat original_img_;
        std::vector<cv::Point2f> points_;

        int screen_width_{};
        int screen_height_{};
        int window_width_{};
        int window_height_{};

        double initial_scale_{};
        double window_scale_;
        double last_window_scale_;
        double min_window_scale_;
        double max_window_scale_;
        bool scale_change_;

        cv::Point2d mouse_point_;
        cv::Point2d now_mouse_point_;
        cv::Point2d window_point_;

        // 获取屏幕分辨率（Linux X11）
        void getScreenResolution() {
            int width = 0, height = 0;
            Display* d = XOpenDisplay(NULL);
            if (d) {
                int screen_num = DefaultScreen(d);
                width = DisplayWidth(d, screen_num);
                height = DisplayHeight(d, screen_num);
                XCloseDisplay(d);
            }

            // 异常兜底
            if (width > 0 && width <= 3840 && height > 0 && height <= 2160) {
                screen_width_ = width;
                screen_height_ = height;
            } else {
                screen_width_ = 1920;
                screen_height_ = 1080;
            }
        }

        // 计算适配屏幕的窗口大小（占屏幕80%，保持原图比例）
        void setShowWindowSize(int img_rows, int img_cols) {
            const double screen_ratio = 0.8;
            double img_aspect = static_cast<double>(img_cols) / img_rows;
            double screen_aspect = static_cast<double>(screen_width_) / screen_height_;

            if (img_aspect > screen_aspect) {
                window_width_ = static_cast<int>(screen_width_ * screen_ratio);
                window_height_ = static_cast<int>(window_width_ / img_aspect);
            } else {
                window_height_ = static_cast<int>(screen_height_ * screen_ratio);
                window_width_ = static_cast<int>(window_height_ * img_aspect);
            }

            window_width_ = std::max(1, window_width_);
            window_height_ = std::max(1, window_height_);
        }

        // 计算当前帧显示图像（缩放+裁剪+边界处理）
        cv::Mat calculateShowImg() {
            double scaled_width = original_img_.cols * initial_scale_;
            double scaled_height = original_img_.rows * initial_scale_;

            if (scale_change_) {
                // 以鼠标为中心缩放，保持指向的图像坐标不变
                double new_window_x = mouse_point_.x / last_window_scale_ + window_point_.x - mouse_point_.x / window_scale_;
                double new_window_y = mouse_point_.y / last_window_scale_ + window_point_.y - mouse_point_.y / window_scale_;

                // 缩小状态下居中显示
                if (window_scale_ <= 1.0) {
                    new_window_x = (scaled_width - window_width_ / window_scale_) / 2.0;
                    new_window_y = (scaled_height - window_height_ / window_scale_) / 2.0;
                } else {
                    // 放大状态下限制不超出图像边界
                    new_window_x = std::max(0.0, std::min(scaled_width - window_width_ / window_scale_, new_window_x));
                    new_window_y = std::max(0.0, std::min(scaled_height - window_height_ / window_scale_, new_window_y));
                }

                window_point_ = cv::Point2d(new_window_x, new_window_y);
                scale_change_ = false;
            }

            // 计算显示区域
            cv::Rect2d roi(window_point_.x, window_point_.y,
                        static_cast<double>(window_width_) / window_scale_,
                        static_cast<double>(window_height_) / window_scale_);

            cv::Mat display_img = cv::Mat::zeros(window_height_, window_width_, original_img_.type());
            cv::Rect2d img_roi(0, 0, scaled_width, scaled_height);
            cv::Rect2d intersect_roi = roi & img_roi;

            if (intersect_roi.width > 0 && intersect_roi.height > 0) {
                // 映射到原始图像的裁剪区域
                cv::Rect original_roi;
                original_roi.x = static_cast<int>(intersect_roi.x / initial_scale_);
                original_roi.y = static_cast<int>(intersect_roi.y / initial_scale_);
                original_roi.width = static_cast<int>(intersect_roi.width / initial_scale_);
                original_roi.height = static_cast<int>(intersect_roi.height / initial_scale_);

                // 边界校验
                original_roi.x = std::max(0, original_roi.x);
                original_roi.y = std::max(0, original_roi.y);
                original_roi.width = std::min(original_img_.cols - original_roi.x, original_roi.width);
                original_roi.height = std::min(original_img_.rows - original_roi.y, original_roi.height);

                if (original_roi.width > 0 && original_roi.height > 0) {
                    cv::Mat img_crop = original_img_(original_roi);
                    cv::Size target_size(
                        static_cast<int>(intersect_roi.width * window_scale_),
                        static_cast<int>(intersect_roi.height * window_scale_));
                    cv::Mat img_scaled;
                    cv::resize(img_crop, img_scaled, target_size, 0, 0, cv::INTER_LINEAR);

                    cv::Point target_pos(
                        static_cast<int>((intersect_roi.x - roi.x) * window_scale_),
                        static_cast<int>((intersect_roi.y - roi.y) * window_scale_));
                    img_scaled.copyTo(display_img(cv::Rect(target_pos, target_size)));
                }
            }
            return display_img;
        }

        // 静态鼠标回调
        static void mouseCallback(int event, int x, int y, int flags, void* userdata) {
            auto* self = static_cast<PointCollector*>(userdata);
            if (!self) return;

            // 更新实时鼠标位置
            if (event == cv::EVENT_MOUSEMOVE || event == cv::EVENT_LBUTTONDOWN || event == cv::EVENT_RBUTTONDOWN) {
                self->now_mouse_point_ = cv::Point2d(static_cast<double>(x), static_cast<double>(y));
            }

            // 左键：添加点
            if (event == cv::EVENT_LBUTTONDOWN) {
                // 窗口坐标转换为原始图像坐标
                double x_scaled = static_cast<double>(x) / self->window_scale_ + self->window_point_.x;
                double y_scaled = static_cast<double>(y) / self->window_scale_ + self->window_point_.y;
                float x_original = static_cast<float>(x_scaled / self->initial_scale_);
                float y_original = static_cast<float>(y_scaled / self->initial_scale_);

                if (x_original >= 0 && x_original < self->original_img_.cols &&
                    y_original >= 0 && y_original < self->original_img_.rows) {
                    self->points_.emplace_back(x_original, y_original);
                    std::cout << std::fixed << std::setprecision(3)
                            << "标记第" << self->points_.size() << "个点: "
                            << x_original << ", " << y_original << std::endl;
                } else {
                    std::cout << "警告：点击位置超出图像范围" << std::endl;
                }
            }
            // 右键：撤销上一个点
            else if (event == cv::EVENT_RBUTTONDOWN && !self->points_.empty()) {
                cv::Point2f p = self->points_.back();
                self->points_.pop_back();
                std::cout << "撤回点: " << p.x << ", " << p.y << std::endl;
            }
            // 滚轮：缩放图像（修正原代码的水平滚轮笔误）
            else if (event == cv::EVENT_MOUSEWHEEL) {
                self->mouse_point_ = self->now_mouse_point_;
                self->last_window_scale_ = self->window_scale_;

                int delta = cv::getMouseWheelDelta(flags);
                double scale = self->window_scale_;
                scale += (delta > 0) ? 0.1 : -0.1;

                self->window_scale_ = std::max(self->min_window_scale_,
                                            std::min(scale, self->max_window_scale_));
                self->scale_change_ = true;
            }
        }
    };


    class mouse
    {
    public:
        std::vector<cv::Point2f> calibration(int dev_idx = Ten::_usb_device_num3_)
        {
            Ten::Ten_usb_cam& usbcam = Ten::Ten_usb_cam::GetInstance(dev_idx,640,480,30);
            cv::Mat black_bgr(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
            int key = 0;
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                cv::Mat img = usbcam.camera_read();
                if(img.empty())
                {
                    img = black_bgr;
                }
                cv::imshow("debugfront", img);
                key = cv::waitKey(1000 / 30);
                if(key == 'q')
                {
                    return std::vector<cv::Point2f>();
                }
                else if(key == 's')
                {
                    break;
                }
            }

            cv::Mat img = usbcam.camera_read().clone();
            if (img.empty()) {
                std::cerr << "图像加载失败" << std::endl;
                return std::vector<cv::Point2f>();
            }

            std::vector<cv::Point2f> result_points = collectPointsToFile(img, std::string(ROOT_DIR) + std::string("src/nine/point.txt"));
            cv::destroyWindow("debugfront");
            return result_points;
        }

    private:

    /**
     * @brief 交互式图像选点，结果保存到txt文件并返回点集
     * @param input_img 输入图像
     * @param save_txt_path 输出txt文件路径，每行一个点，格式为 x y
     * @return 按点击顺序排列的点集，取消或失败时返回空vector
     */
    std::vector<cv::Point2f> collectPointsToFile(const cv::Mat& input_img, const std::string& save_txt_path) 
    {
        PointCollector collector(input_img);
        bool is_confirmed = collector.run();

        if (!is_confirmed) {
            std::cout << "选点已取消" << std::endl;
            return {};
        }

        const std::vector<cv::Point2f>& points = collector.getPoints();

        // 写入txt文件
        std::ofstream out_file(save_txt_path);
        if (!out_file.is_open()) {
            std::cerr << "错误：无法打开文件 " << save_txt_path << " 写入" << std::endl;
            return points;
        }

        out_file << std::fixed << std::setprecision(6);
        for (const auto& p : points) {
            out_file << p.x << " " << p.y << "\n";
        }
        out_file.close();

        std::cout << "选点完成，共 " << points.size() << " 个点，已保存到 " << save_txt_path << std::endl;
        return points;
    }


    };


}

#endif
