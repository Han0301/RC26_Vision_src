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

#include "./../method_math.h"
#include "./../parameter/parameter.h"

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
        // 初始化标定参考3D点
        // TODO:左边点 点位待优化
        points_3d_.emplace_back(4.4, -1.2, 0.4);
        points_3d_.emplace_back(4.4, -2.4, 0.4);
        points_3d_.emplace_back(3.2, -2.4, 0.4);
        points_3d_.emplace_back(4.4, -3.6, 0.4);
        points_3d_.emplace_back(4.4, -3.6, 0.6);
        points_3d_.emplace_back(4.4, -4.8, 0.6);
        points_3d_.emplace_back(5.6, -3.6, 0.6);
        points_3d_.emplace_back(5.6, -2.4, 0.6);
        points_3d_.emplace_back(5.6, -1.2, 0.2);
        // TODO:右边点 点位待优化
        // points_3d_.emplace_back(4.4, -4.8, 0.4);
        // points_3d_.emplace_back(4.4, -4.8, 0.6);
        // points_3d_.emplace_back(4.4, -3.6, 0.6);
        // points_3d_.emplace_back(5.6, -3.6, 0.6);
        // points_3d_.emplace_back(5.6, -4.8, 0.6);
        // points_3d_.emplace_back(6.8, -4.8, 0.4);
        // points_3d_.emplace_back(8.0, -4.8, 0.2);
        // points_3d_.emplace_back(6.8, -3.6, 0.6);
        // points_3d_.emplace_back(8.0, -3.6, 0.4);

        // 初始化相机内参矩阵(1080P)
        K_ = (cv::Mat_<double>(3, 3) << camera_fx_1080, 0, camera_cx_1080,
              0, camera_fy_1080, camera_cy_1080,
              0, 0, 1);

        // 畸变系数初始化为0
        dist_coeffs_ = cv::Mat::zeros(5, 1, CV_64F);

        // 显示参数初始化
        last_window_scale_ = 1.0;
        window_scale_ = 1.0;
        min_window_scale_ = 1.0;
        max_window_scale_ = 4.0;
        mouse_point_ = cv::Point2d(-1, -1);
        window_point_ = cv::Point2d(0, 0);
        getScreenResolution();

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
    Eigen::Matrix4d useFastCalibration(const cv::Mat &img, const Eigen::Matrix4d &world_to_lidar)
    {
        // 保存传入参数
        world_to_lidar_ = world_to_lidar;
        original_img_ = img.clone(); // 保存原始图像

        // 1. 校验输入图像
        if (original_img_.empty())
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

        // 计算可互动图片窗口的合适大小（屏幕0.8倍，保持原图长宽比）
        setShowWindowSize(original_img_.rows, original_img_.cols);
        std::cout << "屏幕分辨率：" << screen_width_ << "x" << screen_height_ << std::endl;
        std::cout << "可互动窗口大小：" << window_width_ << "x" << window_height_ << std::endl;
        std::cout << "原始图像大小：" << original_img_.cols << "x" << original_img_.rows << std::endl;

        // 计算初始缩放比例：将原图完整缩放到窗口大小
        double scale_x = static_cast<double>(window_width_) / original_img_.cols;
        double scale_y = static_cast<double>(window_height_) / original_img_.rows;
        initial_scale_ = std::min(scale_x, scale_y);
        window_scale_ = 1.0; // 相对于初始缩放的比例
        std::cout << "初始缩放比例：" << initial_scale_ << std::endl;

        // 缩放参考图像（实例图片相关不调整）
        cv::resize(example_img, example_img, cv::Size(int(screen_width_ * 0.4), int(screen_height_ * 0.4)));

        // 创建可互动图片窗口，设置为可调整大小并保持比例
        cv::namedWindow("img", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
        cv::resizeWindow("img", window_width_, window_height_);
        cv::moveWindow("img", 0, 0);

        // 创建参考图片窗口
        cv::namedWindow("example", cv::WINDOW_NORMAL);
        cv::moveWindow("example", 0, 0);

        // 3. 绑定鼠标回调函数
        cv::setMouseCallback("img", mouseCallback, this);

        // 4. 鼠标选点循环（采集9个标定点）
        collectMousePoints(example_img);

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
        std::cout << "雷达到相机变换矩阵：\n"
                  << lidar2camera_cv << std::endl;

        // 10. 写入配置文件（保留注释）
        // writeCalibResultToYaml(lidar2camera_cv);
        // std::cout << "标定完成，结果已写入配置文件！注释与格式均保留" << std::endl;

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

    /**
     * @brief 设置屏幕分辨率
     * @param cols 宽
     * @param rows 高
     */
    void setScreenResolution(int cols, int rows)
    {
        screen_width_ = cols;
        screen_height_ = rows;
    }

private:
    // ===================== 成员变量 =====================
    std::vector<cv::Point2d> points_2d_;
    std::vector<cv::Point3d> points_3d_;
    std::string example_img_path_;
    cv::Mat K_;
    cv::Mat dist_coeffs_;
    Eigen::Matrix4d world_to_lidar_;
    cv::Mat original_img_; // 保存原始图像
    int screen_width_;
    int screen_height_;
    int window_width_;     // 可互动窗口的宽度
    int window_height_;    // 可互动窗口的高度
    double initial_scale_; // 初始缩放比例：原图到窗口的比例
    double window_scale_;  // 当前缩放比例（相对于初始缩放）
    double last_window_scale_;
    double min_window_scale_;
    double max_window_scale_;
    bool scale_change = false;
    cv::Point2d mouse_point_;     // 滚轮事件发生时鼠标在窗口的坐标
    cv::Point2d now_mouse_point_; // 当前鼠标在窗口的位置
    cv::Point2d window_point_;    // 窗口左上角在"初始缩放后图像"中的坐标

    // ===================== 私有函数 =====================
    /**
     * @brief 获取屏幕实际分辨率
     */
    void getScreenResolution()
    {
        // 获取当前屏幕分辨率
        int width = 0, height = 0;
        Display *d = XOpenDisplay(NULL);
        if (d)
        {
            // 获取默认屏幕
            int screen_num = DefaultScreen(d);

            // 获取逻辑分辨率（考虑HiDPI缩放）
            width = DisplayWidth(d, screen_num);
            height = DisplayHeight(d, screen_num);

            // 关闭显示连接
            XCloseDisplay(d);
        }

        // 处理异常大或小值
        if (width > 0 && width <= 3840 && height > 0 && height <= 2160)
        {
            screen_width_ = width;
            screen_height_ = height;
        }
        else
        {
            screen_width_ = 1920;
            screen_height_ = 1080;
        }
    }

    /**
     * @brief 根据屏幕分辨率和原图尺寸计算合适的窗口大小
     *        窗口大小为屏幕的0.8倍，保持原图长宽比
     */
    void setShowWindowSize(int img_rows, int img_cols)
    {
        const double screen_ratio = 0.8;
        // 计算图片和屏幕的宽高比
        double img_aspect = static_cast<double>(img_cols) / img_rows;
        double screen_aspect = static_cast<double>(screen_width_) / screen_height_;

        // 根据宽高比决定以哪个维度为基准
        if (img_aspect > screen_aspect)
        {
            // 图片更宽，以宽度为基准
            window_width_ = static_cast<int>(screen_width_ * screen_ratio);
            window_height_ = static_cast<int>(window_width_ / img_aspect);
        }
        else
        {
            // 图片更高，以高度为基准
            window_height_ = static_cast<int>(screen_height_ * screen_ratio);
            window_width_ = static_cast<int>(window_height_ * img_aspect);
        }

        // 确保窗口尺寸至少为1
        window_width_ = std::max(1, window_width_);
        window_height_ = std::max(1, window_height_);
    }

    /**
     * @brief 鼠标选点：左键打点，右键撤销，ESC退出
     */
    void collectMousePoints(const cv::Mat &example)
    {
        while (points_2d_.size() < points_3d_.size())
        {
            cv::Mat display_img;
            display_img = calculateShowImg();

            // 绘制已标记的点
            for (size_t i = 0; i < points_2d_.size(); ++i)
            {
                // 将原图坐标转换为窗口坐标
                cv::Point p;
                p.x = static_cast<int>((points_2d_[i].x * initial_scale_ - window_point_.x) * window_scale_);
                p.y = static_cast<int>((points_2d_[i].y * initial_scale_ - window_point_.y) * window_scale_);

                // 只绘制在窗口内的点
                if (p.x >= 0 && p.x < window_width_ && p.y >= 0 && p.y < window_height_)
                {
                    cv::circle(display_img, p, 4, cv::Scalar(0, 0, 255), -1);
                    cv::putText(display_img, std::to_string(i + 1), p + cv::Point(5, 5),
                                cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);
                }
            }

            // 显示图像
            if (cv::getWindowProperty("img", cv::WindowPropertyFlags::WND_PROP_AUTOSIZE) == -1)
            {
                cv::namedWindow("img", cv::WINDOW_NORMAL | cv::WINDOW_KEEPRATIO);
                cv::resizeWindow("img", window_width_, window_height_);
                cv::moveWindow("img", 0, 0);
                cv::setMouseCallback("img", mouseCallback, this);
            }
            cv::imshow("img", display_img);
            cv::imshow("example", example);

            char key = static_cast<char>(cv::waitKey(1));
            if (key == 27) // ESC键退出
                break;
        }
    }

    /**
     * @brief 计算重投影误差
     */
    double calculateReprojectionError(const cv::Mat &rvec, const cv::Mat &tvec)
    {
        std::vector<cv::Point2d> proj_points;
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
    cv::Mat buildTransformMatrix(const cv::Mat &rvec, const cv::Mat &tvec)
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
    cv::Mat computeLidarToCamera(const cv::Mat &world2camera)
    {
        cv::Mat world2lidar_cv = cv::Mat::eye(4, 4, CV_64F);
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                world2lidar_cv.at<double>(i, j) = world_to_lidar_(i, j);

        cv::Mat lidar_to_camera = world2camera * world2lidar_cv.inv(cv::DECOMP_LU);

        return lidar_to_camera;
    }

    /**
     * @brief 计算当前需要显示的图像
     *        确保初始状态显示完整图片，缩放时以鼠标为中心
     */
    cv::Mat calculateShowImg()
    {
        // 初始缩放后的图像尺寸
        double scaled_width = original_img_.cols * initial_scale_;
        double scaled_height = original_img_.rows * initial_scale_;

        if (scale_change)
        {
            // 核心公式：保持鼠标指向的"初始缩放后图像"坐标不变
            // 鼠标在窗口中的坐标：(mouse_x, mouse_y)
            // 缩放前对应的初始缩放后坐标：(mouse_x/last_scale + window_x, mouse_y/last_scale + window_y)
            // 缩放后对应的初始缩放后坐标：(mouse_x/new_scale + new_window_x, mouse_y/new_scale + new_window_y)
            // 两者相等，解得：
            double new_window_x = mouse_point_.x / last_window_scale_ + window_point_.x - mouse_point_.x / window_scale_;
            double new_window_y = mouse_point_.y / last_window_scale_ + window_point_.y - mouse_point_.y / window_scale_;

            // 限制窗口左上角在合理范围内
            // 当window_scale_ <= 1.0时，图片应该居中显示
            if (window_scale_ <= 1.0)
            {
                new_window_x = (scaled_width - window_width_ / window_scale_) / 2.0;
                new_window_y = (scaled_height - window_height_ / window_scale_) / 2.0;
            }
            else
            {
                // 当window_scale_ > 1.0时，限制不超出图像边界
                new_window_x = std::max(0.0, std::min(scaled_width - window_width_ / window_scale_, new_window_x));
                new_window_y = std::max(0.0, std::min(scaled_height - window_height_ / window_scale_, new_window_y));
            }

            window_point_ = cv::Point2d(new_window_x, new_window_y);
            scale_change = false;
        }

        // 计算需要显示的"初始缩放后图像"区域
        cv::Rect2d roi;
        roi.x = window_point_.x;
        roi.y = window_point_.y;
        roi.width = static_cast<double>(window_width_) / window_scale_;
        roi.height = static_cast<double>(window_height_) / window_scale_;

        // 创建黑色背景
        cv::Mat display_img = cv::Mat::zeros(window_height_, window_width_, original_img_.type());

        // 计算ROI与初始缩放后图像的交集
        cv::Rect2d img_roi(0, 0, scaled_width, scaled_height);
        cv::Rect2d intersect_roi = roi & img_roi;

        if (intersect_roi.width > 0 && intersect_roi.height > 0)
        {
            // 计算在原始图像中的ROI
            cv::Rect original_roi;
            original_roi.x = static_cast<int>(intersect_roi.x / initial_scale_);
            original_roi.y = static_cast<int>(intersect_roi.y / initial_scale_);
            original_roi.width = static_cast<int>(intersect_roi.width / initial_scale_);
            original_roi.height = static_cast<int>(intersect_roi.height / initial_scale_);

            // 确保ROI不超出原始图像边界
            original_roi.x = std::max(0, original_roi.x);
            original_roi.y = std::max(0, original_roi.y);
            original_roi.width = std::min(original_img_.cols - original_roi.x, original_roi.width);
            original_roi.height = std::min(original_img_.rows - original_roi.y, original_roi.height);

            if (original_roi.width > 0 && original_roi.height > 0)
            {
                // 裁剪原始图像
                cv::Mat img_crop = original_img_(original_roi);

                // 缩放到需要的大小
                cv::Size target_size(
                    static_cast<int>(intersect_roi.width * window_scale_),
                    static_cast<int>(intersect_roi.height * window_scale_));
                cv::Mat img_scaled;
                cv::resize(img_crop, img_scaled, target_size, 0, 0, cv::INTER_LINEAR);

                // 计算在显示图像中的位置
                cv::Point target_pos(
                    static_cast<int>((intersect_roi.x - roi.x) * window_scale_),
                    static_cast<int>((intersect_roi.y - roi.y) * window_scale_));

                // 将缩放后的图像复制到显示图像中
                img_scaled.copyTo(display_img(cv::Rect(target_pos, target_size)));
            }
        }

        return display_img;
    }

    /**
     * @brief 静态鼠标回调函数
     */
    static void mouseCallback(int event, int x, int y, int flags, void *userdata)
    {
        FastCalibration *self = static_cast<FastCalibration *>(userdata);
        if (!self)
            return;

        // 更新当前鼠标位置
        if (event == cv::EVENT_MOUSEMOVE || event == cv::EVENT_LBUTTONDOWN ||
            event == cv::EVENT_RBUTTONDOWN)
        {
            self->now_mouse_point_ = cv::Point2d(static_cast<double>(x), static_cast<double>(y));
        }

        // 左键点击：添加标定点
        if (event == cv::EVENT_LBUTTONDOWN)
        {
            // 将窗口坐标转换为原始图像坐标
            double x_scaled = static_cast<double>(x) / self->window_scale_ + self->window_point_.x;
            double y_scaled = static_cast<double>(y) / self->window_scale_ + self->window_point_.y;
            double x_original = x_scaled / self->initial_scale_;
            double y_original = y_scaled / self->initial_scale_;

            // 确保点在原始图像范围内
            if (x_original >= 0 && x_original < self->original_img_.cols &&
                y_original >= 0 && y_original < self->original_img_.rows)
            {
                self->points_2d_.emplace_back(x_original, y_original);
                std::cout << std::setprecision(6) << "标记第" << self->points_2d_.size()
                          << "个点：" << x_original << ", " << y_original << std::endl;
            }
            else
            {
                std::cout << "警告：点击位置超出图像范围！" << std::endl;
            }
        }
        // 右键点击：撤销上一个点
        else if (event == cv::EVENT_RBUTTONDOWN && !self->points_2d_.empty())
        {
            cv::Point2d p = self->points_2d_.back();
            self->points_2d_.pop_back();
            std::cout << "撤回点：" << p.x << ", " << p.y << std::endl;
        }
        // 滚轮事件：缩放图像
        else if (event == cv::EVENT_MOUSEHWHEEL)
        {
            self->mouse_point_ = self->now_mouse_point_;
            self->last_window_scale_ = self->window_scale_;

            // 获取滚轮滚动方向
            int delta = cv::getMouseWheelDelta(flags);

            // 调整缩放因子（每次滚动改变0.1）
            double scale = self->window_scale_;
            if (delta > 0)
            {
                scale += 0.1; // 放大
            }
            else
            {
                scale -= 0.1; // 缩小
            }

            // 限制缩放因子范围
            self->window_scale_ = std::max(self->min_window_scale_,
                                           std::min(scale, self->max_window_scale_));
            self->scale_change = true;
        }
    }
};

// 示例
// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "fast_calibration_node");
//     ros::NodeHandle nh = ros::NodeHandle("~");

//     FastCalibration fc;
//     // fc.setScreenResolution(1000, 500);  // 可自主设置屏幕分辨率
//     Eigen::Vector3d tvec = Eigen::Vector3d(-0.458819, 0.075477, -0.00410471);
//     cv::Mat_<double> rvec = (cv::Mat_<double>(3, 1) << 0.00177115, 0.0217489, 0.133612);
//     Eigen::Matrix3d R_matrix = Ten::rvectoRotationMatrix(rvec);
//     Eigen::Matrix4d world_to_lidar = Ten::combineRotationAndTranslation(R_matrix, tvec);

//     cv::Mat img = cv::imread("/home/awwsome/RC/robocon_ws3/src/merge/src/calibration/img1.png");
//     if (img.empty())
//     {
//         std::cerr << "错误：无法加载标定图像！" << std::endl;
//         return -1;
//     }

//     fc.useFastCalibration(img, world_to_lidar);

//     return 0;
// }