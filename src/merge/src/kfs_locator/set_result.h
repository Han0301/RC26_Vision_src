#ifndef __SET_RESULT_H_
#define __SET_RESULT_H_
#include "set_plane.h"
#include "set_detect.h"
#include "set_filter.h"
#include "../camera.h"


namespace Ten::kfs_locator
{
struct result
{
    double bia_radian = 0.0;     // 目标面的法向量到预设面的法向量的角度
    double x = 0.0;         
    double y = 0.0;           
    double z = 0.0; 
};


class Ten_set_result
{
public:
    Ten_set_result(rs2_intrinsics color_intr)
    :   input_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        filter_cloud(new pcl::PointCloud<pcl::PointXYZ>),
        plane_cloud(new pcl::PointCloud<pcl::PointXYZ>)
    {
        color_intr_ = color_intr;
    }

    // 执行流程 ---------------------------------------------------------------
    // 1 设置初始点云列表
    bool preprocess(Ten::camera_frame& frame);

    // 2 get_input_clouds()

    // 3 核心提取面函数
    bool postprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr = nullptr);

    // 4 设置结果
    result set_result(Ten::XYZRPY bias = Ten::XYZRPY());

    // 执行流程 ---------------------------------------------------------------

    // 取接口
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> get_input_clouds()
    {
        return  input_clouds;
    }
    const pcl::PointCloud<pcl::PointXYZ>::Ptr get_plane_cloud()
    {
        return  plane_cloud;
    }
    const Ten::kfs_locator::Plane_Info get_plane_info() 
    {
        return plane_info;
    }
    const std::string get_state() 
    {
        return state;
    }
    const std::vector<cv::Rect> get_rois()
    {
        return rois_;
    }
    const pcl::PointCloud<pcl::PointXYZ>::Ptr get_filter_cloud()
    {
        return filter_cloud;
    }

    // ── 滤波器控制 ──
    enum FilterMode { OFF = 0, MAD_EMA = 1, KALMAN = 2 };

    void set_filter_mode(FilterMode mode) { filter_mode_ = mode; }

private:
    // 参数
    rs2_intrinsics color_intr_;
    // 处理器
    Ten::kfs_locator::Ten_set_detect SET_DETECT_;
    Ten::kfs_locator::Ten_set_plane SET_PLANE_;
    // 中间变量
    Ten::kfs_locator::Plane_Info plane_info;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> input_clouds;
    std::vector<cv::Rect> rois_;                // 存储最新一次检测的 ROI 框
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filter_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud;
    Eigen::Vector3d key_center;     // 体中心点

    // 滤波器
    FilterMode filter_mode_ = MAD_EMA;  // 默认启用 MAD+EMA
    MadEmaFilter fx_, fy_, fz_, fa_;
    KalmanFilter1D kx_, ky_, kz_, ka_;

    std::string state = "off";

    // 功能函数 计算偏差角度， 确保plane_info已被写入信息
    double calc_deviation_angle(const Eigen::Vector3d& target_plane);
};      // class Ten_set_result

inline bool Ten_set_result::preprocess(Ten::camera_frame& frame)
{
    if (frame.bgr_image.empty() || frame.depth_image.empty())
    {
        state = "preprocess: frame.bgr_image.empty() || frame.depth_image.empty()";
        return false;
    }

    // 设置点云
    rois_ = SET_DETECT_.set_roi_detect(frame.bgr_image);
    bool is_set = SET_DETECT_.set_pcl_clouds(frame.depth_image, color_intr_, rois_, input_clouds);
    if (!is_set) 
    {
        state = "preprocess: set_pcl_clouds error!";
        return false;
    }
    state = "preprocess: ok";
    return true;
}

inline bool Ten_set_result::postprocess(const pcl::PointCloud<pcl::PointXYZ>::Ptr& input_cloud_ptr)
{
    // 选择输入点云
    if (input_cloud_ptr)
    {
        input_cloud = input_cloud_ptr;
    }
    else
    {
        if (input_clouds.empty())
        {
            state = "postprocess: input_clouds is empty";
            return false;
        }

        const auto& front_cloud = input_clouds.front();
        if (!front_cloud)
        {
            state = "postprocess: input_clouds front cloud is nullptr";
            return false;
        }
        input_cloud = front_cloud;
    }

    // 滤波器
    bool is_filtted = SET_PLANE_.cloud_filter(input_cloud,filter_cloud);      // 设置点云
    if (!is_filtted)
    {
        state = "postprocess: cloud_filter error!";
        return false;
    }

    // 提取平面和中心点，法向量
    bool is_plane_flitted = SET_PLANE_.Plane_fitter(filter_cloud, plane_cloud, plane_info);
    if (!is_plane_flitted)
    {
        state = "postprocess: Plane_fitter error!";
        return false;
    }

    // 方形拟合
    bool is_center_flitted = SET_PLANE_.compute_Center(plane_cloud,plane_info);
    if (!is_center_flitted)
    {
        state = "postprocess: compute_Center error!";
        return false;
    }
    // 赋值体中心点
    key_center = SET_PLANE_.cal_base_point(plane_info);
    state = "postprocess: ok";
    return true;
}

inline double Ten_set_result::calc_deviation_angle(const Eigen::Vector3d& target_plane)
{
    Eigen::Vector3d normal = plane_info.plane_normal;
    normal.normalize();

    // 计算向量点积
    double dot_product = normal.dot(target_plane);
    // 修复浮点误差，限制在 [-1, 1] 避免 acos 出现 NaN
    dot_product = std::max(-1.0, std::min(1.0, dot_product));

    // 原始夹角（弧度，范围 0 ~ M_PI）
    double rad_angle = std::acos(dot_product);
    // 1. 取最小夹角，约束到 0 ~ M_PI/2（对应 0° ~ 90°）
    rad_angle = std::min(rad_angle, M_PI - rad_angle);

    // 2. 大于 45°(M_PI/4) 则使用 90°(M_PI/2) - 当前弧度
    if (rad_angle > M_PI / 4)
    {
        rad_angle = M_PI / 2 - rad_angle;
    }

    return rad_angle;
}

inline result Ten_set_result::set_result(Ten::XYZRPY bias)
{
    result out;
    out.x = key_center.x();
    out.y = key_center.y();
    out.z = key_center.z();
    // 误差坐标系绕 X 轴 顺时针旋转 pitch → 目标坐标系
    // 顺时针 = 右手系负方向，sin 项取反
    const double c = std::cos(bias._rpy._pitch);
    const double s = std::sin(bias._rpy._pitch);

    // 校正后的光轴：UnitZ(0,0,1) 绕 X 轴顺时针旋转 pitch
    out.bia_radian = calc_deviation_angle(Eigen::Vector3d(0.0, s, c));

    const double y_rot = out.y * c + out.z * s;   // y' = y·cos + z·sin
    const double z_rot = -out.y * s + out.z * c;  // z' = -y·sin + z·cos

    // 平移偏差 + 旋转后的坐标
    out.x = out.x + bias._xyz._x;
    out.y = y_rot + bias._xyz._y;
    out.z = z_rot + bias._xyz._z;
    out.bia_radian = out.bia_radian + bias._rpy._yaw;

    // ── 输出滤波 ──
    if (filter_mode_ == MAD_EMA) {
        out.x          = fx_.filter(out.x);
        out.y          = fy_.filter(out.y);
        out.z          = fz_.filter(out.z);
        out.bia_radian = fa_.filter(out.bia_radian);
    } else if (filter_mode_ == KALMAN) {
        out.x          = kx_.filter(out.x);
        out.y          = ky_.filter(out.y);
        out.z          = kz_.filter(out.z);
        out.bia_radian = ka_.filter(out.bia_radian);
    }

    return out;
}

// void test()
// {
//     Ten::Ten_camera& _CAMERA_ = Ten::Ten_camera::GetInstance();
//     _CAMERA_.reset_camera_depth(640, 480,30);
//     rs2_intrinsics color_intr = _CAMERA_.get_color_intrinsics();    // 彩色内参 → 绘图用
//     Ten::kfs_locator::Ten_set_result plane_fiter(color_intr);
//     Ten::XYZRPY bias;

//     while (ros::ok())
//     {
//         Ten::camera_frame frame = _CAMERA_.camera_read_depth();
//         // 前处理
//         bool is_pre_ok = plane_fiter.preprocess(frame);
//         std::cout << "state: " <<  plane_fiter.get_state() << std::endl;
//         if (is_pre_ok)
//         {
//             // 后处理
//             bool is_post_ok = plane_fiter.postprocess();        // 置空输入点云，使用点云列表中的第一个点云
//             std::cout << "state: " <<  plane_fiter.get_state() << std::endl;
//             if (is_post_ok)
//             {
//                 // 设置结果
//                 Ten::kfs_locator::result out = plane_fiter.set_result(bias);
//                 std::cout << "angle: " <<  ((out.bia_radian) * 180.0 / M_PI) << std::endl;
//                 std::cout << "res.x: " <<  out.x << std::endl;
//                 std::cout << "res.y: " <<  out.y << std::endl;
//                 std::cout << "res.z: " <<  out.z << std::endl;
//             }
//         }
//     }
// }
} // namespace Ten::kfs_locator
#endif