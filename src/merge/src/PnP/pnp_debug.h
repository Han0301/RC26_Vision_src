#ifndef __PNP_DEBUG_H_
#define __PNP_DEBUG_H_

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "pnp_func.h"
#include "../camera.h"
#include <sensor_msgs/PointCloud2.h>

namespace Ten
{
namespace KFS
{

// 调试绘制配置参数结构体
struct DebugConfig
{
    bool enable_debug_window = true;
    double font_scale = 0.8;
    int line_thickness = 2;
    std::string window_name = "PnP_Debug";
    int win_width = 640;
    int win_height = 480;
};

// 调试绘制工具类
class DebugDrawer
{
public:
    DebugDrawer()
    {
        pcl_pub_ = nh.advertise<sensor_msgs::PointCloud2>(topic_name, 10);
    };

    // 👇 所有函数直接在类内实现，自动内联，无多重定义
    void init()
    {
        if (!config_.enable_debug_window) return;
        cv::namedWindow(config_.window_name, cv::WINDOW_NORMAL);
        cv::resizeWindow(config_.window_name, config_.win_width, config_.win_height);
    }

    void publish_pointcloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& pcl_cloud,
        std::string frame_id = "map"
    )
    {
        if (!pcl_pub_ || pcl_cloud->empty()) return;

        // 转ROS消息并发布
        sensor_msgs::PointCloud2 ros_cloud;
        pcl::toROSMsg(*pcl_cloud, ros_cloud);
        ros_cloud.header.frame_id = frame_id;
        ros_cloud.header.stamp = ros::Time::now();
        pcl_pub_.publish(ros_cloud);
    }

    void shutdown()
    {
        if (!config_.enable_debug_window) return;
        cv::destroyWindow(config_.window_name);
    }

    void draw(const cv::Mat& color, const kfsPnpOutput& pnp_out, const rs2_intrinsics& color_intr)
    {
        if (!config_.enable_debug_window || color.empty()) return;
        cv::Mat img = color.clone();

        try
        {
            if (!pnp_out.valid)
            {
                cv::putText(img, "STATUS: DETECTING...", cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, config_.font_scale, cv::Scalar(0, 0, 255), config_.line_thickness);
            }
            else
            {
                cv::rectangle(img, pnp_out.roi, cv::Scalar(0, 255, 255), config_.line_thickness);
                drawAllPlanes(img, pnp_out.planeClouds, color_intr);
                drawPoseAxis(img, pnp_out, color_intr);

                char buf[256];
                snprintf(buf, sizeof(buf), "X:%.2f Y:%.2f Z:%.2f", pnp_out.center.x(), pnp_out.center.y(), pnp_out.center.z());
                cv::putText(img, buf, cv::Point(20, 80), cv::FONT_HERSHEY_SIMPLEX, config_.font_scale, cv::Scalar(0, 255, 0), config_.line_thickness);
            }
            cv::imshow(config_.window_name, img);
        }
        catch (...)
        {
            cv::imshow(config_.window_name, img);
        }
    }

    void publishPnpDebugImage(
        const cv::Mat& color,
        const kfsPnpOutput& pnp_out,
        const rs2_intrinsics& color_intr,
        ros::Publisher& debug_pub,
        const DebugConfig& config_  // 替换成你真实的配置类名
    )
    {
        // 空图像直接返回
        if (color.empty() || !debug_pub)
            return;

        // —————————— 1:1 复刻原 draw 函数的图像处理逻辑 ——————————
        cv::Mat img = color.clone();
        try
        {
            if (!pnp_out.valid)
            {
                cv::putText(img, "STATUS: DETECTING...", cv::Point(20, 40),
                    cv::FONT_HERSHEY_SIMPLEX, config_.font_scale,
                    cv::Scalar(0, 0, 255), config_.line_thickness);
            }
            else
            {
                cv::rectangle(img, pnp_out.roi, cv::Scalar(0, 255, 255), config_.line_thickness);
                drawAllPlanes(img, pnp_out.planeClouds, color_intr);  // 原有绘制函数
                drawPoseAxis(img, pnp_out, color_intr);              // 原有绘制函数

                char buf[256];
                snprintf(buf, sizeof(buf), "X:%.2f Y:%.2f Z:%.2f",
                    pnp_out.center.x(), pnp_out.center.y(), pnp_out.center.z());
                cv::putText(img, buf, cv::Point(20, 80),
                    cv::FONT_HERSHEY_SIMPLEX, config_.font_scale,
                    cv::Scalar(0, 255, 0), config_.line_thickness);
            }
        }
        catch (...)
        {
            // 异常不崩溃
        }

        // —————————— 发布处理后的图像到ROS话题 ——————————
        cv_bridge::CvImage cv_msg;
        cv_msg.header.stamp = ros::Time::now();
        cv_msg.encoding = sensor_msgs::image_encodings::BGR8;
        cv_msg.image = img;

        sensor_msgs::ImagePtr msg = cv_msg.toImageMsg();
        debug_pub.publish(msg);
    }

private:
    ros::Publisher pcl_pub_;
    ros::NodeHandle nh;
    std::string topic_name = "/camera/pointcloud";

    void drawAllPlanes(cv::Mat& img, const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>& planes, const rs2_intrinsics& intr)
    {
        const cv::Scalar colors[] = {cv::Scalar(255,0,255), cv::Scalar(0,255,0), cv::Scalar(255,255,0)};
        for (size_t i = 0; i < planes.size() && i < 3; ++i)
        {
            drawSinglePlane(img, planes[i], colors[i], intr);
        }
    }

    void drawSinglePlane(cv::Mat& img, const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, const cv::Scalar& color, const rs2_intrinsics& intr)
    {
        if (!cloud || cloud->size() < 10) return;
        std::vector<cv::Point> pixels;

        for (const auto& p : cloud->points)
        {
            if (p.z <= 0.05f) continue;
            int u = cvRound(intr.fx * p.x / p.z + intr.ppx);
            int v = cvRound(intr.fy * p.y / p.z + intr.ppy);
            if (u >= 0 && u < img.cols && v >= 0 && v < img.rows)
            {
                pixels.emplace_back(u, v);
            }
        }

        if (pixels.size() < 5) return;
        std::vector<cv::Point> hull;
        cv::convexHull(pixels, hull);
        if (hull.size() >= 3)
        {
            cv::drawContours(img, std::vector<std::vector<cv::Point>>{hull}, 0, color, config_.line_thickness);
        }
    }

    void drawPoseAxis(cv::Mat& img, const kfsPnpOutput& out, const rs2_intrinsics& intr)
    {
        try
        {
            cv::Mat cameraMat = (cv::Mat_<double>(3,3) << intr.fx, 0, intr.ppx, 0, intr.fy, intr.ppy, 0, 0, 1);
            cv::Mat distCoeff = cv::Mat::zeros(5,1,CV_64F);

            Eigen::Matrix3f rotMat = out.orientation.toRotationMatrix();
            cv::Mat cvRot(3,3,CV_64F);
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 3; j++)
                    cvRot.at<double>(i,j) = rotMat(i,j);

            cv::Mat rvec, tvec;
            cv::Rodrigues(cvRot, rvec);
            tvec = (cv::Mat_<double>(3,1) << out.center.x(), out.center.y(), out.center.z());

            const float axis_len = 0.1f;
            cv::drawFrameAxes(img, cameraMat, distCoeff, rvec, tvec, axis_len, 3);

            cv::Mat axisX = (cv::Mat_<double>(3,1) << axis_len, 0, 0);
            cv::Mat axisY = (cv::Mat_<double>(3,1) << 0, axis_len, 0);
            cv::Mat axisZ = (cv::Mat_<double>(3,1) << 0, 0, axis_len);

            cv::Mat ptX = cvRot * axisX + tvec;
            cv::Mat ptY = cvRot * axisY + tvec;
            cv::Mat ptZ = cvRot * axisZ + tvec;

            auto project = [&](cv::Mat& pt3d) -> cv::Point2f
            {
                double x = pt3d.at<double>(0) / pt3d.at<double>(2);
                double y = pt3d.at<double>(1) / pt3d.at<double>(2);
                int u = cvRound(intr.fx * x + intr.ppx);
                int v = cvRound(intr.fy * y + intr.ppy);
                return {float(u), float(v)};
            };

            cv::Point2f px = project(ptX);
            cv::Point2f py = project(ptY);
            cv::Point2f pz = project(ptZ);

            cv::putText(img, "X", px, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,255), 2);
            cv::putText(img, "Y", py, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,0), 2);
            cv::putText(img, "Z", pz, cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(255,0,0), 2);
        }
        catch (...) {}
    }

    DebugConfig config_;
};

} // namespace KFS
} // namespace Ten

#endif