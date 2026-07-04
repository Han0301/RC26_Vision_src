#include "ros_debug.h"
#include <iostream>
#include <stdexcept>

namespace Ten
{
    namespace debug
    {
        // ===================== DebugImagePub 实现 =====================
        std::once_flag DebugImagePub::init_flag_;

        DebugImagePub::DebugImagePub()
        {
            std::cout << "[DebugImagePub] Instance initialized." << std::endl;
        }

        std::unique_ptr<DebugImagePub> DebugImagePub::create()
        {
            return std::unique_ptr<DebugImagePub>(new DebugImagePub());
        }

        DebugImagePub& DebugImagePub::GetInstance()
        {
            static std::unique_ptr<DebugImagePub> instance = nullptr;
            std::call_once(init_flag_, [&]()
            {
                instance = create();
            });
            return *instance;
        }

        void DebugImagePub::matToImage(const cv::Mat& mat, sensor_msgs::Image& ros_img)
        {
            ros_img.header.stamp = ros::Time::now();
            ros_img.height = mat.rows;
            ros_img.width = mat.cols;
            ros_img.step = mat.step;
            ros_img.data.resize(mat.total() * mat.elemSize());

            std::memcpy(ros_img.data.data(), mat.data, mat.total() * mat.elemSize());

            // 自动匹配OpenCV图像编码
            if (mat.type() == CV_8UC1)
                ros_img.encoding = "mono8";
            else if (mat.type() == CV_8UC3)
                ros_img.encoding = "bgr8";
            else if (mat.type() == CV_16UC1)
                ros_img.encoding = "mono16";
            else
                ros_img.encoding = "bgr8";
        }

        bool DebugImagePub::publish(const cv::Mat& mat, const std::string& topic_name)
        {
            std::lock_guard<std::mutex> lock(mtx_);

            if (topic_name.empty())
            {
                std::cerr << "[DebugImagePub] Error: empty topic name!" << std::endl;
                return false;
            }
            if (mat.empty())
            {
                std::cerr << "[DebugImagePub] Error: empty cv::Mat!" << std::endl;
                return false;
            }

            try
            {
                auto iter = pub_map_.find(topic_name);
                if (iter == pub_map_.end())
                {
                    ros::Publisher pub = nh_.advertise<sensor_msgs::Image>(topic_name, 10);
                    pub_map_.emplace(topic_name, pub);
                    std::cout << "[DebugImagePub] Create new publisher: " << topic_name << std::endl;
                }

                sensor_msgs::Image ros_img;
                matToImage(mat, ros_img);
                pub_map_[topic_name].publish(ros_img);
                return true;
            }
            catch (const std::exception& e)
            {
                std::cerr << "[DebugImagePub] Exception: " << e.what() << std::endl;
                return false;
            }
            catch (...)
            {
                std::cerr << "[DebugPointCloudPub] Unknown fatal exception!" << std::endl;
                return false;
            }
        }

        // ===================== DebugPointCloudPub 普通成员实现 =====================
        std::once_flag DebugPointCloudPub::init_flag_;

        DebugPointCloudPub::DebugPointCloudPub()
        {
            std::cout << "[DebugPointCloudPub] Instance initialized." << std::endl;
        }

        std::unique_ptr<DebugPointCloudPub> DebugPointCloudPub::create()
        {
            return std::unique_ptr<DebugPointCloudPub>(new DebugPointCloudPub());
        }

        DebugPointCloudPub& DebugPointCloudPub::GetInstance()
        {
            static std::unique_ptr<DebugPointCloudPub> instance = nullptr;
            std::call_once(init_flag_, [&]()
            {
                instance = create();
            });
            return *instance;
        }

    } // namespace debug
} // namespace Ten
