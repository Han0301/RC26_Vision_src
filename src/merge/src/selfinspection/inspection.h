#ifndef __INSPECTION_H_
#define __INSPECTION_H_


#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include "./../method_math.h"
#include "./../usb_cam_multi.h"
#include "../parameter/parameter.h"
#include <cmath>


namespace Ten
{
    class inspection
    {
    public:
        void cameraside()
        {
            try
            {
                Ten::parameter::loadyaml::resetUsbDeviceParams();
                std::vector<int> idxs = {Ten::_usb_device_num1_, Ten::_usb_device_num2_};
                Ten::Ten_usb_cam_multi& usbcam = Ten::Ten_usb_cam_multi::GetInstance(idxs,640,480,30);
                Ten::r1_recognition::kfs_detect global_kfs_detector;
                while(Ten::_CAMERA_CLOSE_FLAG_.read_flag()&&Ten::_TREADPOOL_FLAG_.read_flag())
                {
                    cv::Mat img_left = usbcam.camera_read(Ten::_usb_device_num1_);
                    drawRectWithAreaText(img_left, global_kfs_detector.set_roi_detect(img_left));
                    cv::Mat img_right = usbcam.camera_read(Ten::_usb_device_num2_);
                    drawRectWithAreaText(img_right, global_kfs_detector.set_roi_detect(img_right));
                    cv::Mat debug = concatAndLabel(img_left, img_right);
                    cv::imshow("debugrl", debug);
                    cv::waitKey(30);
                }
                cv::destroyWindow("debugrl");
            }
            catch(const std::exception& e)
            {
                std::cerr << e.what() << '\n';
            }
            
        }

    private:

    /**
     * @brief 在图像上绘制矩形框，并在框中心标注矩形像素面积
     * @param img 待绘制的图像（会原地修改）
     * @param rect 需要绘制的矩形
     * @param boxColor 矩形框颜色，默认红色
     * @param textColor 文字颜色，默认白色
     * @param thickness 线条粗细，默认2
     * @param fontScale 文字缩放大小，默认0.6
     */
    void drawRectWithAreaText(cv::Mat& img,
                            const cv::Rect& rect,
                            const cv::Scalar& boxColor = cv::Scalar(0, 0, 0),
                            const cv::Scalar& textColor = cv::Scalar(255, 255, 255),
                            int thickness = 2,
                            double fontScale = 0.6)
    {
        // 1. 校验矩形是否有效（宽高大于0，且在图像范围内）
        if (rect.width <= 0 || rect.height <= 0)
        {
            cv::putText(img, "Invalid Rect", cv::Point(20, 30),
                        cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 255), 1);
            return;
        }

        // 2. 绘制矩形边框
        cv::rectangle(img, rect, boxColor, thickness);

        // 3. 计算矩形像素面积
        int area = rect.width * rect.height;
        std::string areaText = "Area:" + std::to_string(area);

        // 4. 计算矩形中心点
        cv::Point centerPt;
        centerPt.x = rect.x + rect.width / 2;
        centerPt.y = rect.y + rect.height / 2;

        // 5. 获取文字尺寸，用于居中对齐文字
        int baseLine = 0;
        cv::Size textSize = cv::getTextSize(areaText,
                                            cv::FONT_HERSHEY_SIMPLEX,
                                            fontScale,
                                            thickness,
                                            &baseLine);

        // 文字左下角坐标（让文字严格居中在矩形中心）
        cv::Point textOrigin;
        textOrigin.x = centerPt.x - textSize.width / 2;
        textOrigin.y = centerPt.y + textSize.height / 2;

        // 边界保护：防止文字超出图像被截断
        textOrigin.x = std::max(0, std::min(textOrigin.x, img.cols - textSize.width));
        textOrigin.y = std::max(textSize.height, std::min(textOrigin.y, img.rows - baseLine));

        // 6. 先绘制黑色文字描边，增强可读性
        cv::putText(img, areaText, textOrigin,
                    cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(0, 0, 0), thickness + 1);
        // 绘制白色主文字
        cv::putText(img, areaText, textOrigin,
                    cv::FONT_HERSHEY_SIMPLEX, fontScale, textColor, thickness);
    }


    /**
     * @brief 核心图像处理函数：两张图水平拼接→缩放至640x480→左右添加标注文字
     * @param img_left  左侧输入图像
     * @param img_right 右侧输入图像
     * @param text_left 左侧标注文字
     * @param text_right 右侧标注文字
     * @param output_size 最终输出尺寸（宽, 高），默认640x480
     * @return 处理后的图像
     */
    cv::Mat concatAndLabel(cv::Mat& img_left, cv::Mat& img_right,
                        const std::string& text_left = "left",
                        const std::string& text_right = "right",
                        cv::Size output_size = cv::Size(640, 480))
    {
        // 1. 空图替换为纯黑640*480
        cv::Mat left_fixed, right_fixed;
        bool left_empty = img_left.empty();
        bool right_empty = img_right.empty();

        if (left_empty)
        {
            left_fixed = cv::Mat::zeros(480, 640, CV_8UC3);
        }
        else
        {
            cv::resize(img_left, left_fixed, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
        }

        if (right_empty)
        {
            right_fixed = cv::Mat::zeros(480, 640, CV_8UC3);
        }
        else
        {
            cv::resize(img_right, right_fixed, cv::Size(640, 480), 0, 0, cv::INTER_AREA);
        }

        // 2. 水平拼接为 1280x480 大图
        cv::Mat concat_img;
        cv::hconcat(left_fixed, right_fixed, concat_img);

        // 3. 缩放回目标尺寸 640x480
        cv::Mat resized_img;
        cv::resize(concat_img, resized_img, output_size, 0, 0, cv::INTER_AREA);

        // 4. 绘制文字：全部红色，无描边
        const int font_face = cv::FONT_HERSHEY_SIMPLEX;
        const double font_scale = 1.2;
        const int text_thickness = 2;
        const cv::Scalar text_color = cv::Scalar(0, 0, 255); // 统一红色(BGR)
        // 文字距离画面顶部偏移量，可自行调整
        const int top_offset = 40;

        int baseline = 0;
        int img_w = resized_img.cols;
        int img_h = resized_img.rows;
        // 拼接缩放后整体宽640，左右原图各占一半宽度
        int half_w = img_w / 2;

        // ========== 左侧文字：左半张图水平居中，顶部偏上 ==========
        cv::Size text_size_left = cv::getTextSize(text_left, font_face, font_scale, text_thickness, &baseline);
        int left_text_x = half_w / 2 - text_size_left.width / 2;
        int text_y = top_offset + text_size_left.height;
        cv::Point left_pos(left_text_x, text_y);
        cv::putText(resized_img, text_left, left_pos, font_face, font_scale, text_color, text_thickness, cv::LINE_AA);

        // ========== 右侧文字：右半张图水平居中，顶部偏上 ==========
        cv::Size text_size_right = cv::getTextSize(text_right, font_face, font_scale, text_thickness, &baseline);
        int right_text_x = half_w + half_w / 2 - text_size_right.width / 2;
        cv::Point right_pos(right_text_x, text_y);
        cv::putText(resized_img, text_right, right_pos, font_face, font_scale, text_color, text_thickness, cv::LINE_AA);

        return resized_img;
    }


    
    };
}

#endif
