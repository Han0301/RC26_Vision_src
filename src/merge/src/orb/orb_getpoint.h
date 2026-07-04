#ifndef __ORB_GETPOINT_H_
#define __ORB_GETPOINT_H_

#include "orb_debug.h"
#include "./../yolo/yolo_v5.h"

namespace Ten
{
    namespace ORB
    {
        
        class orb_getpoint
        {
        public:
            /** 
                @brief 初始化函数
                @param model_path:模型路径 /xxx/xxx/bin
                @param xpu: cpu or gpu
                @param conf_thres：框置信度
                @param cls_thres: 类别置信度
                @param iou: 交并比
            */
            orb_getpoint(const std::string model_path, const std::string xpu, float conf_thres = 0.75, float cls_thres = 0.75, float iou = 0)
            :yv_(model_path, xpu, conf_thres, cls_thres, iou)
            {

            }

            /**
             * @brief 由用户自己设置存在方块的数组
             * @param exist_boxes 输入 int， 12 数组， 1 表示存在， 0 表示不存在
             */
            void set_exist_boxes(int exist_boxes[12])
            {
                of_.set_exist_boxes(exist_boxes);
            }

            /**
             * @brief 设置外参RT
             * @param rvec:旋转向量
             * @param tvec:平移向量
             * @param img: 相机获取图片
             * @param flag:是否重跑yolo, 0 重跑
             */
            void generate_points(cv::Mat rvec, cv::Mat tvec, cv::Mat img, int flag = 0)
            {
                of_.camerainfo_.set_RT(rvec, tvec);
                of_.filterValidCorners();
                ot_.projectValidCorners(of_.getCubeList(), of_.camerainfo_.K(), of_.camerainfo_.distCoeffs(), of_.camerainfo_.rvec(), of_.camerainfo_.tvec());
                if(flag == 0 || yv_points_.empty())
                {
                    std::vector<Ten::yolo::Detection> results = yv_.worker(img);
                    yv_points_ = convertDetectionsToPoints(results);
                }
                cv::Mat debug1 = ot_.drawValidPoints(img);
                debug_image_= drawYoloPoints(debug1);
                
            }

            /**
             * @brief 获取保存的有效像素点列表（含方块ID）
             * @return 2D像素点列表（含ID）
             */
            std::vector<orb_points> getPixelPoints() const
            {
                return ot_.getPixelPoints();
            }

            /**
             * @brief 获取保存的yolo识别角点
             * @return std::vector<cv::Point2d>
             */
            std::vector<cv::Point2d> getyoloPoints() const
            {
                return yv_points_;
            }

            cv::Mat getdebugimage()
            {
                return debug_image_;
            }

        private:
            orb_filter of_;
            orb_transform ot_;
            Ten::yolo::yolo_v5 yv_;
            std::vector<cv::Point2d> yv_points_;
            cv::Mat debug_image_;


            /**
             * @brief 将Detection向量转换为Point2d向量（提取cx_和cy_）
             * @param detections 输入的检测结果向量
             * @return 转换后的二维点向量，每个点对应Detection的(cx_, cy_)
             */
            std::vector<cv::Point2d> convertDetectionsToPoints(const std::vector<Ten::yolo::Detection>& detections) 
            {
                // 初始化结果向量，预分配内存提升效率
                std::vector<cv::Point2d> points;
                points.reserve(detections.size());

                // 遍历每个检测结果，提取中心点坐标构造Point2d
                for (const auto& det : detections) {
                    // cv::Point2d的构造函数为Point2d(double x, double y)
                    points.emplace_back(det.cx_, det.cy_);
                }
                return points;
            }

             /**
             * @brief 调试函数：在图片上绘制yv_points_的空心圆（基于Detection的cx_/cy_）
             * @param src_img     原始输入图片（CV_8UC3/RGB格式）
             * @param color       圆的颜色（默认红色 BGR格式）
             * @param thickness   线条宽度（默认1，空心需>=1）
             * @param radius      圆的半径（默认3，可根据需求调整）
             * @return 绘制后的图片
             */
            cv::Mat drawYoloPoints(
                const cv::Mat& src_img, 
                const cv::Scalar& color = cv::Scalar(0, 255, 0), 
                int thickness = 1,
                int radius = 3
            )
            {
                // 拷贝原图，避免修改输入的原始图片
                cv::Mat dst_img = src_img.clone();

                // 空值保护：无检测点时输出警告并返回原图
                if (yv_points_.empty())
                {
                    std::cerr << "[Warning] No yolo detection points (yv_points_) to draw!" << std::endl;
                    return dst_img;
                }

                // 遍历所有yolo检测点，绘制抗锯齿空心圆
                for (const cv::Point2d& pt : yv_points_)
                {
                    // cv::circle支持Point2d类型，自动转换为整数坐标（像素级）
                    cv::circle(dst_img, pt, radius, color, thickness, cv::LINE_AA); 
                }

                return dst_img;
            }

        };

    } 
}

#endif 
