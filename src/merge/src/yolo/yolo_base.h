#ifndef __YOLO_BASE_H_
#define __YOLO_BASE_H_
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <openvino/openvino.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>


namespace Ten
{
    namespace yolo
    {
        //检测结果
        struct Detection {
            double cx_; 
            double cy_;
            double w_; 
            double h_;
            double conf_;
            int cls_id_;
        };

        //模型
        struct model
        {
            int item_;
            float confidence_;
        };

        
        class yolo
        {
            public:
                //模型推理
                virtual std::vector<Detection> worker(cv::Mat& img) = 0;
                //virtual ~yolo() = 0;
            protected:
                //将opencv图片转化为Tensor类型
                virtual ov::Tensor preprocess(cv::Mat& image) = 0;
                //推理后模型数据具体解析
                virtual std::vector<Detection> postprocess(ov::Tensor& output, int orig_w, int orig_h) = 0;
        };


        //自定义比较大小
        inline bool compareConfidence(const model& a, const model& b)
        {
            return a.confidence_ > b.confidence_;
        }


        /**
         * @brief 辅助函数：计算两个检测框的IOU（交并比）
         * @param a 检测框a
         * @param b 检测框b
         * @return 0~1的IOU值，无交集则返回0
         */
        float calculateIOU(const Detection& a, const Detection& b);
        // {
        //     // 中心坐标(cx,cy,w,h)转换为左上角(x1,y1)、右下角(x2,y2)格式
        //     // x1 = cx - w/2, y1 = cy - h/2; x2 = cx + w/2, y2 = cy + h/2
        //     float a_x1 = a.cx_ - a.w_ / 2.0f;
        //     float a_y1 = a.cy_ - a.h_ / 2.0f;
        //     float a_x2 = a.cx_ + a.w_ / 2.0f;
        //     float a_y2 = a.cy_ + a.h_ / 2.0f;

        //     float b_x1 = b.cx_ - b.w_ / 2.0f;
        //     float b_y1 = b.cy_ - b.h_ / 2.0f;
        //     float b_x2 = b.cx_ + b.w_ / 2.0f;
        //     float b_y2 = b.cy_ + b.h_ / 2.0f;

        //     // 计算交集的左上角和右下角（取最大值/最小值）
        //     float inter_x1 = std::fmax(a_x1, b_x1);
        //     float inter_y1 = std::fmax(a_y1, b_y1);
        //     float inter_x2 = std::fmin(a_x2, b_x2);
        //     float inter_y2 = std::fmin(a_y2, b_y2);

        //     // 计算交集面积（无交集则面积为0）
        //     float inter_w = std::fmax(0.0f, inter_x2 - inter_x1);
        //     float inter_h = std::fmax(0.0f, inter_y2 - inter_y1);
        //     float inter_area = inter_w * inter_h;

        //     // 计算两个框的自身面积
        //     float a_area = a.w_ * a.h_;
        //     float b_area = b.w_ * b.h_;

        //     // 计算并集面积 = a面积 + b面积 - 交集面积
        //     float union_area = a_area + b_area - inter_area;
        //     if (union_area < 1e-9f) { // 避免除零错误（框面积为0的极端情况）
        //         return 0.0f;
        //     }

        //     // IOU = 交集面积 / 并集面积
        //     return inter_area / union_area;
        // }

        /**
         * @brief NMS非极大值抑制主函数：过滤重叠度超过IOU阈值的检测框
         * @param detections 原始检测框容器
         * @param iou_thresh IOU阈值（0~1，超过则过滤低置信度框）
         * @return 过滤后的检测框容器（保留置信度高的框）
         */
        std::vector<Detection> nmsFilter(std::vector<Detection>& detections, float iou_thresh);
        // {
        //     std::vector<Detection> result;
        //     // 边界条件：空容器直接返回
        //     if (detections.empty() || iou_thresh < 0.0f || iou_thresh > 1.0f) {
        //         return result;
        //     }

        //     // 1. 复制原始数据并按**置信度降序**排序（核心：先保留置信度高的框）
        //     std::vector<Detection>& sorted_dets = detections;
        //     std::sort(sorted_dets.begin(), sorted_dets.end(),
        //         [](const Detection& a, const Detection& b) {
        //             return a.conf_ > b.conf_; // 从大到小排序
        //         });

        //     // 2. 标记数组：记录每个框是否被保留（初始全部保留）
        //     std::vector<bool> keep(sorted_dets.size(), true);

        //     // 3. 遍历所有框，逐个作为基准框，过滤后续重叠度过高的同类别框
        //     for (size_t i = 0; i < sorted_dets.size(); ++i) {
        //         if (!keep[i]) { // 已被标记为过滤，跳过
        //             continue;
        //         }
        //         const Detection& base_det = sorted_dets[i]; // 基准框（当前置信度最高的未过滤框）
        //         // 遍历基准框之后的所有框（前面的已处理，置信度更高，无需回头）
        //         for (size_t j = i + 1; j < sorted_dets.size(); ++j) {
        //             if (!keep[j]) { // 已被标记为过滤，跳过
        //                 continue;
        //             }
        //             const Detection& curr_det = sorted_dets[j];
        //             // 关键：**同类别才计算IOU**，不同类别即使重叠也不过滤
        //             if (base_det.cls_id_ == curr_det.cls_id_) {
        //                 float iou = calculateIOU(base_det, curr_det);
        //                 if (iou > iou_thresh) { // 重叠度超过阈值，过滤当前框
        //                     keep[j] = false;
        //                 }
        //             }
        //         }
        //     }

        //     // 4. 收集所有被标记为保留的框
        //     for (size_t i = 0; i < sorted_dets.size(); ++i) {
        //         if (keep[i]) {
        //             result.push_back(sorted_dets[i]);
        //         }
        //     }

        //     return result;
        // }


    }
}






#endif










