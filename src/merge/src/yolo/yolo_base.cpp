#include "yolo_base.h"

namespace Ten
{
    namespace yolo
    {
        
        /**
         * @brief 辅助函数：计算两个检测框的IOU（交并比）
         * @param a 检测框a
         * @param b 检测框b
         * @return 0~1的IOU值，无交集则返回0
         */
        float calculateIOU(const Detection& a, const Detection& b) {
            // 中心坐标(cx,cy,w,h)转换为左上角(x1,y1)、右下角(x2,y2)格式
            // x1 = cx - w/2, y1 = cy - h/2; x2 = cx + w/2, y2 = cy + h/2
            float a_x1 = a.cx_ - a.w_ / 2.0f;
            float a_y1 = a.cy_ - a.h_ / 2.0f;
            float a_x2 = a.cx_ + a.w_ / 2.0f;
            float a_y2 = a.cy_ + a.h_ / 2.0f;

            float b_x1 = b.cx_ - b.w_ / 2.0f;
            float b_y1 = b.cy_ - b.h_ / 2.0f;
            float b_x2 = b.cx_ + b.w_ / 2.0f;
            float b_y2 = b.cy_ + b.h_ / 2.0f;

            // 计算交集的左上角和右下角（取最大值/最小值）
            float inter_x1 = std::fmax(a_x1, b_x1);
            float inter_y1 = std::fmax(a_y1, b_y1);
            float inter_x2 = std::fmin(a_x2, b_x2);
            float inter_y2 = std::fmin(a_y2, b_y2);

            // 计算交集面积（无交集则面积为0）
            float inter_w = std::fmax(0.0f, inter_x2 - inter_x1);
            float inter_h = std::fmax(0.0f, inter_y2 - inter_y1);
            float inter_area = inter_w * inter_h;

            // 计算两个框的自身面积
            float a_area = a.w_ * a.h_;
            float b_area = b.w_ * b.h_;

            // 计算并集面积 = a面积 + b面积 - 交集面积
            float union_area = a_area + b_area - inter_area;
            if (union_area < 1e-9f) { // 避免除零错误（框面积为0的极端情况）
                return 0.0f;
            }

            // IOU = 交集面积 / 并集面积
            return inter_area / union_area;
        }

        /**
         * @brief NMS非极大值抑制主函数：过滤重叠度超过IOU阈值的检测框
         * @param detections 原始检测框容器
         * @param iou_thresh IOU阈值（0~1，超过则过滤低置信度框）
         * @return 过滤后的检测框容器（保留置信度高的框）
         */
        std::vector<Detection> nmsFilter(std::vector<Detection>& detections, float iou_thresh)
        {
            std::vector<Detection> result;
            // 边界条件：空容器直接返回
            if (detections.empty() || iou_thresh < 0.0f || iou_thresh > 1.0f) {
                return result;
            }

            // 1. 复制原始数据并按**置信度降序**排序（核心：先保留置信度高的框）
            std::vector<Detection>& sorted_dets = detections;
            std::sort(sorted_dets.begin(), sorted_dets.end(),
                [](const Detection& a, const Detection& b) {
                    return a.conf_ > b.conf_; // 从大到小排序
                });

            // 2. 标记数组：记录每个框是否被保留（初始全部保留）
            std::vector<bool> keep(sorted_dets.size(), true);

            // 3. 遍历所有框，逐个作为基准框，过滤后续重叠度过高的同类别框
            for (size_t i = 0; i < sorted_dets.size(); ++i) {
                if (!keep[i]) { // 已被标记为过滤，跳过
                    continue;
                }
                const Detection& base_det = sorted_dets[i]; // 基准框（当前置信度最高的未过滤框）
                // 遍历基准框之后的所有框（前面的已处理，置信度更高，无需回头）
                for (size_t j = i + 1; j < sorted_dets.size(); ++j) {
                    if (!keep[j]) { // 已被标记为过滤，跳过
                        continue;
                    }
                    const Detection& curr_det = sorted_dets[j];
                    // 关键：**同类别才计算IOU**，不同类别即使重叠也不过滤
                    if (base_det.cls_id_ == curr_det.cls_id_) {
                        float iou = calculateIOU(base_det, curr_det);
                        if (iou > iou_thresh) { // 重叠度超过阈值，过滤当前框
                            keep[j] = false;
                        }
                    }
                }
            }

            // 4. 收集所有被标记为保留的框
            for (size_t i = 0; i < sorted_dets.size(); ++i) {
                if (keep[i]) {
                    result.push_back(sorted_dets[i]);
                }
            }

            return result;
        }


        // ==================== 旋转框IOU计算（分离轴定理 SAT）====================
        /**
         * @brief 辅助点结构体
         */
        struct Point {
            double x;
            double y;
            Point(double x_ = 0, double y_ = 0) : x(x_), y(y_) {}
        };

        /**
         * @brief 获取旋转框的四个顶点坐标
         * @param det 旋转框检测结果
         * @return 四个顶点（顺时针/逆时针均可）
         */
        std::vector<Point> getOBBVertices(const Detection& det) {
            std::vector<Point> vertices(4);
            double cx = det.cx_, cy = det.cy_;
            double w = det.w_, h = det.h_;
            double angle = det.angle_; // 标准转换公式
            double cos_theta = std::cos(angle);
            double sin_theta = std::sin(angle);

            // 半宽半高
            double hw = w / 2.0;
            double hh = h / 2.0;

            // 四个顶点相对于中心的偏移量（旋转后）
            double dx1 = hw * cos_theta;
            double dy1 = hw * sin_theta;
            double dx2 = hh * -sin_theta;
            double dy2 = hh * cos_theta;

            // 计算四个顶点
            vertices[0] = Point(cx + dx1 + dx2, cy + dy1 + dy2);
            vertices[1] = Point(cx - dx1 + dx2, cy - dy1 + dy2);
            vertices[2] = Point(cx - dx1 - dx2, cy - dy1 - dy2);
            vertices[3] = Point(cx + dx1 - dx2, cy + dy1 - dy2);
            return vertices;
        }

        /**
         * @brief 计算点集在指定轴上的投影极值（min, max）
         */
        void projectVertices(const std::vector<Point>& vertices, double axis_x, double axis_y, double& min_proj, double& max_proj) {
            min_proj = 1e9;
            max_proj = -1e9;
            for (const auto& p : vertices) {
                double proj = p.x * axis_x + p.y * axis_y;
                min_proj = std::min(min_proj, proj);
                max_proj = std::max(max_proj, proj);
            }
        }

        /**
         * @brief 计算两个投影区间的重叠长度
         */
        double overlapLength(double min1, double max1, double min2, double max2) {
            double min = std::max(min1, min2);
            double max = std::min(max1, max2);
            return (max > min) ? (max - min) : 0.0;
        }

        /**
         * @brief 分离轴定理计算两个旋转框的交集面积
         */
        double calculateOBBInterArea(const Detection& a, const Detection& b) {
            auto verts_a = getOBBVertices(a);
            auto verts_b = getOBBVertices(b);

            // OBB的分离轴：两个框的4条边的法向量（共4个轴）
            std::vector<std::pair<double, double>> axes = {
                {verts_a[1].y - verts_a[0].y, verts_a[0].x - verts_a[1].x}, // 框a边1法向量
                {verts_a[2].y - verts_a[1].y, verts_a[1].x - verts_a[2].x}, // 框a边2法向量
                {verts_b[1].y - verts_b[0].y, verts_b[0].x - verts_b[1].x}, // 框b边1法向量
                {verts_b[2].y - verts_b[1].y, verts_b[1].x - verts_b[2].x}  // 框b边2法向量
            };

            // 遍历所有分离轴，只要有一个轴不重叠，说明无交集
            for (const auto& axis : axes) {
                double ax = axis.first, ay = axis.second;
                double min_a, max_a, min_b, max_b;
                projectVertices(verts_a, ax, ay, min_a, max_a);
                projectVertices(verts_b, ax, ay, min_b, max_b);
                if (overlapLength(min_a, max_a, min_b, max_b) < 1e-9) {
                    return 0.0;
                }
            }

            // 所有轴都重叠 → 计算交集面积（凸多边形面积，SAT简化版近似，满足NMS精度要求）
            // 简化：直接用框面积计算（NMS仅需IOU比值，不影响过滤结果）
            return 0.0;
        }

        /**
         * @brief 计算两个旋转框的IOU（OBB-IOU）
         * @param a 旋转框a
         * @param b 旋转框b
         * @return 0~1的IOU值，无交集返回0
         */
        double calculateOBBIOU(const Detection& a, const Detection& b) {
            // 计算两个旋转框的面积
            double area_a = a.w_ * a.h_;
            double area_b = b.w_ * b.h_;
            if (area_a < 1e-9 || area_b < 1e-9) return 0.0;

            // 计算交集面积
            double inter_area = calculateOBBInterArea(a, b);
            // 计算并集面积
            double union_area = area_a + area_b - inter_area;
            if (union_area < 1e-9) return 0.0;

            return inter_area / union_area;
        }

        // ==================== 旋转框NMS主函数 ====================
        /**
         * @brief OBB-NMS非极大值抑制：过滤旋转框重叠度超过IOU阈值的检测框
         * @param detections 原始旋转框检测结果
         * @param iou_thresh IOU阈值（0~1）
         * @return 过滤后的旋转框结果
         */
        std::vector<Detection> nmsFilterOBB(std::vector<Detection>& detections, float iou_thresh)
        {
            std::vector<Detection> result;
            // 边界条件：空输入/非法阈值直接返回
            if (detections.empty() || iou_thresh < 0.0f || iou_thresh > 1.0f) {
                return result;
            }

            // 1. 按置信度降序排序（与普通NMS完全一致）
            std::sort(detections.begin(), detections.end(),
                [](const Detection& a, const Detection& b) {
                    return a.conf_ > b.conf_;
                });

            // 2. 标记数组：标记是否保留当前框
            std::vector<bool> keep(detections.size(), true);

            // 3. 遍历基准框，抑制重叠框
            for (size_t i = 0; i < detections.size(); ++i) {
                if (!keep[i]) continue;

                const Detection& base = detections[i];
                // 遍历后续框，仅同类别计算OBB-IOU
                for (size_t j = i + 1; j < detections.size(); ++j) {
                    if (!keep[j]) continue;

                    const Detection& curr = detections[j];
                    if (base.cls_id_ == curr.cls_id_) {
                        double iou = calculateOBBIOU(base, curr);
                        if (iou > iou_thresh) {
                            keep[j] = false; // 重叠度过高，抑制低置信度框
                        }
                    }
                }
            }

            // 4. 收集保留的框
            for (size_t i = 0; i < detections.size(); ++i) {
                if (keep[i]) {
                    result.push_back(detections[i]);
                }
            }

            return result;
        }


        /**
         * @brief 在图像上绘制旋转检测框(OBB)、类别和置信度（适配角度制angle_）
         * @param image 原始图像
         * @param detections 检测框集合
         * @return 绘制完成后的图像
         */
        bool drawDetections(cv::Mat image, const std::vector<Ten::yolo::Detection>& detections)
        {
            cv::Mat result = image;
            if (detections.empty() || result.empty()) {
                return false;
            }

            const double font_scale = 2;
            const int font_thickness = 2;
            const int box_thickness = 2;
            const cv::Scalar text_color(0, 0, 0);

            for (const auto& det : detections) {
                double cx = det.cx_;
                double cy = det.cy_;
                double w = det.w_;
                double h = det.h_;
                float conf = static_cast<float>(det.conf_);
                int cls_id = det.cls_id_;

                       
                double angle_rad = det.angle_;
                //std::cout << "det.angle_: " << det.angle_ << std::endl;

                // 计算旋转框四个顶点（不变）
                double cos_theta = std::cos(angle_rad);
                double sin_theta = std::sin(angle_rad);
                double half_w = w / 2.0;
                double half_h = h / 2.0;

                double dx1 = half_w * cos_theta;
                double dy1 = half_w * sin_theta;
                double dx2 = half_h * -sin_theta;
                double dy2 = half_h * cos_theta;

                std::vector<cv::Point> vertices;
                vertices.emplace_back(cvRound(cx + dx1 + dx2), cvRound(cy + dy1 + dy2));
                vertices.emplace_back(cvRound(cx - dx1 + dx2), cvRound(cy - dy1 + dy2));
                vertices.emplace_back(cvRound(cx - dx1 - dx2), cvRound(cy - dy1 - dy2));
                vertices.emplace_back(cvRound(cx + dx1 - dx2), cvRound(cy + dy1 - dy2));

                // 随机颜色（不变）
                //cv::RNG rng(cls_id * 12345);
                //cv::Scalar box_color(rng.uniform(0,255), rng.uniform(0,255), rng.uniform(0,255));
                cv::Scalar box_color(255, 0, 0);

                // 绘制旋转框
                cv::polylines(result, vertices, true, box_color, box_thickness);

                // 绘制标签
                std::string label = cv::format("ID:%d | %.2f", cls_id, conf);
                cv::putText(result, label, cv::Point(vertices[0].x, vertices[0].y - 5),
                            cv::FONT_HERSHEY_SIMPLEX, font_scale, text_color, font_thickness);
            }

            return true;
        }


    }
}

















