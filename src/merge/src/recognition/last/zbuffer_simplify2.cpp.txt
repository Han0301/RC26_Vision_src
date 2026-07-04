#ifndef __ZBUFFER_SIMPLIFY_CPP_
#define __ZBUFFER_SIMPLIFY_CPP_

#include "zbuffer_simplify.h"


namespace Ten
{
    // ---------------------------------------------------------------------------------------------------------------------
    void Ten_zbuffer_simplify::set_box_lists_init(
        const std::vector<cv::Point3f>& C_object_points,
        const std::vector<cv::Point3f>& C_plum_points,
        const std::vector<cv::Point2f>& object_2d_points,
        const std::vector<cv::Point2f>& plum_2d_points,
        std::vector<std::vector<surface_2d_point>>& object_2d_points_,
        std::vector<std::vector<surface_2d_point>>& plum_2d_points_
    ){
        std::vector<surface_2d_point> object_2d_front_points_;
        std::vector<surface_2d_point> object_2d_side_points_;
        std::vector<surface_2d_point> object_2d_up_points_;
        std::vector<surface_2d_point> plum_2d_front_points_;
        std::vector<surface_2d_point> plum_2d_side_points_;
        std::vector<surface_2d_point> plum_2d_up_points_;

        for (int i = 0;i < 96;i +=8 ){
            int idx = i / 8 + 1;
            float object_front_depth = (calc_camera_distance(C_object_points[i]) + calc_camera_distance(C_object_points[i + 1]) + calc_camera_distance(C_object_points[i + 2]) + calc_camera_distance(C_object_points[i + 3])) / 4.0f;
            float object_left_depth = (calc_camera_distance(C_object_points[i + 4]) + calc_camera_distance(C_object_points[i]) + calc_camera_distance(C_object_points[i + 3]) + calc_camera_distance(C_object_points[i + 7])) / 4.0f; 
            float object_right_depth = (calc_camera_distance(C_object_points[i + 1]) + calc_camera_distance(C_object_points[i + 5]) + calc_camera_distance(C_object_points[i + 6]) + calc_camera_distance(C_object_points[i + 2])) / 4.0f; 
            float object_up_depth = (calc_camera_distance(C_object_points[i + 4]) + calc_camera_distance(C_object_points[i + 5]) + calc_camera_distance(C_object_points[i + 1]) + calc_camera_distance(C_object_points[i])) / 4.0f;
            object_2d_front_points_.push_back({idx,object_2d_points[i],object_2d_points[i + 1], object_2d_points[i + 2], object_2d_points[i + 3],object_front_depth});
            object_2d_up_points_.push_back({idx,object_2d_points[i + 4],object_2d_points[i + 5], object_2d_points[i + 1], object_2d_points[i],object_up_depth});
            if (object_left_depth > object_right_depth) {
                object_2d_side_points_.push_back({idx,object_2d_points[i + 1],object_2d_points[i + 5], object_2d_points[i + 6], object_2d_points[i + 2],object_right_depth});
            }else{object_2d_side_points_.push_back({idx,object_2d_points[i + 4],object_2d_points[i], object_2d_points[i + 3], object_2d_points[i + 7],object_left_depth});}

            float plum_front_depth = (calc_camera_distance(C_plum_points[i]) + calc_camera_distance(C_plum_points[i + 1]) + calc_camera_distance(C_plum_points[i + 2])+ calc_camera_distance(C_plum_points[i + 3])) / 4.0f;
            float plum_left_depth = (calc_camera_distance(C_plum_points[i + 4]) + calc_camera_distance(C_plum_points[i]) + calc_camera_distance(C_plum_points[i + 3]) + calc_camera_distance(C_plum_points[i + 7])) / 4.0f; 
            float plum_right_depth = (calc_camera_distance(C_plum_points[i + 1]) + calc_camera_distance(C_plum_points[i + 5]) + calc_camera_distance(C_plum_points[i + 6]) + calc_camera_distance(C_plum_points[i + 2])) / 4.0f; 
            float plum_up_depth = (calc_camera_distance(C_plum_points[i + 4]) + calc_camera_distance(C_plum_points[i + 5]) + calc_camera_distance(C_plum_points[i + 1]) + calc_camera_distance(C_plum_points[i])) / 4.0f;
            plum_2d_front_points_.push_back({idx,plum_2d_points[i],plum_2d_points[i + 1], plum_2d_points[i + 2], plum_2d_points[i + 3],plum_front_depth});
            plum_2d_up_points_.push_back({idx,plum_2d_points[i + 4],plum_2d_points[i + 5], plum_2d_points[i + 1], plum_2d_points[i],plum_up_depth});
            if (object_left_depth > object_right_depth) {
                plum_2d_side_points_.push_back({idx,plum_2d_points[i + 1],plum_2d_points[i + 5], plum_2d_points[i + 6], plum_2d_points[i + 2],plum_right_depth});
            }else{plum_2d_side_points_.push_back({idx,plum_2d_points[i + 4],plum_2d_points[i], plum_2d_points[i + 3], plum_2d_points[i + 7],plum_left_depth});}  
        } 
        object_2d_points_[0] = object_2d_front_points_;
        object_2d_points_[1] = object_2d_side_points_;
        object_2d_points_[2] = object_2d_up_points_;
        plum_2d_points_[0] = plum_2d_front_points_;
        plum_2d_points_[1] = plum_2d_side_points_;
        plum_2d_points_[2] = plum_2d_up_points_;
    };

    void Ten_zbuffer_simplify::set_box_lists_(
        const cv::Mat& image,     
        const std::vector<std::vector<surface_2d_point>>& object_2d_points_lists,
        const std::vector<std::vector<surface_2d_point>>& plum_2d_points_lists,
        std::vector<box>& box_lists){

        int exist_boxes[12];
        int interested_boxes[12];
        {
            std::lock_guard<std::mutex> lock(mtx_);
            for(int i = 0; i < 12; i++)
            {
                exist_boxes[i] = exist_boxes_[i];
            }

            for(int i = 0; i < 12; i++)
            {
                interested_boxes[i] = interested_boxes_[i];
            }
        }

        // 1 检查图像有效性
        if (image.empty() || image.cols <= 0 || image.rows <= 0) {
            ROS_WARN("Invalid image size: cols=%d, rows=%d", image.cols, image.rows);
            return;
        }
        // 2. 填充 zbuffer, object_zbuffer 矩阵
        // 2.1 初始化深度缓冲（初始值为最大浮点数，表示无深度）
        cv::Mat zbuffer = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
        cv::Mat object_zbuffer = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;

        // 2.2 提取2d点坐标
        std::vector<surface_2d_point> object_front_2d = object_2d_points_lists[0];
        std::vector<surface_2d_point> object_side_2d = object_2d_points_lists[1];
        std::vector<surface_2d_point> object_up_2d = object_2d_points_lists[2];
        std::vector<surface_2d_point> plum_front_2d = plum_2d_points_lists[0];
        std::vector<surface_2d_point> plum_side_2d = plum_2d_points_lists[1];
        std::vector<surface_2d_point> plum_up_2d = plum_2d_points_lists[2];
        if (!(object_front_2d.size() == 12 && object_side_2d.size() == 12 && object_up_2d.size() == 12 && plum_front_2d.size() == 12 && plum_side_2d.size() == 12 && plum_up_2d.size() == 12)){
            ROS_WARN("in surface_2d_point, the size is not 12!!!");
            return;
        }

        // 2.3 先填充台阶的深度
        for (size_t i = 0; i < plum_side_2d.size(); i++) {
            auto& p_front = plum_front_2d[i];
            auto& p_side = plum_side_2d[i];
            auto& p_up = plum_up_2d[i];

            // 2.3.1 收集台阶的所有2D点坐标，判断整个台阶的所有点是否都在图像外
            std::vector<cv::Point2f> all_points = {
                p_front.left_up, p_front.right_up, p_front.right_down, p_front.left_down,
                p_side.left_up, p_side.right_up, p_side.right_down, p_side.left_down,
                p_up.left_up, p_up.right_up, p_up.right_down, p_up.left_down

            };
            bool all_outside = true;
            for (const auto& pt : all_points) {
                if (pt.x >= 0 && pt.x < image.cols && pt.y >= 0 && pt.y < image.rows) {
                    all_outside = false;
                    break;
                }
            }
            if (all_outside) continue; 
            
            // 2.3.2 构建台阶轮廓
            std::vector<cv::Point> front_contour = {
                cv::Point(cvRound(p_front.left_up.x), cvRound(p_front.left_up.y)),
                cv::Point(cvRound(p_front.right_up.x), cvRound(p_front.right_up.y)),
                cv::Point(cvRound(p_front.right_down.x), cvRound(p_front.right_down.y)),
                cv::Point(cvRound(p_front.left_down.x), cvRound(p_front.left_down.y))
            };
            std::vector<cv::Point> side_contour = {
                cv::Point(cvRound(p_side.left_up.x), cvRound(p_side.left_up.y)),
                cv::Point(cvRound(p_side.right_up.x), cvRound(p_side.right_up.y)),
                cv::Point(cvRound(p_side.right_down.x), cvRound(p_side.right_down.y)),
                cv::Point(cvRound(p_side.left_down.x), cvRound(p_side.left_down.y))
            };
            std::vector<cv::Point> up_contour = {
                cv::Point(cvRound(p_up.left_up.x), cvRound(p_up.left_up.y)),
                cv::Point(cvRound(p_up.right_up.x), cvRound(p_up.right_up.y)),
                cv::Point(cvRound(p_up.right_down.x), cvRound(p_up.right_down.y)),
                cv::Point(cvRound(p_up.left_down.x), cvRound(p_up.left_down.y))
            };

            // 2.3.3 填充台阶深度到临时矩阵 plum_tmp
            cv::Mat plum_tmp = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
            cv::fillPoly(plum_tmp, std::vector<std::vector<cv::Point>>{front_contour}, cv::Scalar(p_front.surface_depth));
            cv::fillPoly(plum_tmp, std::vector<std::vector<cv::Point>>{side_contour}, cv::Scalar(p_side.surface_depth));
            cv::fillPoly(plum_tmp, std::vector<std::vector<cv::Point>>{up_contour}, cv::Scalar(p_up.surface_depth));

            // 2.3.4 计算台阶像素范围
            float plum_x_min = FLT_MAX,plum_y_min = FLT_MAX,plum_x_max = FLT_MIN,plum_y_max = FLT_MIN;
            for(const auto& p : all_points) {
                if(p.x > plum_x_max) plum_x_max = p.x;
                if(p.x < plum_x_min) plum_x_min = p.x;
                if(p.y > plum_y_max) plum_y_max = p.y;
                if(p.y < plum_y_min) plum_y_min = p.y;
            }

            // 2.3.5 写入主zbuffer（台阶深度更近则更新）
            for (int row = int(plum_y_min) - 1; row < int(plum_y_max) + 1; ++row) {
                for (int col = int(plum_x_min) - 1; col < int(plum_x_max) + 1; ++col) {
                    if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) continue;
                    if (plum_tmp.at<float>(row, col) < zbuffer.at<float>(row, col)) {
                        zbuffer.at<float>(row, col) = plum_tmp.at<float>(row, col);
                    }
                }
            }
        }

        // 2.4 再填充方块的深度, 3 在循环中填充各个方块的roi图像信息
        for (size_t i = 0; i < object_side_2d.size(); i++) {
            if (i >= object_front_2d.size() || i >= object_up_2d.size()) break;

            // // 2.4.1 同时满足 exist_boxes[i] != 0 && interested_boxes[i] == 1 才更新
            // if (!(exist_boxes[i] != 0 && interested_boxes[i] == 1))continue;
            // if (box_lists[i].zbuffer_flag == -1) continue;
            
            auto& o_front = object_front_2d[i];
            auto& o_side = object_side_2d[i];
            auto& o_up = object_up_2d[i];

            // 2.4.2 收集方块所有2D点坐标， 并判断方块的所有点是否都在图像外
            std::vector<cv::Point2f> all_points = {
                o_front.left_up, o_front.right_up, o_front.right_down, o_front.left_down,
                o_side.left_up, o_side.right_up, o_side.right_down, o_side.left_down,
                o_up.left_up, o_up.right_up, o_up.right_down, o_up.left_down
            };
            bool all_outside = true;
            for (const auto& pt : all_points) {
                if (pt.x >= 0 && pt.x < image.cols && pt.y >= 0 && pt.y < image.rows) {
                    all_outside = false;
                    break;
                }
            }
            if (all_outside) continue; 

            // 2.4.3 构建方块轮廓
            std::vector<cv::Point> front_contour = {
                cv::Point(cvRound(o_front.left_up.x), cvRound(o_front.left_up.y)),
                cv::Point(cvRound(o_front.right_up.x), cvRound(o_front.right_up.y)),
                cv::Point(cvRound(o_front.right_down.x), cvRound(o_front.right_down.y)),
                cv::Point(cvRound(o_front.left_down.x), cvRound(o_front.left_down.y))
            };
            std::vector<cv::Point> up_contour = {
                cv::Point(cvRound(o_up.left_up.x), cvRound(o_up.left_up.y)),
                cv::Point(cvRound(o_up.right_up.x), cvRound(o_up.right_up.y)),
                cv::Point(cvRound(o_up.right_down.x), cvRound(o_up.right_down.y)),
                cv::Point(cvRound(o_up.left_down.x), cvRound(o_up.left_down.y))
            };
            std::vector<cv::Point> side_contour = {
                cv::Point(cvRound(o_side.left_up.x), cvRound(o_side.left_up.y)),
                cv::Point(cvRound(o_side.right_up.x), cvRound(o_side.right_up.y)),
                cv::Point(cvRound(o_side.right_down.x), cvRound(o_side.right_down.y)),
                cv::Point(cvRound(o_side.left_down.x), cvRound(o_side.left_down.y))
            };

            // 2.4.4 填充方块深度到临时矩阵 plum_tmp
            cv::Mat tmp = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
            cv::fillPoly(tmp, std::vector<std::vector<cv::Point>>{front_contour}, cv::Scalar(o_front.surface_depth));
            cv::fillPoly(tmp, std::vector<std::vector<cv::Point>>{up_contour}, cv::Scalar(o_up.surface_depth));
            cv::fillPoly(tmp, std::vector<std::vector<cv::Point>>{side_contour}, cv::Scalar(o_side.surface_depth));

            // 2.4.5 计算方块像素范围
            float object_x_min = FLT_MAX,object_y_min = FLT_MAX,object_x_max = FLT_MIN,object_y_max = FLT_MIN;
            for(const auto& p : all_points) {
                if(p.x > object_x_max) object_x_max = p.x;
                if(p.x < object_x_min) object_x_min = p.x;
                if(p.y > object_y_max) object_y_max = p.y;
                if(p.y < object_y_min) object_y_min = p.y;
            }

            // 2.4.7 合并到方块深度缓冲 object_zbuffer + 全局深度缓冲 zbuffer
            for (int row = int(object_y_min) - 1; row < int(object_y_max) + 1; ++row) {
                for (int col = int(object_x_min) - 1; col < int(object_x_max) + 1; ++col) {
                    if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) continue;
                    if (tmp.at<float>(row, col) < zbuffer.at<float>(row, col)) {
                        zbuffer.at<float>(row, col) = tmp.at<float>(row, col);
                        object_zbuffer.at<float>(row, col) = tmp.at<float>(row, col);
                    }
                }
            }
            // 3 填充好单个方块的zbuffer深度信息后， 开始裁剪图像信息
            // 3.1 写入当前方块范围的 depth_regions 
            std::unordered_map<float, std::vector<cv::Point2f>> depth_regions;        // depth_regions 表示 深度-对应深度的点集
            for (int y = int(object_y_min) - 1; y < int(object_y_max) + 1; ++y) {
                for (int x = int(object_x_min) - 1; x < int(object_x_max) + 1; ++x) {
                    float d = object_zbuffer.at<float>(y, x);
                    if (d == FLT_MAX) continue;
                    depth_regions[d].emplace_back(x, y);
                }
            }
            // 3.2 在当前方块范围内，找到 有效的，面积最大的（认为在方块几个面中最优）的 点集 valid_max_points
            int max_points_count = INT_MIN;
            std::vector<cv::Point2f> valid_max_points;
            for(const auto& [depth,points]: depth_regions){
                bool is_valid = false;
                for (const auto& valid_depth : std::vector<float> {o_front.surface_depth,o_up.surface_depth,o_side.surface_depth}){
                    if (std::fabs(depth - valid_depth) < 1e-4){
                        is_valid = true;
                    }
                }
                if(!is_valid) continue;
                if(points.empty()) continue;

                if (int(points.size()) > max_points_count){
                    valid_max_points = points;
                    max_points_count = points.size(); 
                }
            }       
            // 更新 points 和 points_count////////////////////////////////////////////////////////////////////////////////maple
            //if (max_points_count <= box_lists[i].points_count)continue;
                box_lists[i].points_count = max_points_count;

            ///---------------------------------------------------------改-------------------------------
            if (valid_max_points.empty()) {
                ROS_WARN("box idx=%d valid_max_points is empty, skip crop ROI", box_lists[i].idx);
                // box_lists[i].zbuffer_flag = -1; // 标记异常
                continue;
            }

            if (valid_max_points.size() <= 150) {
                ROS_WARN("valid_max_points.size() <= 150", box_lists[i].idx);
                // box_lists[i].zbuffer_flag = -1; // 标记异常
                continue;
            }

            // 同时满足 exist_boxes[i] != 0 && interested_boxes[i] == 1 才更新
            if (!(exist_boxes[i] != 0 && interested_boxes[i] == 1))
            {
                ROS_WARN("!(exist_boxes[i] != 0 && interested_boxes[i] == 1)");
                continue;
            }
            if (box_lists[i].zbuffer_flag == -1)
            {
                ROS_WARN("box_lists[i].zbuffer_flag == -1");
                continue;
            } 
            //-------------------------------------------------------------------------------------------


            // 3.3 准备有效区域的掩码, 并更新有效区域的外接x_min,y_min,x_max,y_max
            int x_min = INT_MAX, x_max = INT_MIN;   
            int y_min = INT_MAX, y_max = INT_MIN;
            cv::Mat roi_mask = cv::Mat::zeros(image.size(), CV_8UC1);       // 掩码的强制格式要求：单通道、8 位灰度图
            for (const auto& p : valid_max_points){
                if (p.y >= 0 && p.y < roi_mask.rows && p.x >= 0 && p.x < roi_mask.cols) {
                    roi_mask.at<uchar>(p.y, p.x) = 255;
                    x_min = std::min(x_min, int(p.x));
                    x_max = std::max(x_max, int(p.x));
                    y_min = std::min(y_min, int(p.y));
                    y_max = std::max(y_max, int(p.y));
                }
            }

            // 3.4 裁剪ROI
            cv::Rect roi_rect(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
            // 3.4.1 校验roi_rect，避免宽高为负
            if (roi_rect.width <= 0 || roi_rect.height <= 0 || 
                roi_rect.x + roi_rect.width > image.cols || 
                roi_rect.y + roi_rect.height > image.rows) {
                ROS_WARN("Invalid ROI rect: x=%d, y=%d, w=%d, h=%d, skip", 
                        roi_rect.x, roi_rect.y, roi_rect.width, roi_rect.height);
                continue;
            }
            // 3.4.2 生成 image_roi,mask_roi
            cv::Mat image_roi = image(roi_rect);
            cv::Mat mask_roi = roi_mask(roi_rect);

            // 3.5 在 image_roi 中 生成有效区域 mask_roi
            cv::Mat crop_roi = cv::Mat::zeros(image_roi.size(), image_roi.type());
            image_roi.copyTo(crop_roi, mask_roi);

            // 3.6 转为正方形
            int roi_width = crop_roi.cols;
            int roi_height = crop_roi.rows;
            int max_side = std::max(roi_width, roi_height);
            cv::Mat square_roi = cv::Mat::zeros(max_side, max_side, crop_roi.type());
            int x_offset = (max_side - roi_width) / 2;
            int y_offset = (max_side - roi_height) / 2;
            cv::Rect paste_rect(x_offset, y_offset, roi_width, roi_height);
            
            // 3.7 最后一次校验paste_rect
            if (paste_rect.x >= 0 && paste_rect.y >= 0 && 
                paste_rect.x + paste_rect.width <= square_roi.cols && 
                paste_rect.y + paste_rect.height <= square_roi.rows) {
                crop_roi.copyTo(square_roi(paste_rect));
            } else {
                ROS_WARN("Invalid paste rect for square ROI, skip");
                continue;
            }

            // 3.8 准备填充 box_lists 信息
            //box_lists[i].roi_image = square_roi;
            square_roi.copyTo(box_lists[i].roi_image);
            box_lists[i].zbuffer_flag = 1;
            
        }    
    
    };

    void Ten_zbuffer_simplify::set_debug_roi_image(
        std::vector<Ten::box>box_lists,
        cv::Mat& debug_best_roi_image
    ){
        // 1. 配置固定参数
        const int SINGLE_SIZE = 160;    // 单个图的目标尺寸（160×160）
        const int COL_NUM = 4;          // 每行列数
        const int ROW_NUM = 3;          // 总行数
        const int TOTAL_IMGS = 12;      // 总图片数（1-12）
        // 2. 初始化12个160×160的全黑图
        std::vector<cv::Mat> roi_images(TOTAL_IMGS, cv::Mat::zeros(SINGLE_SIZE, SINGLE_SIZE, CV_8UC3));

        // 3. 填充有效ROI图（idx1-12）
        for (int idx = 1; idx <= TOTAL_IMGS; ++idx) {
            // 3.1 计算当前idx在vector中的索引（idx1→0，idx12→11）
            int vec_idx = idx - 1;
            // 3.2 检查best_roi_image中是否有该idx的图
            for(const auto& box : box_lists)
            {
                if(box.idx == idx && !box.roi_image.empty())
                {
                    const cv::Mat& src_img = box.roi_image;
                    // 3.3 校验源图类型
                    if (src_img.type() != CV_8UC3) {
                        ROS_WARN("Idx %d image type error (not CV_8UC3), use black image", idx);
                        continue;
                    }
                    // 3.4 resize为160×160（原正方形，无畸变）
                    cv::Mat resized_img;
                    cv::resize(src_img, resized_img, cv::Size(SINGLE_SIZE, SINGLE_SIZE), 0, 0, cv::INTER_LINEAR);
                    // 3.5 替换初始化的黑图
                    roi_images[vec_idx] = resized_img.clone();
                    cv::putText(roi_images[vec_idx], std::to_string(box_lists[vec_idx].cls), cv::Point(roi_images[vec_idx].cols - 100 , roi_images[vec_idx].rows - 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                    cv::putText(roi_images[vec_idx], std::to_string(box_lists[vec_idx].confidence), cv::Point(roi_images[vec_idx].cols - 100 , roi_images[vec_idx].rows - 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                    cv::putText(roi_images[vec_idx], std::to_string(box_lists[vec_idx].exist_flag), cv::Point(roi_images[vec_idx].cols - 100 , roi_images[vec_idx].rows - 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                    break;
                }
            }
        }

        // 4. 拼接成640×480的大图
        for (int row = 0; row < ROW_NUM; ++row) {
            for (int col = 0; col < COL_NUM; ++col) {
                // 4.1 计算当前小图在vector中的索引
                int vec_idx = row * COL_NUM + col;
                if (vec_idx >= TOTAL_IMGS) break; // 防止越界（理论上不会触发）

                // 4.2 计算当前小图在拼接图中的位置
                int x = col * SINGLE_SIZE;
                int y = row * SINGLE_SIZE;
                cv::Rect roi_rect(x, y, SINGLE_SIZE, SINGLE_SIZE);

                // 4.3 将小图复制到拼接图对应位置
                roi_images[vec_idx].copyTo(debug_best_roi_image(roi_rect));
            }
        }
    };

    void Ten_zbuffer_simplify::set_HSV_exist_boxes_(std::vector<box>& box_lists){
        std::unordered_map<int, std::tuple<int, int, int>> idx_hsv_map_ = set_idx_hsv_map_(box_lists);
        std::vector<float> score = {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f}; 
        std::cout << "standard_hsv: " << standard_hsv_[0] << "  "  << standard_hsv_[1] << "  "  << standard_hsv_[2] << std::endl;
        // 1.计算得分并填充 下标-得分 列表 idx_score_lists
        std::vector<std::pair<int,float>> idx_score_lists;
        for(const auto& [idx, hsv_tuple] : idx_hsv_map_) {
            if (idx < 1 || idx > score.size()) 
            {
                //idx_score_lists.emplace_back(idx, score[idx - 1]);
                ROS_WARN("if (idx < 1 || idx > score.size()) ");
                continue;
            }
            
            if (box_lists[idx - 1].zbuffer_flag != 1) 
            {
                idx_score_lists.emplace_back(idx, score[idx - 1]);
                continue;
            }
            int h = std::get<0>(hsv_tuple);
            int s = std::get<1>(hsv_tuple);
            int v = std::get<2>(hsv_tuple);
            std::cout << "idx: " << idx <<", hsv_tuple: "<< h << " " << s << " " << v << "  " << std::endl;
            // 得分公式
            score[idx - 1] = std::max(0.0f,60.0f - abs(h - standard_hsv_[0]) )
                        + std::max(0.0f,10.0f - 0.2f * abs(s - standard_hsv_[1]))
                        + std::max(0.0f,30.0f - 0.5f * abs(v - standard_hsv_[2]));

            idx_score_lists.emplace_back(idx, score[idx - 1]);
        }
        
        // 2. 按照得分进行排序
        std::sort(idx_score_lists.begin(), idx_score_lists.end(),
            [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second > b.second;
            });

        // 3. 填充 exist_boxes
        for(size_t i = 0;i < idx_score_lists.size(); ++i){
            std::cout << "idx: " << idx_score_lists[i].first << ": " << idx_score_lists[i].second << std::endl;
        }

        size_t total = 0;
        size_t noempty = 0;
        for(size_t i = 0; i < box_lists.size(); i++)
        {
            if(box_lists[i].zbuffer_flag == 1)
            {
                total++;
            }
            else
            {
                if(box_lists[i].exist_flag == 1)
                {
                    noempty++;
                }
            }

        }

        // for(size_t i = 0;i < idx_score_lists.size(); ++i){
        //     if (i < 8){
        //         box_lists[idx_score_lists[i].first - 1].exist_flag = 1; 
        //     }
        //     else
        //     {
        //         box_lists[idx_score_lists[i].first - 1].exist_flag = 0;
        //     }
        // }
        std::cout << "total" << total << std::endl;
        std::cout << "noempty" << noempty << std::endl;
        std::cout << "box_lists size" << box_lists.size() << std::endl;
        std::cout << "idx_score_lists size " << idx_score_lists.size() << std::endl;
        for(size_t i = 0;i < total; ++i){
            if (i < 8 - noempty){
                std::cout<<"idx_score_lists[i].first - 1 "<<idx_score_lists[i].first - 1<<std::endl;
                box_lists[idx_score_lists[i].first - 1].exist_flag = 1; 
            }
            else
            {
                std::cout<<"idx_score_lists[i].first - 1 "<<idx_score_lists[i].first - 1<<std::endl;
                box_lists[idx_score_lists[i].first - 1].exist_flag = 0;
            }
        }


        // for(size_t i = 0;i <  8 - std::count(managed_boxes.begin(),managed_boxes.end(),0); ++i){
        //     box_lists[idx_score_lists[i].first - 1].exist_flag = 1;
        // }
    }

    
    void Ten_zbuffer_simplify::set_standard_hsv_(std::vector<box>box_lists){
        std::unordered_map<int, std::tuple<int, int, int>> idx_hsv_map = set_idx_hsv_map_(box_lists);
        // 1. 填入结果向量
        std::vector<int> idx_h_result, idx_s_result, idx_v_result;
        for (const auto& [idx, hsv_tuple] : idx_hsv_map) {
            idx_h_result.push_back(std::get<0>(hsv_tuple));
            idx_s_result.push_back(std::get<1>(hsv_tuple));
            idx_v_result.push_back(std::get<2>(hsv_tuple));
        }
        // 2. 计算标准hsv值（计算方法： 最小总差值）
        standard_hsv_ = {cal_single_standard_hsv(idx_h_result),cal_single_standard_hsv(idx_s_result), cal_single_standard_hsv(idx_v_result)};
    }

    cv::Mat Ten_zbuffer_simplify::update_debug_image(
        cv::Mat image_in,
        const std::vector<std::vector<surface_2d_point>>& object_2d_points_lists
    ){
        
        // 1. 检查输入有效性
        if (image_in.empty()) {
            ROS_WARN("Image is empty, skip draw");
            return cv::Mat();
        }

        cv::Mat image;
        image_in.copyTo(image);

        std::vector<surface_2d_point> object_front_2d = object_2d_points_lists[0];
        std::vector<surface_2d_point> object_side_2d = object_2d_points_lists[1];
        std::vector<surface_2d_point> object_up_2d = object_2d_points_lists[2];

        for (size_t i = 0; i < object_side_2d.size(); i++) {
            if (i >= object_front_2d.size() || i >= object_up_2d.size()) break;
            
            auto& o_front = object_front_2d[i];
            auto& o_side = object_side_2d[i];
            auto& o_up = object_up_2d[i];

            cv::line(image, o_front.left_up, o_front.right_up, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_front.right_up, o_front.right_down, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_front.right_down, o_front.left_down, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_front.left_down, o_front.left_up, cv::Scalar(0,255,0), 2, cv::LINE_AA);

            cv::line(image, o_side.left_up, o_side.right_up, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_side.right_up, o_side.right_down, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_side.right_down, o_side.left_down, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_side.left_down, o_side.left_up, cv::Scalar(0,255,0), 2, cv::LINE_AA);

            cv::line(image, o_up.left_up, o_up.right_up, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_up.right_up, o_up.right_down, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_up.right_down, o_up.left_down, cv::Scalar(0,255,0), 2, cv::LINE_AA);
            cv::line(image, o_up.left_down, o_up.left_up, cv::Scalar(0,255,0), 2, cv::LINE_AA);
    }
    return image;

    }

    Ten::Ten_zbuffer_simplify _ZBUFFER_SIMPLIFY_;
    Ten::init_3d_box _INIT_3D_BOX_;

}






#endif 


