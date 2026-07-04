#ifndef _Ten_occlusion_handing_CPP_
#define _Ten_occlusion_handing_CPP_
#include "occlusion_handing.h"

namespace Ten{
void Ten_occlusion_handing::set_box_lists_(  
    const cv::Mat& image,     
    const std::vector<cv::Point3f>& C_object_plum_points,
    const std::vector<cv::Point2f>& object_plum_2d_points,
    std::vector<box>& box_lists)
{
    // 1. 取到 exist_boxes_ 和 interested_boxes_
    int exist_boxes[12];
    int interested_boxes[12];
    {
        std::lock_guard<std::mutex> lock(mtx_);
        for(int i = 0; i < 12; i++)
        {
            exist_boxes[i] = exist_boxes_[i];
            interested_boxes[i] = interested_boxes_[i];
        }
    }
    // 2. 根据深度信息 更新2d点列表
    std::vector<surface_2d_point> object_2d;
    std::vector<surface_2d_point> plum_2d;      // 通过下标来访问， 【0】表示 正面或后面， 【1】表示 左侧面或右侧面， 【2】表示 上面或地面
    set_surface_2d_point(C_object_plum_points, object_plum_2d_points, object_2d, "object");
    set_surface_2d_point(C_object_plum_points, object_plum_2d_points, plum_2d, "plum");

    // 3. 填充 zbuffer 矩阵
    // 3.1 初始化深度缓冲（初始值为最大浮点数，表示无深度）
    cv::Mat zbuffer = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
    cv::Mat object_zbuffer = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;

    // 3.2 检查surface_2d_point 2d点 合理性
    if (!(object_2d.size() == 36 && plum_2d.size() == 36)){
        std::cout << "😨❌in func: set_box_lists_ 3 , the object_2d or plum_2d is not 36❓❓❓" << std::endl;
        std::cout << "🤡object_2d.size(): " << object_2d.size() << ", 🤡plum_2d.size(): " << plum_2d.size() << std::endl;
        return;
    }

    // 3.3 先填充台阶的深度
    for (size_t i = 0; i < plum_2d.size(); i+=3) 
    {
        auto& p_front = plum_2d[i];
        auto& p_side = plum_2d[i + 1];
        auto& p_up = plum_2d[i + 2];

        // 3.3.1 收集台阶的所有2D点坐标，判断整个台阶的所有点是否都在图像外
        std::vector<cv::Point2f> all_points;
        bool all_outside = set_all_outside(p_front,p_side,p_up,image.cols,image.rows,all_points);
        if (all_outside) continue; 
        // 3.3.2 构建台阶轮廓, 填充台阶深度到临时矩阵 plum_tmp
        cv::Mat plum_temp = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
        set_temp(p_front,p_side,p_up,plum_temp);
        // 3.3.3 计算台阶像素范围
        float plum_x_min = FLT_MAX,plum_y_min = FLT_MAX,plum_x_max = FLT_MIN,plum_y_max = FLT_MIN;
        cal_points_range(all_points,plum_x_min,plum_y_min,plum_x_max,plum_y_max);
        // 3.3.4 写入主zbuffer（台阶深度更近则更新）
        for (int row = int(plum_y_min) - 1; row < int(plum_y_max) + 1; ++row) {
            for (int col = int(plum_x_min) - 1; col < int(plum_x_max) + 1; ++col) {
                if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) continue;
                if (plum_temp.at<float>(row, col) < zbuffer.at<float>(row, col)) {
                    zbuffer.at<float>(row, col) = plum_temp.at<float>(row, col);
                }
            }
        }  
    }
    // 3.4 再填充方块的深度, 4 在循环中填充各个方块的roi图像信息
    for(size_t i = 0; i < object_2d.size(); i+=3)
    {
        // 3.4.0 重置 roi_valid_flag 状态
        box_lists[i / 3].roi_valid_flag = 0;

        auto& o_front = object_2d[i];
        auto& o_side = object_2d[i + 1];
        auto& o_up = object_2d[i + 2];

        // 3.4.1 收集方块所有2D点坐标， 并判断方块的所有点是否都在图像外
        std::vector<cv::Point2f> all_points;
        bool all_outside = set_all_outside(o_front,o_side,o_up,image.cols,image.rows,all_points);
        if (all_outside) continue; 
        // 3.4.2 构建方块轮廓, 填充方块深度到临时矩阵 object_temp
        cv::Mat object_temp = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
        set_temp(o_front,o_side,o_up,object_temp);
        // 3.4.3 计算方块像素范围
        float object_x_min = FLT_MAX,object_y_min = FLT_MAX,object_x_max = FLT_MIN,object_y_max = FLT_MIN;
        cal_points_range(all_points,object_x_min,object_y_min,object_x_max,object_y_max);
        // 3.4.4 合并到方块深度缓冲 object_zbuffer + 全局深度缓冲 zbuffer, 并 写入当前方块范围的 depth_regions 
        std::unordered_map<float, std::vector<cv::Point2f>> depth_regions;        // depth_regions 表示 深度-对应深度的点集
        for (int row = int(object_y_min) - 1; row < int(object_y_max) + 1; ++row) {
            for (int col = int(object_x_min) - 1; col < int(object_x_max) + 1; ++col) {
                if (row < 0 || row >= image.rows || col < 0 || col >= image.cols) continue;
                if (object_temp.at<float>(row, col) == FLT_MAX) continue;
                if (object_temp.at<float>(row, col) < zbuffer.at<float>(row, col)) {
                    zbuffer.at<float>(row, col) = object_temp.at<float>(row, col);
                    object_zbuffer.at<float>(row, col) = object_temp.at<float>(row, col);
                }
                depth_regions[object_zbuffer.at<float>(row, col)].emplace_back(col, row);
            }
        }     
        // 4 填充好单个方块的zbuffer深度信息后， 开始裁剪图像信息
        // 4.1 在当前方块范围内，找到 有效的，面积最大的（认为在方块几个面中最优）的 点集 valid_max_points
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

            // 逻辑1： 取三个面中最大的面填充 roi_image
            // if (int(points.size()) > max_points_count){
            //     valid_max_points = points;
            //     max_points_count = points.size(); 
            // }

            // 逻辑2： 同时取三个面填来充 roi_image
            valid_max_points.insert(valid_max_points.end(), points.begin(),points.end());
        }
        ///---------------------------------------------------------4.2 不更新 roi_image 的条件 -------------------------------
        bool is_update_img = is_update_image(box_lists,valid_max_points,exist_boxes,interested_boxes,i);
        if (!(is_update_img)) continue;
        
        // 4.3 准备有效区域的掩码, 并更新有效区域的外接x_min,y_min,x_max,y_max
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
        // 4.4 裁剪ROI
        cv::Rect roi_rect(x_min, y_min, x_max - x_min + 1, y_max - y_min + 1);
        // 4.4.1 校验roi_rect，避免宽高为负
        if (roi_rect.width <= 0 || roi_rect.height <= 0 || 
            roi_rect.x + roi_rect.width > image.cols || 
            roi_rect.y + roi_rect.height > image.rows) {
            std::cout << "🤡in func: set_box_lists_ 4.4.1,Invalid ROI rect: x= " <<roi_rect.x <<", y="<<roi_rect.y<<", w="<<roi_rect.width<<", h="<<roi_rect.height <<", skip" << std::endl;
            continue;
        }
        // 4.4.2 生成 image_roi,mask_roi
        cv::Mat image_roi = image(roi_rect);
        cv::Mat mask_roi = roi_mask(roi_rect);

        // 4.5 在 image_roi 中 生成有效区域 mask_roi
        cv::Mat crop_roi = cv::Mat::zeros(image_roi.size(), image_roi.type());
        image_roi.copyTo(crop_roi, mask_roi);

        // 4.6 转为正方形
        int max_side = std::max(crop_roi.cols, crop_roi.rows);
        cv::Mat square_roi = cv::Mat::zeros(max_side, max_side, crop_roi.type());
        int x_offset = (max_side - crop_roi.cols) / 2;
        int y_offset = (max_side - crop_roi.rows) / 2;
        cv::Rect paste_rect(x_offset, y_offset, crop_roi.cols, crop_roi.rows);
        
        // 4.7 最后一次校验paste_rect
        if (paste_rect.x >= 0 && paste_rect.y >= 0 && 
            paste_rect.x + paste_rect.width <= square_roi.cols && 
            paste_rect.y + paste_rect.height <= square_roi.rows) {
            crop_roi.copyTo(square_roi(paste_rect));
        } else {
            std::cout << "🤡in func: set_box_lists_ 4.7, Invalid paste rect for square ROI, skip" << std::endl;
            continue;
        }

        // 4.8 准备填充 box_lists 信息
        square_roi.copyTo(box_lists[i / 3].roi_image);
        box_lists[i / 3].zbuffer_flag = 1;
        box_lists[i / 3].roi_valid_flag = 1;
    }


}

cv::Mat Ten_occlusion_handing::update_debug_image(
    cv::Mat image,
    const std::vector<cv::Point2f>& object_plum_2d_points_
){
    // 1. 检查输入有效性
    if (image.empty()) {
        ROS_WARN("Image is empty, skip draw");
        return cv::Mat();
    }
    cv::Mat img;
    image.copyTo(img);

    for (size_t i = 0; i < object_plum_2d_points_.size(); i++) {
        
        if (i < 96 && i % 8 == 0){
        cv::line(img, object_plum_2d_points_[i], object_plum_2d_points_[i + 1], cv::Scalar(0,255,0), 2, cv::LINE_AA);
        cv::line(img, object_plum_2d_points_[i + 1], object_plum_2d_points_[i + 2], cv::Scalar(0,255,0), 2, cv::LINE_AA);
        cv::line(img, object_plum_2d_points_[i + 2], object_plum_2d_points_[i + 3], cv::Scalar(0,255,0), 2, cv::LINE_AA);
        cv::line(img, object_plum_2d_points_[i + 3], object_plum_2d_points_[i], cv::Scalar(0,255,0), 2, cv::LINE_AA);
    }   
}
return img;
}
void Ten_occlusion_handing::set_debug_roi_image(
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
                cv::putText(roi_images[vec_idx], std::to_string(box_lists[vec_idx].cls), cv::Point(roi_images[vec_idx].cols - 80 , roi_images[vec_idx].rows - 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                cv::putText(roi_images[vec_idx], std::to_string(box_lists[vec_idx].confidence), cv::Point(roi_images[vec_idx].cols - 80 , roi_images[vec_idx].rows - 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
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
    Ten::Ten_occlusion_handing _OCCLUSION_HANDING_;
    Ten::init_3d_box _INIT_3D_BOX_;
}       // namespace Ten
#endif