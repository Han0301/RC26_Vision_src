#ifndef __ZBUFFER_SIMPLIFY_CPP_
#define __ZBUFFER_SIMPLIFY_CPP_

#include "zbuffer_simplify.h"

namespace Ten
{
    // ---------------------------------------------------------------------------------------------------------------------
void Ten_zbuffer_simplify::set_box_lists_(
    const cv::Mat& image,     
    const std::vector<cv::Point3f>& C_object_plum_points,
    const std::vector<cv::Point2f>& object_plum_2d_points,
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

    std::vector<surface_2d_point> o_front_2d;
    std::vector<surface_2d_point> o_side_2d;
    std::vector<surface_2d_point> o_up_2d;
    std::vector<surface_2d_point> p_front_2d;
    std::vector<surface_2d_point> p_side_2d;
    std::vector<surface_2d_point> p_up_2d;
    for (int i = 0;i < 96;i +=8){
        int idx = i / 8 + 1;
        float o_front_depth = (cal_distance(C_object_plum_points[i]) + cal_distance(C_object_plum_points[i + 1]) + cal_distance(C_object_plum_points[i + 2]) + cal_distance(C_object_plum_points[i + 3])) / 4.0f;
        float o_left_depth = (cal_distance(C_object_plum_points[i + 4]) + cal_distance(C_object_plum_points[i]) + cal_distance(C_object_plum_points[i + 3]) + cal_distance(C_object_plum_points[i + 7])) / 4.0f; 
        float o_right_depth = (cal_distance(C_object_plum_points[i + 1]) + cal_distance(C_object_plum_points[i + 5]) + cal_distance(C_object_plum_points[i + 6]) + cal_distance(C_object_plum_points[i + 2])) / 4.0f; 
        float o_up_depth = (cal_distance(C_object_plum_points[i + 4]) + cal_distance(C_object_plum_points[i + 5]) + cal_distance(C_object_plum_points[i + 1]) + cal_distance(C_object_plum_points[i])) / 4.0f;
        
        o_front_2d.push_back({idx,object_plum_2d_points[i],object_plum_2d_points[i + 1], object_plum_2d_points[i + 2], object_plum_2d_points[i + 3],o_front_depth});
        o_up_2d.push_back({idx,object_plum_2d_points[i + 4],object_plum_2d_points[i + 5], object_plum_2d_points[i + 1], object_plum_2d_points[i],o_up_depth});
        if (o_left_depth > o_right_depth) {
            o_side_2d.push_back({idx,object_plum_2d_points[i + 1],object_plum_2d_points[i + 5], object_plum_2d_points[i + 6], object_plum_2d_points[i + 2],o_right_depth});
        }else{o_side_2d.push_back({idx,object_plum_2d_points[i + 4],object_plum_2d_points[i], object_plum_2d_points[i + 3], object_plum_2d_points[i + 7],o_left_depth});}

        float p_front_depth = (cal_distance(C_object_plum_points[96 + i]) + cal_distance(C_object_plum_points[96 + i + 1]) + cal_distance(C_object_plum_points[96 + i + 2])+ cal_distance(C_object_plum_points[96 + i + 3])) / 4.0f;
        float p_left_depth = (cal_distance(C_object_plum_points[96 + i + 4]) + cal_distance(C_object_plum_points[96 + i]) + cal_distance(C_object_plum_points[96 + i + 3]) + cal_distance(C_object_plum_points[96 + i + 7])) / 4.0f; 
        float p_right_depth = (cal_distance(C_object_plum_points[96 + i + 1]) + cal_distance(C_object_plum_points[96 + i + 5]) + cal_distance(C_object_plum_points[96 + i + 6]) + cal_distance(C_object_plum_points[96 + i + 2])) / 4.0f; 
        float p_up_depth = (cal_distance(C_object_plum_points[96 + i + 4]) + cal_distance(C_object_plum_points[96 + i + 5]) + cal_distance(C_object_plum_points[96 + i + 1]) + cal_distance(C_object_plum_points[96 + i])) / 4.0f;
        
        p_front_2d.push_back({idx,object_plum_2d_points[96 + i],object_plum_2d_points[96 + i + 1], object_plum_2d_points[96 + i + 2], object_plum_2d_points[96 + i + 3],p_front_depth});
        p_up_2d.push_back({idx,object_plum_2d_points[96 + i + 4],object_plum_2d_points[96 + i + 5], object_plum_2d_points[96 + i + 1], object_plum_2d_points[96 + i],p_up_depth});
        if (o_left_depth > o_right_depth) {
            p_side_2d.push_back({idx,object_plum_2d_points[96 + i + 1],object_plum_2d_points[96 + i + 5], object_plum_2d_points[96 + i + 6], object_plum_2d_points[96 + i + 2],p_right_depth});
        }else{p_side_2d.push_back({idx,object_plum_2d_points[96 + i + 4],object_plum_2d_points[96 + i], object_plum_2d_points[96 + i + 3], object_plum_2d_points[96 + i + 7],p_left_depth});}  
    } 

    // 2. 填充 zbuffer, object_zbuffer 矩阵
    // 2.1 初始化深度缓冲（初始值为最大浮点数，表示无深度）
    cv::Mat zbuffer = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;
    cv::Mat object_zbuffer = cv::Mat::ones(image.rows, image.cols, CV_32F) * FLT_MAX;

    // 2.2 提取2d点坐标
    if (!(o_front_2d.size() == 12 && o_side_2d.size() == 12 && o_up_2d.size() == 12 && p_front_2d.size() == 12 && p_side_2d.size() == 12 && p_up_2d.size() == 12)){
        ROS_WARN("in surface_2d_point, the size is not 12!!!");
        return;
    }

    // 2.3 先填充台阶的深度
    for (size_t i = 0; i < p_side_2d.size(); i++) {
        auto& p_front = p_front_2d[i];
        auto& p_side = p_side_2d[i];
        auto& p_up = p_up_2d[i];

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
    for (size_t i = 0; i < o_side_2d.size(); i++) {
        if (i >= o_front_2d.size() || i >= o_up_2d.size()) continue;

        // // 2.4.1 同时满足 exist_boxes[i] != 0 && interested_boxes[i] == 1 才更新
        // if (!(exist_boxes[i] != 0 && interested_boxes[i] == 1))continue;
        // if (box_lists[i].zbuffer_flag == -1) continue;
        
        auto& o_front = o_front_2d[i];
        auto& o_side = o_side_2d[i];
        auto& o_up = o_up_2d[i];

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
        ///---------------------------------------------------------不更新 roi_image 的条件 -------------------------------
        if (valid_max_points.empty()) {
            ROS_WARN("box idx=%d valid_max_points is empty, skip crop ROI", box_lists[i].idx);
            // box_lists[i].zbuffer_flag = -1; // 标记异常
            continue;
        }

        if (valid_max_points.size() <= 600) {
            ROS_WARN("valid_max_points.size() <= 600");
            //box_lists[i].zbuffer_flag = -1; // 标记异常
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
        //--------------------------------------------------------------------------------------------------------------
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
        square_roi.copyTo(box_lists[i].roi_image);
        box_lists[i].zbuffer_flag = 1;
    }    
};

    void Ten_zbuffer_simplify::set_hsv_topn_stand(std::vector<box>& box_lists, std::vector<score>& score_lists,int top_n,int min_manage_points)
    {
        set_hsv_top_n(box_lists, score_lists,top_n,min_manage_points);
        std::unordered_map<int,float> total_h_pos_degree;
        std::unordered_map<int,float> total_s_pos_degree;
        std::unordered_map<int,float> total_v_pos_degree;
        for (int i = 0; i < 12; i++)
        {
            if (!(score_lists[i].is_managed))
            {
                continue;
            }           
            for(int j = 0; j < top_n; j++)
            {
                total_h_pos_degree[score_lists[i].top_n_h[j].position] += score_lists[i].top_n_h[j].degree_of_promacy;
                total_s_pos_degree[score_lists[i].top_n_s[j].position] += score_lists[i].top_n_s[j].degree_of_promacy;
                total_v_pos_degree[score_lists[i].top_n_v[j].position] += score_lists[i].top_n_v[j].degree_of_promacy;
            }
        }
        std::vector<std::pair<int, float>> h_pos_degree(total_h_pos_degree.begin(), total_h_pos_degree.end());
        std::vector<std::pair<int, float>> s_pos_degree(total_s_pos_degree.begin(), total_s_pos_degree.end());
        std::vector<std::pair<int, float>> v_pos_degree(total_v_pos_degree.begin(), total_v_pos_degree.end());
        
        std::sort(h_pos_degree.begin(), h_pos_degree.end(),
            [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second > b.second;
            });

        std::sort(s_pos_degree.begin(), s_pos_degree.end(),
            [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second > b.second;
            });
        std::sort(v_pos_degree.begin(), v_pos_degree.end(),
            [](const std::pair<int, float>& a, const std::pair<int, float>& b) {
                return a.second > b.second;
            });

        standard_hsv_ = {h_pos_degree[0].first, s_pos_degree[0].first, v_pos_degree[0].first};
    }

    void Ten_zbuffer_simplify::set_hsv_topn_score(std::vector<box>& box_lists, std::vector<score>& score_lists,int top_n,int min_manage_points)
    {
        set_hsv_top_n(box_lists, score_lists,top_n,min_manage_points);
        std::vector<float> score = {-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f,-1.0f}; 
        for(size_t i = 0; i < 12; i++) {
            if (!(score_lists[i].is_managed))
            {
                score_lists[i].hsv_score = score[i];
                continue;
            }
            float total_degree_h = 0.0f;
            float total_degree_s = 0.0f;
            float total_degree_v = 0.0f;

            for(size_t j = 0; j < top_n; j++)
            {
                total_degree_h += score_lists[i].top_n_h[j].degree_of_promacy;
                total_degree_s += score_lists[i].top_n_s[j].degree_of_promacy;
                total_degree_v += score_lists[i].top_n_v[j].degree_of_promacy;
            }
            for(size_t j = 0; j < top_n; j++)
            {   
                int h = score_lists[i].top_n_h[j].position;
                int s = score_lists[i].top_n_s[j].position;
                int v = score_lists[i].top_n_v[j].position;

                float h_diff = std::min( std::abs(h - standard_hsv_[0]), (std::min(h,standard_hsv_[0]) + 180 - std::max(h,standard_hsv_[0]))   );
                float h_diff_score = 0.0f;

                if (h_diff < 5)
                {
                    h_diff_score = 1.0f * h_diff;
                }
                else if (5 < h_diff < 10)
                {
                    h_diff_score = 5.0f + 3.0f * (h_diff - 5);
                }
                else
                {
                    h_diff_score = 20.0f + 5.0f * (h_diff - 10);
                }
                float now_score = (score_lists[i].top_n_h[j].degree_of_promacy / total_degree_h) * 
                                  std::max( 0.0f, 70.0f - h_diff_score)
                     + (score_lists[i].top_n_s[j].degree_of_promacy / total_degree_s) * std::max(0.0f,10.0f - 0.2f * std::abs(s - standard_hsv_[1]))
                     + (score_lists[i].top_n_v[j].degree_of_promacy / total_degree_v) * std::max(0.0f,15.0f - 0.3f * std::abs(v - standard_hsv_[2]));
                score[i] += now_score;
            }   
            score_lists[i].hsv_score = score[i];
        }

        std::sort(score_lists.begin(),score_lists.end(),
        [](const Ten::score&a, const Ten::score&b){
            return a.hsv_score > b.hsv_score;
        });

    }

void Ten_zbuffer_simplify::set_hsv_top_n(std::vector<box>& box_lists, std::vector<score>& score_lists,int top_n,int min_manage_points)
{
    // 1 清空上一帧的缓存
    if (!(score_lists.empty()))
    {
        score_lists.clear();
    }
    score_lists.resize(12);

    // 2 填充 score_lists中的 idx,hsv_mode 字段
    for (const auto& box: box_lists){
        cv::Mat hsv_image;
        cv::cvtColor(box.roi_image, hsv_image, cv::COLOR_BGR2HSV);
        //  2.1 填充HSV直方图
        cv::Mat h_hist = cv::Mat::zeros(1, 180, CV_32S);     // HSV直方图
        cv::Mat s_hist = cv::Mat::zeros(1, 256, CV_32S);
        cv::Mat v_hist = cv::Mat::zeros(1, 256, CV_32S);

        int valid_points = 0;
        for(int i = 0; i < hsv_image.rows; i ++){
            for(int j = 0;j < hsv_image.cols;j++){
                cv::Vec3b hsv = hsv_image.at<cv::Vec3b>(i,j);
                if(hsv[2] == 0 || hsv[1] < 15)continue;     // 过滤掉 黑色无效像素 和 白色干扰像素
                h_hist.at<int>(0, hsv[0])++;
                s_hist.at<int>(0, hsv[1])++;
                v_hist.at<int>(0, hsv[2])++;
                valid_points++;
            }
        }
        score_lists[box.idx - 1].idx = box.idx;
        score_lists[box.idx - 1].valid_count = valid_points;

        if (valid_points < min_manage_points) 
        {
            score_lists[box.idx - 1].is_managed = false;
            continue;
        }
        else{score_lists[box.idx - 1].is_managed = true;}
        std::vector<HistValue> h_topn = getTopNValues(h_hist, top_n);
        std::vector<HistValue> s_topn = getTopNValues(s_hist, top_n);
        std::vector<HistValue> v_topn = getTopNValues(v_hist, top_n);

        int h_mode = h_topn.empty() ? 0 : h_topn[0].position;
        int s_mode = s_topn.empty() ? 0 : s_topn[0].position;
        int v_mode = v_topn.empty() ? 0 : v_topn[0].position;

        // 2.3 填充数据
        score_lists[box.idx - 1].top_n_h = h_topn;
        score_lists[box.idx - 1].top_n_s = s_topn;
        score_lists[box.idx - 1].top_n_v = v_topn;
    }
}

void Ten_zbuffer_simplify::set_debug_roi_image(
    std::vector<Ten::box>box_lists,
    std::vector<Ten::score> score_lists,
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
                std::map<int,float> idx_score;
                for(const auto& score: score_lists)
                {
                    idx_score[score.idx] = score.hsv_score;
                }
                roi_images[vec_idx] = resized_img.clone();
                cv::putText(roi_images[vec_idx], std::to_string(box_lists[vec_idx].cls), cv::Point(roi_images[vec_idx].cols - 80 , roi_images[vec_idx].rows - 120), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                cv::putText(roi_images[vec_idx], std::to_string(box_lists[vec_idx].confidence), cv::Point(roi_images[vec_idx].cols - 80 , roi_images[vec_idx].rows - 80), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
                cv::putText(roi_images[vec_idx], std::to_string(idx_score[vec_idx + 1]), cv::Point(roi_images[vec_idx].cols - 80 , roi_images[vec_idx].rows - 40), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 255, 0), 2);
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

cv::Mat Ten_zbuffer_simplify::update_debug_image(
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

    
    Ten::Ten_zbuffer_simplify _ZBUFFER_SIMPLIFY_;
    Ten::init_3d_box _INIT_3D_BOX_;

}

#endif 