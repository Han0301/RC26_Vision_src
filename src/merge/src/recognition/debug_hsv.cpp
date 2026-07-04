#ifndef __DEBUG_HSV_CPP_
#define __DEBUG_HSV_CPP_

#include "debug_hsv.h"

namespace Ten{
    
void Ten_debug_hsv::save_hsv_hist_visualization(const std::vector<cv::Mat>& hsv_hist, 
                                 const std::string& save_path,
                                 int h_bin_num = 1,  // H通道独立bin数
                                 int s_bin_num = 1,  // S通道独立bin数
                                 int v_bin_num = 1,  // V通道独立bin数
                                 int canvas_height = 480) {
    // -------------------------- 1. 输入合法性校验 --------------------------
    if (hsv_hist.size() != 3) {
        std::cerr << "[ERROR] 输入直方图容器长度必须为3（H/S/V三通道），当前长度：" << hsv_hist.size() << std::endl;
        return;
    }
    if (save_path.empty()) {
        std::cerr << "[ERROR] 保存路径不能为空！" << std::endl;
        return;
    }
    // 校验bin数合法性
    if (h_bin_num <= 0 || s_bin_num <=0 || v_bin_num <=0) {
        std::cerr << "[ERROR] 分箱数必须为正整数！当前：H=" << h_bin_num << ", S=" << s_bin_num << ", V=" << v_bin_num << std::endl;
        return;
    }

    // -------------------------- 2. 定义绘图参数 --------------------------
    const int CHANNEL_NUM = 3;               // 通道数（H/S/V）
    const int CHANNEL_WIDTH = 360;           // 每个通道的绘图宽度
    const int CANVAS_W = CHANNEL_NUM * CHANNEL_WIDTH;  // 整体画布宽度（750）
    const int CANVAS_H = canvas_height;      // 整体画布高度（默认300）
    const cv::Scalar COLOR_H = cv::Scalar(0, 0, 255);    // H通道绘图颜色（红色）
    const cv::Scalar COLOR_S = cv::Scalar(0, 255, 0);    // S通道绘图颜色（绿色）
    const cv::Scalar COLOR_V = cv::Scalar(255, 0, 0);    // V通道绘图颜色（蓝色）
    const cv::Scalar COLOR_TEXT = cv::Scalar(0, 0, 0);   // 文字颜色（黑色）
    const cv::Scalar COLOR_BG = cv::Scalar(255, 255, 255);// 背景色（白色）
    const cv::Scalar COLOR_BORDER = cv::Scalar(180, 180, 180); // 通道分隔线颜色

    // 创建整体画布
    cv::Mat canvas(CANVAS_H, CANVAS_W, CV_8UC3, COLOR_BG);

    // -------------------------- 3. 定义各通道基础信息 --------------------------
    // 通道配置：bin数、绘图颜色、通道名称、直方图数据
    struct ChannelConfig {
        int bin_num;
        cv::Scalar color;
        std::string name;
        const cv::Mat& hist;
    };
    std::vector<ChannelConfig> channel_configs = {
        {h_bin_num, COLOR_H, "H Channel (bin=" + std::to_string(h_bin_num) + ")", hsv_hist[0]},
        {s_bin_num, COLOR_S, "S Channel (bin=" + std::to_string(s_bin_num) + ")", hsv_hist[1]},
        {v_bin_num, COLOR_V, "V Channel (bin=" + std::to_string(v_bin_num) + ")", hsv_hist[2]}
    };

    // -------------------------- 4. 遍历3个通道绘制直方图 --------------------------
    for (int ch_idx = 0; ch_idx < CHANNEL_NUM; ch_idx++) {
        // 4.1 计算当前通道的绘图区域
        int x_start = ch_idx * CHANNEL_WIDTH;  // 通道左上角X坐标
        int x_end = (ch_idx + 1) * CHANNEL_WIDTH; // 通道右下角X坐标
        int draw_height = CANVAS_H - 50;      // 绘图高度（预留文字空间）
        int bin_width = CHANNEL_WIDTH / channel_configs[ch_idx].bin_num; // 每个bin的宽度

        // 4.2 获取当前通道配置
        const auto& config = channel_configs[ch_idx];
        const cv::Mat& hist = config.hist;

        // 4.3 绘制通道名称
        cv::putText(canvas, config.name, 
                    cv::Point(x_start + 10, 30), 
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, COLOR_TEXT, 1);

        // 4.4 绘制通道分隔线
        cv::line(canvas, cv::Point(x_start, 0), cv::Point(x_start, CANVAS_H), COLOR_BORDER, 1);

        // 4.5 绘制直方图（空/无效直方图则绘制警告框）
        if (!hist.empty() && hist.channels() == 1) {
            // 归一化直方图到绘图高度（0 ~ draw_height）
            cv::Mat hist_norm;
            cv::normalize(hist, hist_norm, 0, draw_height, cv::NORM_MINMAX);

            // 遍历每个bin绘制矩形
            for (int bin = 0; bin < config.bin_num; bin++) {
                // 跳过宽度为0的bin（避免绘图异常）
                if (bin_width <= 0) break;

                // 计算bin的像素值和绘制位置（从下往上绘制）
                float bin_val = hist_norm.at<float>(bin);
                int bin_val_int = cvRound(bin_val);
                int bin_x_start = x_start + bin * bin_width;
                int bin_y_start = CANVAS_H - 20 - bin_val_int;  // 底部预留20像素
                int bin_x_end = bin_x_start + bin_width - 1;
                int bin_y_end = CANVAS_H - 20;

                // 绘制bin矩形（填充）
                cv::rectangle(canvas,
                              cv::Point(bin_x_start, bin_y_start),
                              cv::Point(bin_x_end, bin_y_end),
                              config.color, -1);
                // 绘制bin边框（增强可读性）
                cv::rectangle(canvas,
                              cv::Point(bin_x_start, bin_y_start),
                              cv::Point(bin_x_end, bin_y_end),
                              cv::Scalar(0,0,0), 0.5);
            }
        } else {
            // 直方图为空/无效，绘制警告框
            cv::rectangle(canvas,
                          cv::Point(x_start + 20, 50),
                          cv::Point(x_end - 20, CANVAS_H - 20),
                          config.color, 2);
            cv::putText(canvas, "INVALID HIST", 
                        cv::Point(x_start + 40, CANVAS_H/2), 
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, config.color, 2);
        }
    }

    // 绘制最后一条分隔线
    cv::line(canvas, cv::Point(CANVAS_W-1, 0), cv::Point(CANVAS_W-1, CANVAS_H), COLOR_BORDER, 1);

    // -------------------------- 5. 保存图像 --------------------------
    if (!cv::imwrite(save_path, canvas)) {
        std::cerr << "[ERROR] 保存图像失败！路径：" << save_path << std::endl;
    } else {
        std::cout << "[INFO] HSV直方图可视化图像已保存至：" << save_path << std::endl;
    }
}

std::vector<cv::Mat> Ten_debug_hsv::set_hsv_hist(cv::Mat roi_image){
    std::vector<cv::Mat> hist;
    cv::Mat hsv_image;
    cv::cvtColor(roi_image, hsv_image, cv::COLOR_BGR2HSV);
    //  填充HSV直方图
    cv::Mat h_hist = cv::Mat::zeros(1, 180, CV_32S);     // HSV直方图
    cv::Mat s_hist = cv::Mat::zeros(1, 255, CV_32S);
    cv::Mat v_hist = cv::Mat::zeros(1, 255, CV_32S);

    for(int i = 0; i < hsv_image.cols; i ++){
        for(int j = 0;j < hsv_image.rows;j++){
            cv::Vec3b hsv = hsv_image.at<cv::Vec3b>(i,j);
            if(hsv[2] == 0)continue;
            h_hist.at<int>(0, hsv[0])++;
            s_hist.at<int>(0, hsv[1])++;
            v_hist.at<int>(0, hsv[2])++;
        }
    hist.push_back(h_hist);
    hist.push_back(s_hist);
    hist.push_back(v_hist);
    }
    return hist;
}



}
#endif 