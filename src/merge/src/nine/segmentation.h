#ifndef __SEGMENTATION_H_
#define __SEGMENTATION_H_
// #include <iostream>
// #include <fstream>
// #include <vector>
// #include <string>
// #include <opencv2/opencv.hpp>
#include "mouse.h"


namespace Ten
{

    class segmentation
    {
    public:
        std::vector<cv::Mat> getroi(cv::Mat img)
        {
            if (img.empty()) {
                std::cerr << "图像加载失败" << std::endl;
                std::vector<cv::Mat>();
            }
            // 调用函数（img会被绘制上绿色网格线）
            std::vector<cv::Mat> grids = splitQuadrantTo9Grids(img, std::string(ROOT_DIR) + std::string("src/nine/point.txt"));
            return grids;
        }

        /**
         * @brief 调试用：将9张子图按3行3列拼接，最终缩放到640x640返回
         * @param grids 输入子图集合，顺序要求：从上到下、从左到右（行优先，与分割函数输出顺序完全对应）
         * @return 拼接并缩放后的640x640图像；数量异常或包含空图时返回640x640纯黑图
         */
        cv::Mat debugAssemble3x3Grid(const std::vector<cv::Mat>& grids)
        {
            // ========== 1. 异常校验：数量不为9直接返回纯黑图 ==========
            if (grids.size() != 9)
            {
                int img_type = CV_8UC3;
                if (!grids.empty())
                    img_type = grids[0].type();
                return cv::Mat::zeros(640, 640, img_type);
            }

            // ========== 2. 校验所有子图非空 ==========
            for (const auto& cell : grids)
            {
                if (cell.empty())
                {
                    return cv::Mat::zeros(640, 640, grids[0].type());
                }
            }

            // ========== 3. 以第一张图为基准，统一所有子图尺寸 ==========
            int cell_width = grids[0].cols;
            int cell_height = grids[0].rows;

            // 创建拼接大图（3行3列）
            cv::Mat big_image(3 * cell_height, 3 * cell_width, grids[0].type());

            // ========== 4. 按行优先顺序填充3x3网格 ==========
            for (int row = 0; row < 3; ++row)
            {
                for (int col = 0; col < 3; ++col)
                {
                    int idx = row * 3 + col;
                    // 计算当前格子在大图中的ROI位置
                    cv::Rect roi_rect(col * cell_width, row * cell_height, cell_width, cell_height);
                    cv::Mat roi = big_image(roi_rect);

                    // 尺寸一致直接复制，不一致则先缩放再复制
                    if (grids[idx].cols == cell_width && grids[idx].rows == cell_height)
                    {
                        grids[idx].copyTo(roi);
                    }
                    else
                    {
                        cv::Mat resized_cell;
                        cv::resize(grids[idx], resized_cell, cv::Size(cell_width, cell_height));
                        resized_cell.copyTo(roi);
                    }
                }
            }

            // ========== 5. 整体缩放到640x640并返回 ==========
            cv::Mat result;
            cv::resize(big_image, result, cv::Size(640, 640), 0, 0, cv::INTER_LINEAR);
            return result;
        }

    private:


    /**
     * @brief 计算两点之间的欧氏距离
     */
    double pointDistance(const cv::Point2f& p1, const cv::Point2f& p2) {
        double dx = p1.x - p2.x;
        double dy = p1.y - p2.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    /**
     * @brief 校验四个点是否按 左上、右上、右下、左下 顺序排列
     * @param points 输入的4个点
     * @return 顺序正确返回true，否则返回false
     */
    bool checkPointOrder(const std::vector<cv::Point2f>& points) {
        if (points.size() != 4) return false;

        const cv::Point2f& a = points[0]; // 预期左上
        const cv::Point2f& b = points[1]; // 预期右上
        const cv::Point2f& c = points[2]; // 预期右下
        const cv::Point2f& d = points[3]; // 预期左下

        // 上方两点y应小于下方两点y
        float avg_y_top = (a.y + b.y) * 0.5f;
        float avg_y_bottom = (c.y + d.y) * 0.5f;
        if (avg_y_top >= avg_y_bottom) return false;

        // 上方两点：左点x < 右点x
        if (a.x >= b.x) return false;

        // 下方两点：左点x < 右点x（左下d，右下c）
        if (d.x >= c.x) return false;

        // 左侧两点x应小于右侧两点x
        float avg_x_left = (a.x + d.x) * 0.5f;
        float avg_x_right = (b.x + c.x) * 0.5f;
        if (avg_x_left >= avg_x_right) return false;

        return true;
    }

    /**
     * @brief 线性插值点
     */
    cv::Point2f lerpPoint(const cv::Point2f& p1, const cv::Point2f& p2, float t) {
        return p1 + (p2 - p1) * t;
    }

    /**
     * @brief 读取txt点集，将四边形9等分并截取小格子，统一输出640x640
     * @param img 输入原图（会在上面绘制绿色网格线）
     * @param txt_path 点坐标txt文件路径
     * @return 9张子图的vector，顺序：从上到下、从左到右；失败返回空
     */
    std::vector<cv::Mat> splitQuadrantTo9Grids(cv::Mat& img, const std::string& txt_path) {
        // ===================== 1. 读取并校验4个点 =====================
        cv::Mat roi = img.clone();
        std::vector<cv::Point2f> corners;
        std::ifstream in_file(txt_path);
        if (!in_file.is_open()) {
            std::cerr << "错误：无法打开txt文件 " << txt_path << std::endl;
            return {};
        }

        float x, y;
        while (in_file >> x >> y) {
            corners.emplace_back(x, y);
        }
        in_file.close();

        // 校验点数量
        if (corners.size() != 4) {
            std::cerr << "错误：点数量不为4，实际数量 = " << corners.size() << std::endl;
            return {};
        }

        // 校验是否超出图像边界
        if (img.empty()) {
            std::cerr << "错误：输入图像为空" << std::endl;
            return {};
        }
        int img_w = img.cols;
        int img_h = img.rows;
        for (const auto& p : corners) {
            if (p.x < 0 || p.x >= img_w || p.y < 0 || p.y >= img_h) {
                std::cerr << "错误：点超出图像范围 (" << p.x << ", " << p.y << ")" << std::endl;
                return {};
            }
        }

        // 校验点顺序：左上、右上、右下、左下
        if (!checkPointOrder(corners)) {
            std::cerr << "错误：点顺序不正确，需按 左上-右上-右下-左下 排列" << std::endl;
            return {};
        }

        cv::Point2f a = corners[0]; // 左上
        cv::Point2f b = corners[1]; // 右上
        cv::Point2f c = corners[2]; // 右下
        cv::Point2f d = corners[3]; // 左下

        // ===================== 2. 计算3等分点，构建4x4网格 =====================
        // 四条边的3等分点（每条边4个点，含端点）
        std::vector<cv::Point2f> ab_pts, bc_pts, cd_pts, da_pts;
        for (int i = 0; i <= 3; ++i) {
            float t = static_cast<float>(i) / 3.0f;
            ab_pts.push_back(lerpPoint(a, b, t)); // 上边
            bc_pts.push_back(lerpPoint(b, c, t)); // 右边
            cd_pts.push_back(lerpPoint(c, d, t)); // 下边
            da_pts.push_back(lerpPoint(d, a, t)); // 左边
        }

        // 4x4 网格点矩阵 grid[row][col]
        std::vector<std::vector<cv::Point2f>> grid(4, std::vector<cv::Point2f>(4));
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                // 横向插值：左边点到右边点
                float t_col = static_cast<float>(col) / 3.0f;
                grid[row][col] = lerpPoint(da_pts[3 - row], bc_pts[row], t_col);
            }
        }

        // ===================== 3. 在原图上绘制绿色网格线 =====================
        // 外框
        cv::line(img, a, b, cv::Scalar(0, 255, 0), 2);
        cv::line(img, b, c, cv::Scalar(0, 255, 0), 2);
        cv::line(img, c, d, cv::Scalar(0, 255, 0), 2);
        cv::line(img, d, a, cv::Scalar(0, 255, 0), 2);

        // 内部竖线（2条）
        for (int col = 1; col <= 2; ++col) {
            cv::line(img, grid[0][col], grid[3][col], cv::Scalar(0, 255, 0), 2);
        }
        // 内部横线（2条）
        for (int row = 1; row <= 2; ++row) {
            cv::line(img, grid[row][0], grid[row][3], cv::Scalar(0, 255, 0), 2);
        }

        // ===================== 4. 逐个截取9个小格子 =====================
        std::vector<cv::Mat> result;

        for (int row = 0; row < 3; ++row) {        // 从上到下
            for (int col = 0; col < 3; ++col) {    // 从左到右
                // 小四边形四个顶点（顺时针）
                cv::Point2f p1 = grid[row][col];       // 左上
                cv::Point2f p2 = grid[row][col + 1];   // 右上
                cv::Point2f p3 = grid[row + 1][col + 1]; // 右下
                cv::Point2f p4 = grid[row + 1][col];   // 左下

                // 计算目标矩形尺寸：上下边平均宽，左右边平均高
                double w_top = pointDistance(p1, p2);
                double w_bottom = pointDistance(p4, p3);
                double dst_width = (w_top + w_bottom) * 0.5;

                double h_left = pointDistance(p1, p4);
                double h_right = pointDistance(p2, p3);
                double dst_height = (h_left + h_right) * 0.5;

                int out_w = static_cast<int>(std::round(dst_width));
                int out_h = static_cast<int>(std::round(dst_height));
                out_w = std::max(1, out_w);
                out_h = std::max(1, out_h);

                // 透视变换源点
                std::vector<cv::Point2f> src_pts = {p1, p2, p3, p4};
                std::vector<cv::Point2f> dst_pts = {
                    cv::Point2f(0, 0),
                    cv::Point2f(static_cast<float>(out_w - 1), 0),
                    cv::Point2f(static_cast<float>(out_w - 1), static_cast<float>(out_h - 1)),
                    cv::Point2f(0, static_cast<float>(out_h - 1))
                };

                cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
                cv::Mat warp_img;
                cv::warpPerspective(roi, warp_img, M, cv::Size(out_w, out_h),
                                    cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

                // ===================== 5. 黑色填充成正方形 =====================
                int square_size = std::max(out_w, out_h);
                cv::Mat square_img = cv::Mat::zeros(square_size, square_size, warp_img.type());

                int offset_x = (square_size - out_w) / 2;
                int offset_y = (square_size - out_h) / 2;
                warp_img.copyTo(square_img(cv::Rect(offset_x, offset_y, out_w, out_h)));

                // ===================== 6. 统一resize到640x640 =====================
                cv::Mat final_img;
                cv::resize(square_img, final_img, cv::Size(640, 640), 0, 0, cv::INTER_LINEAR);

                result.push_back(final_img);
            }
        }

        return result;
    }

    };


}

#endif
