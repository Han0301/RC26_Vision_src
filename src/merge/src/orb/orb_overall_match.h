#ifndef __ORB_OVERALL_MATCH_H_
#define __ORB_OVERALL_MATCH_H_

#include "orb_match.h"


namespace Ten
{
    namespace ORB
    {
        
        class orb_optimize_match
        {
        public:
            /**
             * @brief 初始化函数
             * @param match_threshold：最小阈值
             */
            orb_optimize_match(const double share_threshold = 9.0)
            :SHARE_THRESHOLD_(share_threshold)
            {
                debug_image_ = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC3);
            }

            /**
             * @brief 设置调试图像
             * @param img: 输入调试图像
             */
            void set_debug_image(cv::Mat img)
            {
               img.copyTo(debug_image_);
            }

            /**
             * @brief 获取调试图像
             * @return cv::Mat: 调试图像
             */
            cv::Mat get_debug_img()
            {
                return debug_image_;
            }


            /**
             * @brief 按规则匹配orb点和yolo点
             * @param a orb_points列表（使用point_成员）
             * @param b cv::Point2d列表
             * @return loss损失均值
             */
            std::vector<match_points> get_optimize_match(const std::vector<orb_points>& a, const std::vector<cv::Point2d>& b)
            {
                orb_match_points_ = matchOrbAndYoloPoints(a,b);
                orb_optimize_match_points_ = globalOptimalMatch(orb_match_points_);
                drawMatchedPointsLines(debug_image_, orb_optimize_match_points_);
                return orb_optimize_match_points_;
            }

        private:
        double SHARE_THRESHOLD_;
        std::vector<match_points> orb_match_points_;
        std::vector<match_points> orb_optimize_match_points_;
        cv::Mat debug_image_;


            /**
             * @brief 匹配函数
             * @param a: const std::vector<orb_points>
             * @param b: const std::vector<cv::Point2d>
             * @return std::vector<match_points>: 匹配结果
             */
            std::vector<match_points> matchOrbAndYoloPoints(const std::vector<orb_points>& a, const std::vector<cv::Point2d>& b)
            {

                if(a.empty() || b.empty())
                {
                    return std::vector<match_points>();
                }
                //匹配结果
                std::vector<match_points> match_result;
                //每个a点与每个b点的损失
                std::vector<std::vector<CandidatePair>> all_loss_match;

                all_loss_match.resize(a.size());
                for(size_t i = 0; i < a.size(); i++)
                {
                    all_loss_match[i].resize(b.size());
                }

                //遍历每个a
                for(size_t i = 0; i < all_loss_match.size(); i++)
                {
                    //遍历每个b
                    for(size_t j = 0; j < all_loss_match[i].size(); j++)
                    {
                        CandidatePair c;
                        //计算损失
                        const cv::Point2d& ap = a[i].point_;
                        const cv::Point2d& bp = b[j];

                        double dx = ap.x - bp.x;
                        double dy = ap.y - bp.y;

                        c.a_idx = i;
                        c.b_idx = j;
                        c.loss = dx*dx + dy*dy;

                        all_loss_match[i][j] = c;
                    }
                    // 按损失升序排序当前a点的候选列表
                    std::sort(all_loss_match[i].begin(), all_loss_match[i].end());
                }

                //用来记录匹配信息
                std::vector<bool> a_match;
                a_match.resize(a.size(), false);
                std::vector<bool> b_match;
                b_match.resize(b.size(), false);
                int a_not_match_num = a.size();
                int b_not_match_num = b.size();


                //现匹配阈值以下的，共享关系
                for(size_t i = 0; i < all_loss_match.size(); i++)
                {
                    CandidatePair c = all_loss_match[i][0];
                    if(c.loss < SHARE_THRESHOLD_)
                    {
                        match_points mp;
                        //b没被匹配过
                        if(b_match[c.b_idx] == false)
                        {
                            //标记配准过的b点
                            b_match[c.b_idx] = true;
                            b_not_match_num--;
                        }
                        //保存匹配结果
                        mp.loss_ = c.loss;
                        mp.project_point_ = a[c.a_idx];
                        mp.yolo_point_ = b[c.b_idx];
                        match_result.push_back(mp);
                        // std::cout<< "a: " << a[c.a_idx].id_ + 1 << std::endl;
                        // std::cout<< "loss: " << c.loss << std::endl;
                        //绘制调试图像
                        //cv::line(debug_image_, mp.project_point_.point_, mp.yolo_point_, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
                        //标记配准过的a点
                        a_match[c.a_idx] = true;
                        a_not_match_num--;
                    }
                }

                //判断是否已经全部配准
                if(a_not_match_num <= 0 || b_not_match_num <= 0)
                {
                    return match_result;
                }



                //匹配竞争关系
                //展平没有配准的a点
                std::vector<CandidatePair> all_less_match;
                for(size_t i = 0; i < all_loss_match.size(); i++)
                {
                    //没有配准的a
                    if(a_match[i] == false)
                    {
                        for(size_t j = 0; j < all_loss_match[i].size(); j++)
                        {
                            all_less_match.push_back(all_loss_match[i][j]);
                        }
                    }
                }

                // 按损失升序排序当前a点的候选列表
                std::sort(all_less_match.begin(), all_less_match.end());

                for(size_t i = 0; i < all_less_match.size(); i++)
                {
                    CandidatePair c = all_less_match[i];
                    //如果对应的a和b都没有配准过
                    if(b_match[c.b_idx] == false && a_match[c.a_idx] == false)
                    {
                        match_points mp;
                        //保存匹配结果
                        mp.loss_ = c.loss;
                        mp.project_point_ = a[c.a_idx];
                        mp.yolo_point_ = b[c.b_idx];
                        match_result.push_back(mp);

                        // std::cout<< "a: " << a[c.a_idx].id_ + 1 << std::endl;
                        // std::cout<< "loss: " << c.loss << std::endl;
                        // std::cout << "a3dpoint: " << a[c.a_idx].point_last_ << std::endl;
                        //绘制调试图像
                        //cv::line(debug_image_, mp.project_point_.point_, mp.yolo_point_, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

                        //记录剩余点数
                        b_match[c.b_idx] = true;
                        a_match[c.a_idx] = true;
                        a_not_match_num--;
                        b_not_match_num--;
                    }
                    //判断是否已经全部配准
                    if(a_not_match_num <= 0 || b_not_match_num <= 0)
                    {
                        return match_result;
                    }
                }

                return match_result;
            }





            /**
             * @brief 匈牙利算法（KM实现）：求解二分图的最大权匹配（适配最小损失匹配）
             * @param cost_matrix：输入的代价矩阵（n行m列，元素为匹配损失（距离平方））
             * @param match_A_to_B：输出的匹配结果（A→B的下标映射，-1无匹配）
             * @param min_total_loss：输出的最小总匹配损失（所有有效匹配的代价之和）
             * @return bool：true匹配成功，false输入为空
             */
            bool hungarianAlgorithm(const std::vector<std::vector<double>>& cost_matrix,
                                    std::vector<int>& match_A_to_B,
                                    double& min_total_loss)
            {
                int n = cost_matrix.size();
                if (n == 0) return false;
                int m = cost_matrix[0].size();

                match_A_to_B.assign(n, -1);
                std::vector<int> match_B_to_A(m, -1);
                std::vector<double> u(n + 1, 0);  // A集合顶标
                std::vector<double> v(m + 1, 0);  // B集合顶标
                std::vector<bool> vis_A(n + 1, false);
                std::vector<bool> vis_B(m + 1, false);
                std::vector<double> slack(m + 1, std::numeric_limits<double>::max());

                // 关键修改1：构建权值矩阵（损失取反，将最小损失转为最大权）
                std::vector<std::vector<double>> weight_matrix(n, std::vector<double>(m, 0.0));
                for (int a = 0; a < n; ++a)
                {
                    for (int b = 0; b < m; ++b)
                    {
                        weight_matrix[a][b] = -cost_matrix[a][b]; // 损失取反
                    }
                }

                // 关键修改2：初始化A集合顶标u为该行最大权值（KM算法必须）
                for (int a = 0; a < n; ++a)
                {
                    u[a] = *std::max_element(weight_matrix[a].begin(), weight_matrix[a].end());
                }

                // 深度优先搜索找增广路
                auto dfs = [&](auto&& self, int a) -> bool {
                    vis_A[a] = true;
                    for (int b = 0; b < m; ++b)
                    {
                        if (vis_B[b]) continue;

                        // 核心条件：基于取反后的权值计算gap
                        double gap = u[a] + v[b] - weight_matrix[a][b];
                        if (std::fabs(gap) < 1e-9)
                        {
                            vis_B[b] = true;
                            if (match_B_to_A[b] == -1 || self(self, match_B_to_A[b]))
                            {
                                match_A_to_B[a] = b;
                                match_B_to_A[b] = a;
                                return true;
                            }
                        }
                        else
                        {
                            slack[b] = std::min(slack[b], gap);
                        }
                    }
                    return false;
                };

                // 为每个A元素找最优匹配
                for (int a = 0; a < n; ++a)
                {
                    std::fill(slack.begin(), slack.end(), std::numeric_limits<double>::max());
                    while (true)
                    {
                        std::fill(vis_A.begin(), vis_A.end(), false);
                        std::fill(vis_B.begin(), vis_B.end(), false);

                        if (dfs(dfs, a)) break;

                        // 计算顶标调整量delta
                        double delta = std::numeric_limits<double>::max();
                        for (int b = 0; b < m; ++b)
                        {
                            if (!vis_B[b]) delta = std::min(delta, slack[b]);
                        }

                        // 调整顶标（逻辑不变，基于权值矩阵）
                        for (int i = 0; i < n; ++i)
                        {
                            if (vis_A[i]) u[i] -= delta;
                        }
                        for (int j = 0; j < m; ++j)
                        {
                            if (vis_B[j]) v[j] += delta;
                            else slack[j] -= delta;
                        }
                    }
                }

                // 关键修改3：计算最小总损失（将最大权结果取反）
                double max_total_weight = 0.0;
                for (int a = 0; a < n; ++a)
                {
                    if (match_A_to_B[a] != -1)
                    {
                        max_total_weight += weight_matrix[a][match_A_to_B[a]];
                    }
                }
                min_total_loss = -max_total_weight; // 取反还原为最小损失

                return true;
            }

            /**
             * @brief 基于匈牙利算法的全局最优匹配（损失=距离平方，确保最小损失）
             * @param orb_match_points 输入：原始点对集合（只读）
             * @return 输出：仅包含全局最优（损失最小）匹配的点对集合
             */
            std::vector<match_points> globalOptimalMatch(const std::vector<match_points>& orb_match_points)
            {
                std::vector<orb_points> valid_A;   // 有效ORB投影点
                std::vector<cv::Point2d> valid_B;  // 有效YOLO检测点

                // 过滤无效点
                for (const auto& pair : orb_match_points)
                {
                    if (pair.project_point_.point_.x == -1 || pair.project_point_.point_.y == -1 ||
                        pair.yolo_point_.x == -1 || pair.yolo_point_.y == -1)
                    {
                        continue;
                    }
                    valid_A.push_back(pair.project_point_);
                    valid_B.push_back(pair.yolo_point_);
                }

                int n = valid_A.size();
                int m = valid_B.size();
                if (n == 0 || m == 0)
                {
                    return {};
                }

                // 构建代价矩阵（距离平方，损失）
                std::vector<std::vector<double>> cost_matrix(n, std::vector<double>(m, 0.0));
                for (int a = 0; a < n; ++a)
                {
                    for (int b = 0; b < m; ++b)
                    {
                        double dx = valid_A[a].point_.x - valid_B[b].x;
                        double dy = valid_A[a].point_.y - valid_B[b].y;
                        cost_matrix[a][b] = dx * dx + dy * dy; // 损失=距离平方
                    }
                }

                // 调用修正后的匈牙利算法
                std::vector<int> match_A_to_B;
                double min_total_loss = 0.0;
                if (!hungarianAlgorithm(cost_matrix, match_A_to_B, min_total_loss))
                {
                    return {};
                }

                // 构建最优匹配结果
                std::vector<match_points> optimal_matches;
                for (int a = 0; a < n; ++a)
                {
                    int b_idx = match_A_to_B[a];
                    if (b_idx != -1)
                    {
                        match_points optimal_pair;
                        optimal_pair.project_point_ = valid_A[a];
                        optimal_pair.yolo_point_ = valid_B[b_idx];
                        optimal_pair.loss_ = cost_matrix[a][b_idx]; // 保留原始损失值
                        optimal_matches.push_back(optimal_pair);
                    }
                }

                return optimal_matches;
            }


            /**
             * @brief 绘制匹配点对之间的连线
             * @param img 绘制的目标图像（会直接在该图像上绘制）
             * @param match_list 匹配点对列表
             * @param line_color 连线颜色（默认红色）
             * @param line_thickness 线宽（默认2）
             */
            void drawMatchedPointsLines(cv::Mat& img, 
                                    const std::vector<match_points>& match_list,
                                    const cv::Scalar& line_color = cv::Scalar(255, 255, 255),
                                    int line_thickness = 1)
            {
                // 遍历所有匹配点对
                for (const auto& match : match_list)
                {
                    // 获取两个需要连线的2D点
                    cv::Point2d orb_point = match.project_point_.point_;
                    cv::Point2d yolo_point = match.yolo_point_;
                    
                    // 绘制两点之间的线段
                    cv::line(img, orb_point, yolo_point, line_color, line_thickness);
                }
            }

        };



        class orb_optimize_rt
        {
        public:
            Ten_camerainfo camera_info_;

            /**
             * @brief 优化内参
             * @param optimal_matches: 优化后的点对
             * @return bool: 是否优化成功
             */
            bool optimize_rt(const std::vector<match_points>& optimal_matches)
            {
                cv::Mat rvec;
                cv::Mat tvec;
                if(pnp(optimal_matches, camera_info_.K(), camera_info_.distCoeffs(), rvec, tvec))
                {
                    camera_info_.set_RT(rvec, tvec);
                    return true;
                }
                return false;
            }
        private:

            /**
             * @brief pnp优化
             * @param optimal_matches: 优化后的点对
             * @param K: 相机投影矩阵
             * @param distCoeffs: 相机畸变系数
             * @param _rvec: 旋转向量
             * @param _tvec: 平移向量
             * @return bool: 是否成功
             */
            bool pnp(const std::vector<match_points>& optimal_matches, const cv::Mat K, const cv::Mat distCoeffs, cv::Mat& _rvec, cv::Mat& _tvec)
            {
                //如果小于4对点匹配失败
                if(optimal_matches.size() < 6)
                {
                    return false;
                }

                std::vector<cv::Point3d> objectPoints; 
                std::vector<cv::Point2d> imagePoints;

                //插入数据
                for(size_t i = 0; i < optimal_matches.size(); i++)
                {
                    objectPoints.push_back(optimal_matches[i].project_point_.point_last_);
                    imagePoints.push_back(optimal_matches[i].yolo_point_);
                }

                cv::Mat rvec, tvec;  // 输出：旋转向量（3x1）、平移向量（3x1）
                std::vector<int> inliers; // 输出：RANSAC筛选出的内点索引

                cv::solvePnPRansac(objectPoints, imagePoints, K, distCoeffs,
                    rvec, tvec, 
                    false,        // 是否使用初始猜测
                    100,          // RANSAC迭代次数
                    5.0,          // 重投影误差阈值（核心RANSAC参数）
                    0.99,         // 置信度（99%概率找到最优解）
                    inliers,      // 输出内点索引
                    cv::SOLVEPNP_EPNP); 

                
                std::vector<cv::Point3d> objectPoints2; 
                std::vector<cv::Point2d> imagePoints2;

                //插入数据 （内点）
                for(int idx : inliers)
                {
                    objectPoints2.push_back(objectPoints[idx]);
                    imagePoints2.push_back(imagePoints[idx]);
                }

                if(objectPoints2.size() < 6)
                {
                    return false;
                }

                cv::Mat rvec2, tvec2;  // 输出：旋转向量（3x1）、平移向量（3x1）
                cv::solvePnP(objectPoints2, imagePoints2, K, distCoeffs, rvec2, tvec2);
                rvec2.copyTo(_rvec);
                tvec2.copyTo(_tvec);

                return true;

            }
            
        };



    } 
}

#endif 
