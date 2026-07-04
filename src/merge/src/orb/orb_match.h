#ifndef __ORB_MATCH_H_
#define __ORB_MATCH_H_

#include "orb_getpoint.h"


namespace Ten
{
    namespace ORB
    {
        
        struct match_points
        {
            orb_points project_point_;
            cv::Point2d yolo_point_ = cv::Point2d(-1,-1); 
            double loss_ = 0;
        };

        // 候选配对结构体：存储a索引、b索引、损失值
        struct CandidatePair
        {
            int a_idx;    // a列表的索引
            int b_idx;    // b列表的索引
            double loss;  // 距离平方（损失）

            // 重载排序运算符：损失小的优先
            bool operator<(const CandidatePair& other) const
            {
                return loss < other.loss;
            }
        };


        class orb_match
        {
        public:
            /**
             * @brief 初始化函数
             * @param match_threshold：最小阈值
             */
            orb_match(const double share_threshold = 9.0)
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
            double getloss(const std::vector<orb_points>& a, const std::vector<cv::Point2d>& b)
            {
                orb_match_points_ = matchOrbAndYoloPoints(a,b);
                // std::cout<< "a.size(): " << a.size() << std::endl;
                // std::cout<< "b.size(): " << b.size() << std::endl;
                // std::cout<< "orb_match_points_.size(): " << orb_match_points_.size() << std::endl;
                //mean_loss_ = calculateMeanLossWithGaussianFilter(orb_match_points_);
                mean_loss_ = calculateMeanLossDirectly(orb_match_points_);
                return mean_loss_;
            }
            

        private:
            // 匹配阈值（距离平方）
            double SHARE_THRESHOLD_;
            std::vector<match_points> orb_match_points_;
            double mean_loss_ = 0.0;
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
                        cv::line(debug_image_, mp.project_point_.point_, mp.yolo_point_, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
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
                        cv::line(debug_image_, mp.project_point_.point_, mp.yolo_point_, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);

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
             * @brief 基于高斯分布（3σ原则）剔除loss噪声，计算过滤后的平均值
             * @param matches 匹配结果列表（包含loss_字段）
             * @return 过滤噪声后loss的平均值（无有效数据返回0.0）
             */
            double calculateMeanLossWithGaussianFilter(const std::vector<match_points>& matches)
            {
                // 步骤1：提取所有有效loss值（过滤默认值0和负数，仅保留正的损失值）
                std::vector<double> valid_losses;
                for (const auto& mp : matches)
                {
                    // loss_=0通常是未配对的默认值，负数无意义，均过滤
                    if (mp.loss_ > 0)
                    {
                        valid_losses.push_back(mp.loss_);
                    }
                }

                // 边界条件1：无有效loss值，直接返回DBL_MAX
                if (valid_losses.empty())
                {
                    std::cerr << "警告：无有效loss值，返回最大值DBL_MAX\n";
                    return DBL_MAX / 5.0;//防止后续运算溢出
                }

                // 边界条件2：仅1个有效loss值，无噪声可剔除，直接返回该值
                if (valid_losses.size() == 1)
                {
                    return valid_losses[0];
                }

                // 步骤2：计算有效loss的均值（μ）
                // accumulate：累加容器内所有元素，初始值0.0
                double mean = std::accumulate(valid_losses.begin(), valid_losses.end(), 0.0) / valid_losses.size();

                // 步骤3：计算标准差（σ）
                double squared_sum = 0.0;
                for (double loss : valid_losses)
                {
                    squared_sum += std::pow(loss - mean, 2); // 累加（loss-均值）的平方
                }
                // 样本标准差（除以n，若需无偏估计可除以n-1，此处用n更贴合工程场景）
                double std_dev = std::sqrt(squared_sum / valid_losses.size());

                // 步骤4：应用3σ原则过滤噪声（保留 [μ-3σ, μ+3σ] 范围内的loss）
                std::vector<double> filtered_losses;
                double lower_bound = mean - 3 * std_dev; // 下界（μ-3σ）
                double upper_bound = mean + 3 * std_dev; // 上界（μ+3σ）
                for (double loss : valid_losses)
                {
                    // 仅保留在3σ范围内的loss（下界取0，避免负数，因为loss不可能为负）
                    // if (loss >= std::max(0.0, lower_bound) && loss <= upper_bound)
                    // {
                    //     filtered_losses.push_back(loss);
                    // }
                    // 只过滤上届
                    if (loss < upper_bound)
                    {
                        filtered_losses.push_back(loss);
                    }
                }

                // 边界条件3：过滤后无有效值，返回原始均值（避免空容器计算）
                if (filtered_losses.empty())
                {
                    std::cerr << "警告：3σ过滤后无有效loss值，返回原始均值\n";
                    return mean;
                }

                // 步骤5：计算过滤后loss的平均值
                double filtered_mean = std::accumulate(filtered_losses.begin(), filtered_losses.end(), 0.0) / filtered_losses.size();

                return filtered_mean;
            }

            /**
             * @brief 直接计算match_points列表中有效loss的平均值（无噪声过滤）
             * @param matches 匹配结果列表，loss_>0为有效损失值
             * @return 有效loss的平均值；无有效数据时返回0.0
             */
            double calculateMeanLossDirectly(const std::vector<match_points>& matches)
            {
                // 步骤1：提取所有有效损失值（过滤默认值0，负数无物理意义也过滤）
                std::vector<double> valid_losses;
                for (const auto& mp : matches)
                {
                    if (mp.loss_ > 0)  // 仅保留正的损失值（有效配对的损失）
                    {
                        valid_losses.push_back(mp.loss_);
                        //std::cout<< "loss: " << mp.loss_ << std::endl;
                    }
                }

                // 步骤2：处理边界条件——无有效损失值时返回0.0
                if (valid_losses.empty())
                {
                    std::cerr << "提示：无有效loss值（所有loss为0或负数），返回最大值DBL_MAX\n";
                    return DBL_MAX / 5.0;
                }

                // 步骤3：计算有效loss的总和（std::accumulate高效累加，初始值为0.0）
                double total_loss = std::accumulate(valid_losses.begin(), valid_losses.end(), 0.0);
                
                // 步骤4：计算平均值（总和 / 有效数据数量）
                double mean_loss = total_loss / valid_losses.size();

                return mean_loss;
            }

        };

    } 
}

#endif 
