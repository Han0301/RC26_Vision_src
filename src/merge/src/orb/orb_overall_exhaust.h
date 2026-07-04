#ifndef __ORB_OVERALL_EXHAUST_H_
#define __ORB_OVERALL_EXHAUST_H_

#include "orb_exhaust.h"
#include "orb_overall_match.h"

namespace Ten
{
    namespace ORB
    {
        
        class orb_optimize_exhaust
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
            orb_optimize_exhaust(const std::string model_path, const std::string xpu, float conf_thres = 0.75, float cls_thres = 0.75, float iou = 0)
            :og_(model_path, xpu, conf_thres, cls_thres, iou)
            {
                place_.resize(12,0);
                loss_.resize(5,0.0);
            }

            /**
             * @brief 获取最可能的摆放方式
             * @param oees: 提供推理要素
             * @return std::vector<int>: 最可能摆放方式
             */
            std::vector<int> getplace(std::vector<orb_exhaust_element> oees)
            {
                if(oees.empty())
                {
                    return place_;
                }
                std::vector<std::vector<double>> losses;
                std::vector<std::vector<int>> all_places = generateAllBoxCombinations();
                //std::cout<< "all_place_num: " << all_places.size() << std::endl;
                losses.resize(oees.size());
                for(size_t i = 0; i < losses.size(); i++)
                {
                    losses[i].resize(all_places.size(),0);
                }
                for(size_t i = 0; i < oees.size(); i++)
                {
                    for(size_t j = 0; j < all_places.size(); j++)
                    {
                        int exist_box[12] = {0};
                        for(size_t k = 0; k < 12 && k < all_places[j].size(); k++)
                        {
                            exist_box[k] = all_places[j][k];
                        }

                        // for(int test = 0; test < 12; test++)
                        // {
                        //     std::cout << exist_box[test] << " ";
                        // }
                        // std::cout << std::endl;

                        //设置存在的方块
                        og_.set_exist_boxes(exist_box);
                        //设置rt,生成点对
                        og_.generate_points(oees[i].rvec_, oees[i].tvec_, oees[i].image_, j);
                        //优化rt,并判断是否成功
                        if(oort_.optimize_rt(oom_.get_optimize_match(og_.getPixelPoints(), og_.getyoloPoints())))
                        {
                            //重新生成点对
                            og_.generate_points(oort_.camera_info_.rvec(), oort_.camera_info_.tvec(), oees[i].image_, 1);
                        }
                        //计算损失
                        losses[i][j] = om_.getloss(og_.getPixelPoints(), og_.getyoloPoints());
                    }
                }
                std::vector<double> total_losses;
                total_losses.resize(all_places.size(), 0);
                for(size_t i = 0; i < total_losses.size(); i++)
                {
                    for(size_t j = 0; j < losses.size(); j++)
                    {
                        total_losses[i] += losses[j][i];
                    }
                }

                // 找到指向最小值的迭代器
                // std::min_element 返回容器中第一个最小值的迭代器
                auto min_it = std::min_element(total_losses.begin(), total_losses.end());
                

                // 计算迭代器对应的索引（从0开始）
                // std::distance 计算两个迭代器之间的距离，即索引值
                int min_index = std::distance(total_losses.begin(), min_it);
                std::cout<< "min_loss: " << total_losses[min_index] << std::endl;

                std::sort(total_losses.begin(), total_losses.end());

                for(size_t i = 0; i < total_losses.size() && i < 5; i++)
                {
                    loss_[i] = total_losses[i];
                    //std::cout << "top" << i << ": " << total_losses[i] << std::endl;
                }

                
                place_ = all_places[min_index];
                return place_;
            }


            /**
             * @brief 获取前5个损失
             * @return std::vector<double>: 前5个损失
             */
            std::vector<double> get_loss()
            {
                return loss_;
            }

            /**
             * @brief 获取优化后的rt向量
             * @param oees: 提供推理要素
             * @return std::vector<orb_exhaust_element>: 优化后的RT
             */
            std::vector<orb_exhaust_element> get_RT(std::vector<orb_exhaust_element> oees)
            {
                get_optimize_RT(oees, place_);
                return optimize_oees_;
            }


        private:
            orb_optimize_match oom_;
            orb_optimize_rt oort_;
            orb_getpoint og_;
            orb_match om_;
            std::vector<int> place_; //方块摆放顺序
            std::vector<double> loss_; //前5个损失值
            std::vector<orb_exhaust_element> optimize_oees_; //优化过后的rt;

            /**
             * @brief 生成由8个1和4个0组成的长度为12的int数组的所有不重复组合
             * @return std::vector<std::vector<int>> 所有组合的集合，每个子vector长度为12，含8个1、4个0
             */
            std::vector<std::vector<int>> generateAllBoxCombinations()
            {
                // 步骤1：初始化基础数组（4个0 + 8个1，总长度12）
                std::vector<int> base_arr(4, 0);       // 先创建4个0
                base_arr.insert(base_arr.end(), 8, 1); // 再追加8个1

                // 步骤2：存储所有不重复组合的结果容器
                std::vector<std::vector<int>> all_combinations;

                // 步骤3：生成所有不重复的排列（next_permutation自动跳过重复排列）
                do {
                    all_combinations.push_back(base_arr); // 将当前排列加入结果
                } while (std::next_permutation(base_arr.begin(), base_arr.end()));

                return all_combinations;
            }

            /**
             * @brief 获取优化后的rt向量
             * @param oees: 提供推理要素
             * @param place: 方块摆放位置
             */
            void get_optimize_RT(std::vector<orb_exhaust_element>& oees, std::vector<int>& place)
            {
                optimize_oees_.clear();
                int exist_box[12] = {0};
                for(size_t k = 0; k < 12 && k < place.size(); k++)
                {
                    exist_box[k] = place[k];
                }

                for(size_t i = 0; i < oees.size(); i++)
                {
                    orb_exhaust_element oee;
                    //设置存在的方块
                    og_.set_exist_boxes(exist_box);
                    //设置rt,生成点对
                    // std::cout<< "r: " << oees[i].rvec_ << std::endl;
                    // std::cout<< "t: " << oees[i].tvec_ << std::endl;
                    // cv::imshow("test", oees[i].image_);
                    // cv::waitKey(0);
                    og_.generate_points(oees[i].rvec_, oees[i].tvec_, oees[i].image_);
                    //优化rt,并判断是否成功
                    if(oort_.optimize_rt(oom_.get_optimize_match(og_.getPixelPoints(), og_.getyoloPoints())))
                    {
                        oees[i].image_.copyTo(oee.image_);
                        oort_.camera_info_.rvec().copyTo(oee.rvec_);
                        oort_.camera_info_.tvec().copyTo(oee.tvec_);
                        // std::cout<< "r: " << oee.rvec_ << std::endl;
                        // std::cout<< "t: " << oee.tvec_ << std::endl;
                        optimize_oees_.push_back(oee);
                        //std::cout << i << " true" << std::endl;
                    } 
                    else
                    {
                        optimize_oees_.push_back(oees[i]);
                    }
                }
            }
        };

    } 
}

#endif 
