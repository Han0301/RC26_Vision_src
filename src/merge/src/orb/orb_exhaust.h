#ifndef __ORB_EXHAUST_H_
#define __ORB_EXHAUST_H_

#include "orb_match.h"


namespace Ten
{
    namespace ORB
    {

        struct orb_exhaust_element
        {
            // 默认构造函数：创建实例时自动初始化所有成员
            orb_exhaust_element()
            {
                // 1. 初始化旋转向量：3行1列，64位浮点型，全0
                rvec_ = cv::Mat::zeros(3, 1, CV_64F);
                // 2. 初始化平移向量：3行1列，64位浮点型，全0
                tvec_ = cv::Mat::zeros(3, 1, CV_64F);
                // 3. 初始化1920×1080彩色图像（全黑）：Size(宽度, 高度)
                // 若需灰度图，将 CV_8UC3 改为 CV_8UC1 即可
                image_ = cv::Mat::zeros(cv::Size(1920, 1080), CV_8UC3);
            }
            cv::Mat rvec_;
            cv::Mat tvec_;
            cv::Mat image_;
        };

        struct debug_orb_exhaust_element
        {
            // 默认构造函数：创建实例时自动初始化所有成员
            debug_orb_exhaust_element()
            {
                //resize为12
                label.resize(12,0);
            }
            orb_exhaust_element oee;
            std::vector<int> label;
        };
        
        class orb_exhaust
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
            orb_exhaust(const std::string model_path, const std::string xpu, float conf_thres = 0.75, float cls_thres = 0.75, float iou = 0)
            :og_(model_path, xpu, conf_thres, cls_thres, iou)
            {
                place_.resize(12,0);
                loss_.resize(5,0.0);
            }

            /**
             * @brief 获取最可能的摆放方式
             * @param oe: 提供推理要素
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
                        for(int k = 0; k < 12 && k < all_places[j].size(); k++)
                        {
                            exist_box[k] = all_places[j][k];
                        }

                        // for(int test = 0; test < 12; test++)
                        // {
                        //     std::cout << exist_box[test] << " ";
                        // }
                        // std::cout << std::endl;


                        og_.set_exist_boxes(exist_box);
                        og_.generate_points(oees[i].rvec_, oees[i].tvec_, oees[i].image_, j);
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

        private:
            orb_getpoint og_;
            orb_match om_;
            std::vector<int> place_; //方块摆放顺序
            std::vector<double> loss_; //前5个损失值


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
        };

        /**
         * @brief 加载数据集主函数
         * @param root_dir 总文件夹路径（如 "test"）
         * @return 外层vector：子文件夹1/2/3...；内层vector：每个文件对应的元素
         */
        std::vector<std::vector<debug_orb_exhaust_element>> load_exhaust_dataset(const std::string& root_dir);

        // ====================== 核心函数 ======================
        /**
         * @brief 生成JSON文件并写入loss和label
         * @param root_dir 主文件夹路径（如 test）
         * @param i 文件名序号，最终生成 i.json
         * @param label 标签向量，写入JSON的 "label" 字段
         * @param loss 损失值，写入JSON的 "loss" 字段
         * @throw std::runtime_error 文件夹创建/文件写入失败时抛出异常
         */
        void save_loss_label_to_json(const std::string& root_dir, int i, const std::vector<int>& label, double loss);

    } 
}

#endif 
