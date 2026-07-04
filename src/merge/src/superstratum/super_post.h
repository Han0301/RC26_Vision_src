#ifndef _SUPEER_POST_
#define _SUPPER_POST_

#include "super.h"
#include "./../yolo/yolo_han2.h"

namespace Ten
{
namespace superstratum
{
class super_post
{
public:
    /**
     * 最终的后处理， 请确保已经调用set_roi12_place 和 set_cls， 结合筛空和分类模型的结果给出最终结果
     * 设置 final_result
     * @param classifier   分类模型的分类类别
     * @param confidence   分类模型的置信度
     * @param place         各个位置是否有方块
     * @param per_loss     每个位置筛空的损失
     * @param is_count_infer 是否采用数量约束进行推理
     * @param is_debug_print 对填充结果过程的打印调试
     * @param limit_count   由1,2,3,4类数量上限拼接成的约束列表
    */
    void set_final_result(
        const std::vector<int> classifier,       
        const std::vector<float> confidence,
        const std::vector<int> place,
        const std::vector<float> per_loss = {},
        const bool is_count_infer = true,
        const bool is_debug_print = false,
        const std::vector<int> limit_count = {_limit_count_1_, _limit_count_2_, _limit_count_3_, _limit_count_4_}
    )
    {
        if (classifier.size() != 12 || confidence.size() != 12 || place.size() != 12 || limit_count.size() != 4)
        {
            std::cout << "in func set_post_cls: classifier.size() != 12 || confidence.size() != 12 || place.size() != 12 || limit_count.size() != 4!!!" << std::endl;
            return;
        }

        std::vector<int> now_count = {0,0,0,0};     // 每个类别确定的数量
        std::vector<int> is_process(12, -1);         // 是否处理的标志位， -1表示未处理未填充， 0 表示已处理但未填充， 1 表示已处理已填充
        final_result.assign(12, 0);

        // ℹ 首次直接填充  
        for (int i = 0; i < classifier.size(); i++)
        {
            // 1 高置信度 && 两个模型输出一致表示有方块 && 对应位置有空位
            if (classifier[i] != 4 && std::abs(confidence[i] - 1) < 0.02 && place[i] != 0
                && now_count[ classifier[i] - 1] < limit_count[ classifier[i] - 1])     
            {
                final_result[i] = classifier[i];
                now_count[ final_result[i] - 1] += 1;
                is_process[i] = 1;
                if (is_debug_print)
                {
                    std::cout << "in i 1 fill_place: " << i + 1 << ", result: " << final_result[i] << std::endl;
                }
            }
            // 2 高置信度 && 两个模型输出一致表示无方块 && 对应位置有空位
            if(classifier[i] == 4 && std::abs(confidence[i] - 1) < 0.02 && place[i] == 0 
                && now_count[ classifier[i] - 1] < limit_count[ classifier[i] - 1])        
            {
                final_result[i] = 4;
                now_count[ final_result[i] - 1] += 1;
                is_process[i] = 1;
                if (is_debug_print)
                {
                    std::cout << "in i 2 fill_place: " << i + 1 << ", result: " << final_result[i] << std::endl;
                }
            }                    
        }

        // ℹℹ 非极高置信度， 依次取当前最高进行处理
        while (Ten::_TREADPOOL_FLAG_.read_flag())
        {
            if (now_count == limit_count)
            {
                break;
            }
            // 1. 找到本次填充的最大置信度
            int max_place = -1;
            float max_conf = 0.0f;
            int isnt_filled_count = 0;      // 本次 未填充的数量
            // 1.1 优先考虑未处理 的最大置信度
            for (int i =0; i < classifier.size(); i++)
            {
                if (final_result[i] == 0 && is_process[i] == -1)
                {
                    isnt_filled_count += 1;
                    if (confidence[i] > max_conf)
                    {
                        max_conf = confidence[i];
                        max_place = i;
                    }
                }
            }
            // 1.2 若 未处理的数量为0， max_place未赋值， 为-1， 此时考虑已经处理但未填充
            if (max_place == -1)
            {
                for (int i =0; i < classifier.size(); i++)
                {
                    if (final_result[i] == 0 && is_process[i] == 0)
                    {
                        isnt_filled_count += 1;
                        if (confidence[i] > max_conf)
                        {
                            max_conf = confidence[i];
                            max_place = i;
                        }
                    }
                }
            }
            // 1.3 全为 已处理已填充 直接退出
            if (is_debug_print)
            {
                std::cout << "in ii 1: max_place: " << max_place + 1 << ", isnt_filled_count:  " << isnt_filled_count << ",cls: " << classifier[max_place] << std::endl;
            }
            if (max_place == -1) break;
            int max_cls = classifier[max_place];

            // 2.1 两个模型输出一致 且 填充位置没有到对应位置上限， 直接填充
            if (max_cls != 4 && place[max_place] == 1 && now_count[ max_cls - 1 ] < limit_count[ max_cls - 1 ])
            {
                final_result[max_place] = max_cls;
                now_count[ max_cls - 1 ] += 1;
                is_process[max_place] = 1;
                if (is_debug_print)
                {
                    std::cout << "in ii 2 fill_place: " << max_place + 1 << ", result: " << final_result[max_place] << std::endl;
                }
                continue;
            }
            else if (!per_loss.empty())
            // 2.2 模型输出不一致 或者 填充位置到达对应位置上限
            /**
             * 对于模型输出结果不一致： 有以下几种情况
             *  model1(分类)      model2（筛空）
             *   1，2，3(损失小)    4            直接确认
             *   1，2，3           4(损失小)     直接确认
             *   4(损失小)         非4           直接确认
             *   4                非4(损失小)    无法确认
            */
            {
                // 2.2.1 model1 损失小，且为空类， 直接确认结果
                if (1.0f - max_conf < per_loss[max_place] && classifier[max_place] == 4
                    && now_count[ max_cls - 1 ] < limit_count[ max_cls - 1 ])
                {
                    final_result[max_place] = max_cls;
                    now_count[ max_cls - 1 ] += 1;
                    is_process[max_place] = 1;
                    continue;
                }
                // 2.2.2 model2 损失小，并且为空类 直接确认结果          尽可能相信筛空模型对空的结果
                if (1.0f - max_conf > per_loss[max_place] && place[max_place] == 0 
                    && now_count[ 3 ] < limit_count[ 3 ])
                {
                    final_result[max_place] = 4;
                    now_count[ 3 ] += 1;
                    is_process[max_place] = 1;
                    continue;
                }
            }

            // 3 本次无法确认 或者 本次填入的类别已经满了, 但仅剩一个未填充， 直接根据数量关系填充
            if (is_count_infer && isnt_filled_count == 1)
            {                    
                for (int i = 0;i < now_count.size(); i++)
                {
                    if (limit_count[i] - now_count[i] == isnt_filled_count)
                    {
                        final_result[max_place] = i + 1;
                        now_count[i] += 1;
                        is_process[max_place] = 1;
                        if (is_debug_print)
                        {
                            std::cout << "in ii 3 fill_place: " << max_place + 1 << ", result: " << final_result[max_place] << std::endl;
                        }
                        continue;
                    }
                }
                
            }

            // 4 走到这就 给了
            if (is_process[max_place] != 0)
            {
                is_process[max_place]  = 0;     // 0 表示已处理但未填充
                continue;
                if (is_debug_print)
                {
                    std::cout << "in ii 4 set is_process to 0: " << max_place + 1  << std::endl;
                }
            }
            else 
            {
                break;      // 已处理但未填充一次， 本次还是无法填充， 无法确认， 直接给了
            }
        }
    }
    std::vector<int> get_final_result() const 
    {
        return final_result;
    }

private:
    std::vector<int> final_result = {0,0,0,0, 0,0,0,0, 0,0,0,0};//每个位置对应的类别信息，综合两个模型结果生成， 0：未知 、1：R1 、2：R2 、3：fake 、4：空

};      // class super2_post
}       // namespace superstratum
}       // namespace Ten

#endif