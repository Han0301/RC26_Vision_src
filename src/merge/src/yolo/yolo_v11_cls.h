#ifndef __YOLO_VE_CLS_H_
#define __YOLO_VE_CLS_H_

#include "yolo_v5.h"

namespace Ten
{

    namespace yolo
    {


        class yolo_v11_cls : public yolo_vx
        {
        public:
            /** 
                @brief 初始化函数
                @param model_path:模型路径 /xxx/xxx/best
                @param xpu: cpu or gpu
                @param map: 映射关系
                @param conf_thres：框置信度
                @param topk:前k个
            */
            yolo_v11_cls(const std::string model_path, const std::string xpu, std::vector<int> map, float conf_thres = 0.75, size_t topk = 1)
            :yolo_vx(model_path, xpu)
            ,map_(map)
            ,conf_thres_(conf_thres)
            ,topk_(topk)
            {
                if(output_shape_.size() != 2 || output_shape_[1] != map_.size() || topk > output_shape_[1])
                {
                    flag_ = 0;
                }
            }


        protected:

            virtual std::vector<Detection> postprocess(ov::Tensor& output, int orig_w, int orig_h) override
            {
                std::vector<Detection> detections;
                std::vector<Detection> result;
                const float* data = output.data<const float>();
                auto shape = output.get_shape();
                std::cout<<"yolo11_cls"<<std::endl;
                std::vector<model> filter_;
                for(int i = 0; i < output_shape_[1]; i++)
                {
                    model mod;
                    mod.item_ = i;
                    mod.confidence_ =  data[i];
                    //std::cout << "data[" << i << "]" << data[i] <<std::endl;
                    filter_.push_back(mod);                
                }
                std::sort(filter_.begin(), filter_.end(), compareConfidence);
   
                for(size_t i = 0 ; i < topk_; i++)
                {
                    detections.push_back({-1, -1, -1, -1, filter_[i].confidence_, map_[filter_[i].item_], -1});
                }
                return detections;
            }

            std::vector<int> map_;
            float conf_thres_;
            size_t topk_;
            
        };



    }




}





#endif


