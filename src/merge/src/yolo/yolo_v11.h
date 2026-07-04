#ifndef __YOLO_VE_H_
#define __YOLO_VE_H_

#include "yolo_vx.h"

namespace Ten
{

    namespace yolo
    {


        class yolo_v11 : public yolo_vx
        {
        public:
            /** 
                @brief 初始化函数
                @param model_path:模型路径 /xxx/xxx/best
                @param xpu: cpu or gpu
                @param conf_thres：框置信度
                @param iou: 交并比
            */
            yolo_v11(const std::string model_path, const std::string xpu, float cls_thres = 0.75, float iou = 0)
            :yolo_vx(model_path, xpu)
            ,cls_thres_(cls_thres)
            ,iou_(iou)
            {
                if(output_shape_.size() != 3 || output_shape_[2] != 8400)
                {
                    std::cout<<"output_shape_.size() != 3 || output_shape_[2] != 8400"<<std::endl;
                    flag_ = 0;
                }
            }


        protected:

            void virtual Get_result(ov::Tensor& output, int orig_w, int orig_h, std::vector<Detection>& detections)
            {
                std::vector<Detection> result;

                const float* data = output.data<const float>();
                auto shape = output.get_shape();
                std::cout<<"yolo11"<<std::endl;
                const int num_detections = shape[2];  // 8400
                const int features_per_box = shape[1];// 8
                const int num_classes = features_per_box - 4;// 根据实际情况调整

                // 特征索引定义（需根据模型文档确认）
                const int CX_IDX = 0;    // 中心x坐标通道
                const int CY_IDX = 1;    // 中心y坐标通道
                const int W_IDX  = 2;    // 宽度通道
                const int H_IDX  = 3;    // 高度通道
                for (int i = 0; i < num_detections; ++i) {
                    // 内存布局为 [1][5][8400]，按通道优先访问
                    float cx = data[CX_IDX * num_detections + i];
                    float cy = data[CY_IDX * num_detections + i];
                    float w  = data[W_IDX * num_detections + i];
                    float h  = data[H_IDX * num_detections + i];
                    //filter_.clear();
                    std::vector<model> filter_;
                    for(int j = 0; j < features_per_box - 4; j++)
                    {
                        model mod;
                        mod.item_ = j+1;
                        
                        mod.confidence_ =  data[(j+4)* num_detections + i];
                        filter_.push_back(mod);
                    }
                    std::sort(filter_.begin(), filter_.end(), compareConfidence);
                    if(filter_[0].confidence_ < cls_thres_) continue;
                    // 坐标反归一化（假设原始输入为640x640）
                    float cx_ = cx * (float)orig_w /input_shape_[3];
                    float cy_ = cy * (float)orig_h /input_shape_[2];
                    float w_ = w * (float)orig_w /input_shape_[3];
                    float h_ = h * (float)orig_h /input_shape_[2];
                    result.push_back({cx_, cy_, w_, h_, filter_[0].confidence_, filter_[0].item_, 0.0});
                }
                detections = nmsFilter(result, iou_);
            }

            virtual std::vector<Detection> postprocess(ov::Tensor& output, int orig_w, int orig_h) override
            {
                std::vector<Detection> detections;
                Get_result(output, orig_w, orig_h, detections);
                return detections;
            }
            
            float cls_thres_;
            float iou_;

        };



    }




}





#endif


