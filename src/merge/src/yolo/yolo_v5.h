#ifndef __YOLO_VF_H_
#define __YOLO_VF_H_

#include "yolo_vx.h"

namespace Ten
{

    namespace yolo
    {


        class yolo_v5 : public yolo_vx
        {
        public:
            /** 
                @brief 初始化函数
                @param model_path:模型路径 /xxx/xxx/best
                @param xpu: cpu or gpu
                @param conf_thres：框置信度
                @param cls_thres: 类别置信度
                @param iou: 交并比
            */
            yolo_v5(const std::string model_path, const std::string xpu, float conf_thres = 0.75, float cls_thres = 0.75, float iou = 0)
            :yolo_vx(model_path, xpu)
            ,conf_thres_(conf_thres)
            ,cls_thres_(cls_thres)
            ,iou_(iou)
            {
                if(output_shape_.size() != 3 || output_shape_[1] != 25200)
                {
                    flag_ = 0;
                }
            }


        protected:

            virtual void Get_result(ov::Tensor& output, int orig_w, int orig_h, std::vector<Detection>& detections)
            {
                std::vector<Detection> result;
                const float* data = output.data<const float>();
                auto shape = output.get_shape();
                std::cout<<"yolov5"<<std::endl;
                const int num_detections = shape[1];  
                const int features_per_box = shape[2];
                const int num_classes = shape[2] - 5;
                // 特征索引定义（需根据模型文档确认）
                const int CX_IDX = 0;    // 中心x坐标通道
                const int CY_IDX = 1;    // 中心y坐标通道
                const int W_IDX  = 2;    // 宽度通道
                const int H_IDX  = 3;    // 高度通道
                const int CONF_IDX = 4;  // 综合置信度通道
                for (int i = 0; i < num_detections; ++i) {
                    //内存布局为 [1][25200][8]，按通道优先访问
                    float cx = data[i * features_per_box + CX_IDX];
                    float cy = data[i * features_per_box + CY_IDX];
                    float w  = data[i * features_per_box + W_IDX];
                    float h  = data[i * features_per_box + H_IDX];
                    float confidence = data[i * features_per_box + CONF_IDX];
                    std::vector<model> filter_;
                    //filter_.clear();
                    for(int j = 0; j < num_classes; j++)
                    {
                        model mod;
                        mod.item_ = j+1;
                        mod.confidence_ =  data[i * features_per_box + 5 + j];
                        filter_.push_back(mod);
                    }
                    //std::cout<<"filter_.size()"<<filter_.size()<<std::endl;
                    std::sort(filter_.begin(), filter_.end(), compareConfidence);
                    if (confidence < conf_thres_) continue;
                    if(filter_[0].confidence_ < cls_thres_) continue;
                    // 坐标反归一化（假设原始输入为640x640）
                    float cx_ = cx * (float)orig_w /input_shape_[3];
                    float cy_ = cy * (float)orig_h /input_shape_[2];
                    float w_ = w * (float)orig_w /input_shape_[3];
                    float h_ = h * (float)orig_h /input_shape_[2];
                    result.push_back({cx_, cy_, w_, h_, confidence, filter_[0].item_});
                }

                detections = nmsFilter(result, iou_);
            }

            virtual std::vector<Detection> postprocess(ov::Tensor& output, int orig_w, int orig_h) override
            {
                std::vector<Detection> detections;
                Get_result(output, orig_w, orig_h, detections);
                return detections;
            }

            float conf_thres_;
            float cls_thres_;
            float iou_;
        };



    }




}





#endif


