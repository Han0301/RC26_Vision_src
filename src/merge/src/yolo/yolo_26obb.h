#ifndef __YOLO_VTSOBB_H_
#define __YOLO_VTSOBB_H_

#include "yolo_vx.h"

namespace Ten
{

    namespace yolo
    {


        class yolo_26obb : public yolo_vx
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
            yolo_26obb(const std::string model_path, const std::string xpu, float conf_thres = 0.75, float iou = 0)
            :yolo_vx(model_path, xpu)
            ,conf_thres_(conf_thres)
            ,iou_(iou)
            {
                if(output_shape_.size() != 3 || output_shape_[1] != 300)
                {
                    flag_ = 0;
                }
                else
                {
                    std::cout<<"yolov26obb"<<std::endl;
                }
            }


        protected:

            virtual void Get_result(ov::Tensor& output, int orig_w, int orig_h, std::vector<Detection>& detections)
            {
                std::vector<Detection> result;
                const float* data = output.data<const float>();
                auto shape = output.get_shape();
                
                const int num_detections = shape[1];  
                const int features_per_box = shape[2];
                // const int num_classes = shape[2] - 5;
                // 特征索引定义（需根据模型文档确认）
                const int CX_IDX = 0;    // 中心x坐标通道
                const int CY_IDX = 1;    // 中心y坐标通道
                const int W_IDX  = 2;    // 宽度通道
                const int H_IDX  = 3;    // 高度通道
                const int CONF_IDX = 4;  // 综合置信度通道
                const int CLS_ID = 5; //类别id
                const int ANGLE = 6; //角度
                for (int i = 0; i < num_detections; ++i) {
                    //内存布局为 [1][300][7]，按通道优先访问
                    float cx = data[i * features_per_box + CX_IDX];
                    float cy = data[i * features_per_box + CY_IDX];
                    float w  = data[i * features_per_box + W_IDX];
                    float h  = data[i * features_per_box + H_IDX];
                    float confidence = data[i * features_per_box + CONF_IDX];
                    int idx = data[i * features_per_box + CLS_ID];
                    float angle = data[i * features_per_box + ANGLE];
                    //置信度过滤
                    if (confidence < conf_thres_) continue;

                    // 坐标反归一化（假设原始输入为640x640）
                    float cx_ = cx * (float)orig_w /input_shape_[3];
                    float cy_ = cy * (float)orig_h /input_shape_[2];
                    float w_ = w * (float)orig_w /input_shape_[3];
                    float h_ = h * (float)orig_h /input_shape_[2];
                    result.push_back({cx_, cy_, w_, h_, confidence, idx+1, angle});
                }

                detections = nmsFilterOBB(result, iou_);
            }

            virtual std::vector<Detection> postprocess(ov::Tensor& output, int orig_w, int orig_h) override
            {
                std::vector<Detection> detections;
                Get_result(output, orig_w, orig_h, detections);
                return detections;
            }

            float conf_thres_;
            float iou_;
        };



    }




}





#endif


