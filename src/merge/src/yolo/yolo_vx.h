#ifndef __YOLO_VX_H_
#define __YOLO_VX_H_


#include "yolo_base.h"



namespace Ten
{
    namespace yolo
    {

        class yolo_vx : public yolo
        {
        public:
            /** 
                @brief 初始化函数
                @param model_path:模型路径 /xxx/xxx/best
                @param xpu: cpu or gpu
            */
            yolo_vx(const std::string model_path, const std::string xpu)
            {
                core_ = ov::Core();
                model_ = core_.read_model(model_path + ".xml", model_path + ".bin");
                if(xpu == "gpu") 
                {
                    core_.set_property("GPU", ov::cache_dir("./cache"));
                    compiled_model_ = core_.compile_model(model_, "GPU");
                }
                else compiled_model_ = core_.compile_model(model_, "CPU");
                input_shape_ = compiled_model_.input().get_shape();
                output_shape_ = compiled_model_.output().get_shape();
                infer_request_ = compiled_model_.create_infer_request();
            }

            /** 
                @brief 模型推理
                @param img: cv::Mat类型三通道图片
                @return std::vector<Detection>: 检测结果
            */
            virtual std::vector<Detection> worker(cv::Mat& img) override
            {
                if(flag_ == 0)
                {
                    std::cout << "😡 模型输出不匹配" << std::endl;
                    return std::vector<Detection>();
                }
                cv::Mat resized;
                cv::resize(img, resized, cv::Size(input_shape_[3],input_shape_[2]));
                //std::cout<< "input_shape_[2]h: "<<input_shape_[2] << " input_shape_[3]w: "<< input_shape_[3]<<std::endl;
                ov::Tensor input_tensor = preprocess(resized);
                infer_request_.set_input_tensor(input_tensor);
                // 同步推理：阻塞当前线程，直到推理完成
                //infer_request_.infer();
                infer_request_.start_async();
                infer_request_.wait();
                //std::cout<<"infer_request_.infer();"<<std::endl;
                auto output = infer_request_.get_output_tensor();
                auto detections = postprocess(output, img.cols, img.rows);
                return detections;
                
            }
        protected:

            virtual ov::Tensor preprocess(cv::Mat& image) override
            {
                cv::Mat imageRGB;
                cv::cvtColor(image, imageRGB, cv::COLOR_BGR2RGB);
                // 转换为浮点并归一化
                cv::Mat float_img;
                imageRGB.convertTo(float_img, CV_32F, 1.0/255.0);
                // 手动转置维度 (HWC -> CHW)
                std::vector<cv::Mat> channels(3);
                cv::split(float_img, channels);
                // 创建符合OpenVINO要求的形状 (1, 3, 640, 640)
                ov::Shape input_shape = {1, 3, input_shape_[2], input_shape_[3]};
                ov::Tensor input_tensor(ov::element::f32, input_shape);
                // 将数据复制到张量
                float* tensor_data = input_tensor.data<float>();
                for(int c = 0; c < 3; ++c) {
                    memcpy(tensor_data + c*input_shape_[2] * input_shape_[3],
                        channels[c].data,
                        input_shape_[2] * input_shape_[3] * sizeof(float));
                }
                return input_tensor;
            }
        
        // OpenVINO的核心引擎类，是所有操作的入口点，全局唯一即可
        ov::Core core_;
        // 针对指定硬件（CPU/GPU/NPU等）编译后的模型实例，包含硬件优化的执行图
        ov::CompiledModel compiled_model_;
        // 输入维度 0: 1（批次N）   输出维度 0: 1（批次N）
        // 输入维度 1: 3（通道C）   输出维度 1: 8（4坐标+4类别）
        // 输入维度 2: 640（高度H） 输出维度 2: 8400（检测框数量）
        // 输入维度 3: 640（宽度W）
        // 模型输入张量的形状，NCHW维度：0-批次1、1-通道3、2-高度640、3-宽度640（符合YOLO输入规范）
        ov::Shape input_shape_;
        // 模型输出张量的形状，NCHW维度：0-批次1、1-特征数8（4坐标+4类别）、2-检测框数量8400
        ov::Shape output_shape_;
        // 未编译的原生模型指针，存储模型的完整拓扑结构（层、张量、连接关系）、元信息等
        std::shared_ptr<ov::Model> model_;
        // 推理请求实例，由CompiledModel创建，负责**单次推理**的全流程：设置输入、执行推理、获取输出
        ov::InferRequest infer_request_;
        int flag_ = 1;
        };



    }
}



#endif

