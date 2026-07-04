#ifndef __YOLO_HAN_H_
#define __YOLO_HAN_H_


#include "yolo_base.h"


namespace Ten
{

    namespace yolo
    {
        
        struct han
        {
            float invalid_ = 0;
            float valid_empty_ = 0;
            float valid_exist_ = 0;
        };


        class yolo_han
        {
        public:
            /** 
                @brief 初始化函数
                @param model_path:模型路径 /xxx/xxx/best
                @param xpu: cpu or gpu
            */
            yolo_han(const std::string model_path, const std::string xpu)
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
                if(input_shape_.size() != 5 || input_shape_[1] != 12)
                {
                    std::cout << "input_shape_[1]: " << input_shape_[1]<< std::endl;
                    flag_ = 0;
                }
                if(output_shape_.size() != 3 || output_shape_[1] != 12 || output_shape_[2] != 3 )
                {
                    flag_ = 0;
                }
            }

            /** 
                @brief 模型推理
                @param img: std::vector<cv::Mat>12章类型三通道图片
                @return std::vector<Detection>: 检测结果
            */
            virtual std::vector<han> worker(std::vector<cv::Mat>& img) 
            {
                if(flag_ == 0)
                {
                    std::cout << "😡 模型输出不匹配" << std::endl;
                    return std::vector<han>();
                }
                if(img.size() != 12)
                {
                    std::cout << "😡 输入图片不符合要求" << std::endl;
                    return std::vector<han>();
                }
                std::vector<cv::Mat> resizeds;
                for(size_t i = 0; i < img.size(); i++)
                {
                    cv::Mat resized;
                    cv::resize(img[i], resized, cv::Size(input_shape_[4],input_shape_[3]));
                    resizeds.push_back(resized);
                }
                
                //std::cout<< "input_shape_[2]h: "<<input_shape_[2] << " input_shape_[3]w: "<< input_shape_[3]<<std::endl;
                ov::Tensor input_tensor = preprocess(resizeds);
                infer_request_.set_input_tensor(input_tensor);
                // 同步推理：阻塞当前线程，直到推理完成
                //infer_request_.infer();
                infer_request_.start_async();
                infer_request_.wait();
                //std::cout<<"infer_request_.infer();"<<std::endl;
                auto output = infer_request_.get_output_tensor();
                auto detections = postprocess(output);
                return detections;
                
            }
        protected:


            /**
            * @brief 对RGB图像进行归一化（与PyTorch模型训练/推理逻辑完全一致）
            * @param src       输入图像（OpenCV默认BGR格式，CV_8UC3）
            * @param dst       输出归一化后的图像（CV_32FC3，维度顺序：RGB，已归一化）
            * @param mean      RGB三通道均值 [0.485, 0.456, 0.406]（与训练一致）
            * @param std       RGB三通道标准差 [0.229, 0.224, 0.225]（与训练一致）
            * @return bool     归一化是否成功
            */
            bool normalizeRGBImage(const cv::Mat& src, cv::Mat& dst, 
                                    const std::vector<float>& mean = {0.485f, 0.456f, 0.406f},
                                    const std::vector<float>& std = {0.229f, 0.224f, 0.225f}) {
                // ========== 1. 输入有效性检查 ==========
                if (src.empty()) {
                    std::cerr << "❌ 输入图像为空！" << std::endl;
                    return false;
                }
                if (src.channels() != 3) {
                    std::cerr << "❌ 输入图像必须是3通道（BGR），当前通道数：" << src.channels() << std::endl;
                    return false;
                }
                if (mean.size() != 3 || std.size() != 3) {
                    std::cerr << "❌ 均值/标准差必须为3个元素（RGB）！" << std::endl;
                    return false;
                }
            
                // ========== 2. BGR转RGB（OpenCV默认BGR，模型要求RGB） ==========
                cv::Mat rgb_img;
                cv::cvtColor(src, rgb_img, cv::COLOR_BGR2RGB);
            
                // ========== 3. 转换为float类型 + 归一化到0~1（除以255） ==========
                cv::Mat float_img;
                rgb_img.convertTo(float_img, CV_32F, 1.0 / 255.0);  // 等价于 PyTorch: ToTensor()
            
                // ========== 4. 拆分通道，逐通道减均值/除标准差 ==========
                std::vector<cv::Mat> rgb_channels(3);
                cv::split(float_img, rgb_channels);  // 拆分后：channels[0]=R, channels[1]=G, channels[2]=B
            
                // 逐通道执行：(x - mean_c) / std_c
                for (int c = 0; c < 3; ++c) {
                    rgb_channels[c] = (rgb_channels[c] - mean[c]) / std[c];
                }
            
                // ========== 5. 合并通道，输出最终归一化图像 ==========
                cv::merge(rgb_channels, dst);
            
                return true;
            }
            
            
            /**
             * @brief YOLOv11标准Softmax实现（多值版，网络原生层）
             * @param x 输入float向量，元素范围建议[-2,2]（YOLOv11网络输出）
             * @return std::vector<float> 输出概率分布，所有元素和为1，范围[0,1]
             * @note 完全对齐YOLOv11 Softmax层：数值稳定（减最大值）、float32单精度、高效推理
             */
            std::vector<float> yolov11_softmax(const std::vector<float>& x) {
                std::vector<float> result(x.size());
                if (x.empty()) return result;

                // 1. 数值稳定核心：找到输入的最大值（避免expf指数爆炸/下溢，YOLO全系列必做）
                float max_val = x[0];
                for (float val : x) {
                    if (val > max_val) max_val = val;
                }

                // 2. 计算指数化值（float32单精度expf，YOLOv11推理层标准）
                float sum_exp = 0.0f; // 指数化值的和，用于后续归一化
                for (int i = 0; i < x.size(); ++i) {
                    result[i] = expf(x[i] - max_val); // 减最大值保证数值稳定
                    sum_exp += result[i];
                }

                // 3. 归一化：所有值除以总和，得到和为1的概率分布（Softmax核心）
                for (int i = 0; i < x.size(); ++i) {
                    result[i] /= sum_exp;
                }

                return result;
            }



            virtual ov::Tensor preprocess(std::vector<cv::Mat>& image) 
            {
                std::vector<cv::Mat> changes;
                // std::vector<cv::Mat> changes1;
                // std::vector<cv::Mat> changes2;
                // std::vector<cv::Mat> changes3;
                for(size_t i = 0; i < image.size(); i++)
                {
                    //cv::Mat imageRGB;;
                    //cv::cvtColor(image[i], imageRGB, cv::COLOR_BGR2RGB);
                    // 转换为浮点并归一化
                    cv::Mat float_img;
                    //imageRGB.convertTo(float_img, CV_32F, 1.0/255.0);
                    normalizeRGBImage(image[i],float_img);
                    std::vector<cv::Mat> channels(3);
                    cv::split(float_img, channels);
                    changes.push_back(channels[0]);
                    changes.push_back(channels[1]);
                    changes.push_back(channels[2]);
                    // changes1.push_back(channels[0]);
                    // changes2.push_back(channels[1]);
                    // changes3.push_back(channels[2]);
                }
                
                //std::cout << changes[0] << std::endl;
                //std::cout << changes[1] << std::endl;

                // for(size_t i = 0; i < changes1.size(); i++)
                // {
                //     changes.push_back(changes1[i]);
                // }
                // for(size_t i = 0; i < changes2.size(); i++)
                // {
                //     changes.push_back(changes2[i]);
                // }
                // for(size_t i = 0; i < changes3.size(); i++)
                // {
                //     changes.push_back(changes3[i]);
                // }


                ov::Shape input_shape = {1, input_shape_[1], 3, input_shape_[3], input_shape_[4]};
                ov::Tensor input_tensor(ov::element::f32, input_shape);
                // 将数据复制到张量
                float* tensor_data = input_tensor.data<float>();
                for(int c = 0; c < input_shape_[1]*3; ++c) {
                    memcpy(tensor_data + c*input_shape_[3] * input_shape_[4],
                        changes[c].data,
                        input_shape_[3] * input_shape_[4] * sizeof(float));
                }
                return input_tensor;


            }

            virtual std::vector<han> postprocess(ov::Tensor& output) 
            {
                std::vector<han> detections;
                const float* data = output.data<const float>();
                auto shape = output.get_shape();
                std::cout<<"yolo_han"<<std::endl;
                const int num_image = output_shape_[1];
                const int cls = output_shape_[2];

                std::vector<std::vector<float>> channels;
                channels.resize(12);
                for (int i = 0; i < num_image; ++i) {
                    for(int j = 0; j < cls; j++)
                    {
                        channels[i].push_back(data[i * cls + j]);
                    }
                }

                for (int i = 0; i < num_image; ++i) {
                    channels[i] = yolov11_softmax(channels[i]);
                }
                


                for (int i = 0; i < num_image; ++i) {

                    han h;
                    h.invalid_ = channels[i][0];
                    h.valid_empty_ = channels[i][1];
                    h.valid_exist_  = channels[i][2];
                    detections.push_back(h);
                }
                return detections;

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

