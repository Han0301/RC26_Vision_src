#ifndef __OPENVINO_CPP_
#define __OPENVINO_CPP_
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <openvino/openvino.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include "openvino.h"

namespace Ten
{

// struct Detection {
//     float cx_; 
//     float cy_;
//     float w_; 
//     float h_;
//     float conf_;
//     int cls_id_;
// };

// class Ten_yolo
// {
// public:
//     Ten_yolo& operator=(const Ten_yolo& yolo) = delete;
//     Ten_yolo(const Ten_yolo& yolo) = delete;
//     /** 
//         @brief 初始化函数
//         @param model_path:模型路径 /xxx/xxx/bin
//         @param xpu: cpu or gpu
//         @param way: 0 输出最优 1 输出每个类别最优
//         @param conf_thres：框置信度
//         @param cls_thres: 类别置信度
//     */
//     Ten_yolo(const std::string model_path, const std::string xpu = "cpu", size_t way = 0, float conf_thres = 0.75, float cls_thres = 0.75)
//     :way_(way),
//     conf_thres_(conf_thres),
//     cls_thres_(cls_thres)
//     {
//         core_ = ov::Core();
//         model_ = core_.read_model(model_path + ".xml", model_path + ".bin");
//         if(xpu == "gpu") 
//         {
//             core_.set_property("GPU", ov::cache_dir("./cache"));
//             compiled_model_ = core_.compile_model(model_, "GPU");
//         }
//         else compiled_model_ = core_.compile_model(model_, "CPU");
//         input_shape_ = compiled_model_.input().get_shape();
//         output_shape_ = compiled_model_.output().get_shape();
//         infer_request_ = compiled_model_.create_infer_request();
//     }
//     /** 
//         @brief 模型推理，支持yolov5/yolv11
//         @param img: cv::Mat类型三通道图片
//         @return std::vector<Detection>: 检测结果
//     */
//     std::vector<Detection> worker(cv::Mat& img)
//     {
//         cv::Mat resized;
//         cv::resize(img, resized, cv::Size(input_shape_[3],input_shape_[2]));
//         //std::cout<< "input_shape_[2]h: "<<input_shape_[2] << " input_shape_[3]w: "<< input_shape_[3]<<std::endl;
//         ov::Tensor input_tensor = preprocess(resized);
//         infer_request_.set_input_tensor(input_tensor);
//         // 同步推理：阻塞当前线程，直到推理完成
//         //infer_request_.infer();
//         infer_request_.start_async();
//         infer_request_.wait();
//         //std::cout<<"infer_request_.infer();"<<std::endl;
//         auto output = infer_request_.get_output_tensor();
//         auto detections = postprocess(output, img.cols, img.rows);
//         return detections;
//     }

//     int get_out_shape_for_cls()
//     {
//         return output_shape_[1];
//     }

//     void init_map(std::vector<int>& map)
//     {
//         for(int i = 0; i < map.size(); i++)
//         {
//             map_.push_back(map[i]);
//         }
//     }

//     ~Ten_yolo()
//     {

//     }
// private:
//     struct model
//     {
//         int item_;
//         float confidence_;
//     };


//     ov::Tensor preprocess(cv::Mat& image) {
//         cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
//         // 转换为浮点并归一化
//         cv::Mat float_img;
//         image.convertTo(float_img, CV_32F, 1.0/255.0);
//         // 手动转置维度 (HWC -> CHW)
//         std::vector<cv::Mat> channels(3);
//         cv::split(float_img, channels);
//         // 创建符合OpenVINO要求的形状 (1, 3, 640, 640)
//         ov::Shape input_shape = {1, 3, input_shape_[2], input_shape_[3]};
//         ov::Tensor input_tensor(ov::element::f32, input_shape);
//         // 将数据复制到张量
//         float* tensor_data = input_tensor.data<float>();
//         for(int c = 0; c < 3; ++c) {
//             memcpy(tensor_data + c*input_shape_[2] * input_shape_[3],
//                 channels[c].data,
//                 input_shape_[2] * input_shape_[3] * sizeof(float));
//         }
//         return input_tensor;
//     }
//     //排序
//     static bool compareConfidence(const model& a, const model& b) {
//         return a.confidence_ > b.confidence_;
//     }
//     std::vector<Detection> postprocess(ov::Tensor& output, int orig_w, int orig_h)
//     {
//         std::vector<Detection> detections;
//         std::vector<Detection> result_;
//         //std::vector<model> filter_;
//         const float* data = output.data<const float>();
//         auto shape = output.get_shape();
//         // 打印形状信息
//         std::ostringstream oss;
//         oss << "[";
//         for (size_t i = 0; i < shape.size(); ++i) {
//             oss << shape[i] << (i != shape.size()-1 ? ", " : "]");
//         }
//         // 验证基础形状
//         // if (shape.size() != 3 || shape[0] != 1) {
//         //     ROS_ERROR("Unexpected output shape");
//         //     return detections;
//         // }
//         // 新输出结构解析
//         if(output_shape_[2] == 8400)
//         {
//             std::cout<<"yolo11"<<std::endl;
//             const int num_detections = shape[2];  // 8400
//             const int features_per_box = shape[1];// 8
//             const int num_classes = features_per_box - 4;// 根据实际情况调整
//             classes_num_ = num_classes;
//             // 特征索引定义（需根据模型文档确认）
//             const int CX_IDX = 0;    // 中心x坐标通道
//             const int CY_IDX = 1;    // 中心y坐标通道
//             const int W_IDX  = 2;    // 宽度通道
//             const int H_IDX  = 3;    // 高度通道
//             for (int i = 0; i < num_detections; ++i) {
//                 // 内存布局为 [1][5][8400]，按通道优先访问
//                 float cx = data[CX_IDX * num_detections + i];
//                 float cy = data[CY_IDX * num_detections + i];
//                 float w  = data[W_IDX * num_detections + i];
//                 float h  = data[H_IDX * num_detections + i];
//                 //filter_.clear();
//                 std::vector<model> filter_;
//                 for(int j = 0; j < features_per_box - 4; j++)
//                 {
//                     model mod;
//                     mod.item_ = j+1;
                    
//                     mod.confidence_ =  data[(j+4)* num_detections + i];
//                     filter_.push_back(mod);
//                 }
//                 std::sort(filter_.begin(), filter_.end(), &Ten_yolo::compareConfidence);
//                 if(filter_[0].confidence_ < cls_thres_) continue;
//                 // 坐标反归一化（假设原始输入为640x640）
//                 float cx_ = cx * (float)orig_w /input_shape_[3];
//                 float cy_ = cy * (float)orig_h /input_shape_[2];
//                 float w_ = w * (float)orig_w /input_shape_[3];
//                 float h_ = h * (float)orig_h /input_shape_[2];
//                 detections.push_back({cx_, cy_, w_, h_, filter_[0].confidence_, filter_[0].item_});
//             }
//         }
//         else if(output_shape_[1] == 25200)
//         {
//             std::cout<<"yolov5"<<std::endl;
//             const int num_detections = shape[1];  
//             const int features_per_box = shape[2];
//             const int num_classes = shape[2] - 5;
//             classes_num_ = num_classes;       
//             //std::cout<<"classes_num_"<<classes_num_<<std::endl;
//             // 特征索引定义（需根据模型文档确认）
//             const int CX_IDX = 0;    // 中心x坐标通道
//             const int CY_IDX = 1;    // 中心y坐标通道
//             const int W_IDX  = 2;    // 宽度通道
//             const int H_IDX  = 3;    // 高度通道
//             const int CONF_IDX = 4;  // 综合置信度通道
//             for (int i = 0; i < num_detections; ++i) {
//                 //内存布局为 [1][25200][8]，按通道优先访问
//                 float cx = data[i * features_per_box + CX_IDX];
//                 float cy = data[i * features_per_box + CY_IDX];
//                 float w  = data[i * features_per_box + W_IDX];
//                 float h  = data[i * features_per_box + H_IDX];
//                 float confidence = data[i * features_per_box + CONF_IDX];
//                 std::vector<model> filter_;
//                 //filter_.clear();
//                 for(int j = 0; j < num_classes; j++)
//                 {
//                     model mod;
//                     mod.item_ = j+1;
//                     mod.confidence_ =  data[i * features_per_box + 5 + j];
//                     filter_.push_back(mod);
//                 }
//                 //std::cout<<"filter_.size()"<<filter_.size()<<std::endl;
//                 std::sort(filter_.begin(), filter_.end(), &Ten_yolo::compareConfidence);
//                 if (confidence < conf_thres_) continue;
//                 if(filter_[0].confidence_ < cls_thres_) continue;
//                 // 坐标反归一化（假设原始输入为640x640）
//                 float cx_ = cx * (float)orig_w /input_shape_[3];
//                 float cy_ = cy * (float)orig_h /input_shape_[2];
//                 float w_ = w * (float)orig_w /input_shape_[3];
//                 float h_ = h * (float)orig_h /input_shape_[2];
//                 detections.push_back({cx_, cy_, w_, h_, confidence, filter_[0].item_});
//             }
//         }
//         else if(output_shape_.size() == 2)
//         {
//             std::cout<<"yolo11_cls"<<std::endl;
//             std::vector<model> filter_;
//             for(int i = 0; i < output_shape_[1]; i++)
//             {
//                 model mod;
//                 mod.item_ = i;
//                 mod.confidence_ =  data[i];
//                 //std::cout << "data[" << i << "]" << data[i] <<std::endl;
//                 filter_.push_back(mod);                
//             }
//             std::sort(filter_.begin(), filter_.end(), &Ten_yolo::compareConfidence);
//             if(filter_[0].confidence_ < cls_thres_)
//             {
//                 return detections;
//             }
//             if(map_.size() != 0)
//             {
//                 //std::cout<< "detections.push_back({-1, -1, -1, -1, filter_[0].confidence_, map_[filter_[0].item_]});" << std::endl;
//                 detections.push_back({-1, -1, -1, -1, filter_[0].confidence_, map_[filter_[0].item_]});
//             }
//             else
//             {
//                 detections.push_back({-1, -1, -1, -1, filter_[0].confidence_, filter_[0].item_});
//             }
            
//             return detections;
//         }
//         else
//         {
//             return detections;
//         }
        
//         if(way_ == 0)
//         {
//             if(detections.empty())
//             {
//                 return detections;
//             }
//             else
//             {
//                 auto best = *std::max_element(detections.begin(), detections.end(),
//                     [](const Detection& a, const Detection& b){
//                         return a.conf_ < b.conf_;
//                     });   
//                 result_.push_back(best);             
//             }
//             return result_;
//         }
//         else if(way_ == 1)
//         {
//             std::vector<std::vector<Detection>> detections_s;
//             detections_s.resize(classes_num_);
//             for(int i = 0; i < detections.size(); i++)
//             {
//                 Detection best = detections[i];
//                 detections_s[best.cls_id_-1].push_back(best);
//             }
//             for(int i = 0; i < detections_s.size(); i++)
//             {
//                 std::vector<Detection> _detection = detections_s[i];
//                 if(!_detection.empty())
//                 {
//                     auto best = *std::max_element(_detection.begin(), _detection.end(),[](const Detection& a, const Detection& b){return a.conf_ < b.conf_;});
//                     result_.push_back(best);
//                 }
//                 else
//                 {
//                     continue;
//                 } 
//             }
//             return result_;
//         }
//         else
//         {
//             return detections;
//         }
//     }

// //openvino
// ov::Core core_;
// ov::CompiledModel compiled_model_;
// // 输入维度 0: 1（批次N）   输出维度 0: 1（批次N）
// // 输入维度 1: 3（通道C）   输出维度 1: 8（4坐标+4类别）
// // 输入维度 2: 640（高度H） 输出维度 2: 8400（检测框数量）
// // 输入维度 3: 640（宽度W）
// ov::Shape input_shape_, output_shape_;
// std::shared_ptr<ov::Model> model_; 
// ov::InferRequest infer_request_;
// //others
// size_t way_;//筛选方式
// float conf_thres_;//框置信度
// float cls_thres_;//类别置信度
// size_t classes_num_ = 0;
// std::vector<int> map_;
// };



// class Ten_yolo_cls
// {
// public:
//     /** 
//         @brief 初始化函数
//         @param model_path:模型路径 /xxx/xxx/bin
//         @param map:映射关系
//         @param xpu: cpu or gpu
//         @param way: 0 输出最优 1 输出每个类别最优
//         @param conf_thres：框置信度
//         @param cls_thres: 类别置信度
//     */
//     Ten_yolo_cls(const std::string model_path, std::vector<int> map ,const std::string xpu = "cpu", size_t way = 0, float conf_thres = 0.75, float cls_thres = 0.75)
//     :yolo_cls_(model_path, xpu, way, conf_thres, cls_thres)
//     {
//         if(map.size() == yolo_cls_.get_out_shape_for_cls())
//         {
//             yolo_cls_.init_map(map);
//         }
//     }

//     /** 
//         @brief 模型推理，支持yolov5/yolv11
//         @param img: cv::Mat类型三通道图片
//         @return std::vector<Detection>: 检测结果
//     */
//     std::vector<Detection> worker(cv::Mat& img)
//     {
//         return yolo_cls_.worker(img);
//     }

//     ~Ten_yolo_cls()
//     {

//     }

// private:
// Ten_yolo yolo_cls_;
// };


// class Ten_map
// {
// public:
//     Ten_map()
//     {
//         object_.resize(12,0);
//     }

// std::vector<int> object_;
// float object_confidence_[12] = {0};
// };


    Ten_yolo::Ten_yolo(const std::string model_path, const std::string xpu, size_t way, float conf_thres, float cls_thres)
    :way_(way),
    conf_thres_(conf_thres),
    cls_thres_(cls_thres)
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



    std::vector<Detection> Ten_yolo::worker(cv::Mat& img)
    {
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

    ov::Tensor Ten_yolo::preprocess(cv::Mat& image) {
        cv::cvtColor(image, image, cv::COLOR_BGR2RGB);
        // 转换为浮点并归一化
        cv::Mat float_img;
        image.convertTo(float_img, CV_32F, 1.0/255.0);
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


    std::vector<Detection> Ten_yolo::postprocess(ov::Tensor& output, int orig_w, int orig_h)
    {
        std::vector<Detection> detections;
        std::vector<Detection> result_;
        //std::vector<model> filter_;
        const float* data = output.data<const float>();
        auto shape = output.get_shape();
        // 打印形状信息
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < shape.size(); ++i) {
            oss << shape[i] << (i != shape.size()-1 ? ", " : "]");
        }
        // 验证基础形状
        // if (shape.size() != 3 || shape[0] != 1) {
        //     ROS_ERROR("Unexpected output shape");
        //     return detections;
        // }
        // 新输出结构解析
        if(output_shape_.size() == 3)
        {
            if(output_shape_[2] == 8400)
            {
                std::cout<<"yolo11"<<std::endl;
                const int num_detections = shape[2];  // 8400
                const int features_per_box = shape[1];// 8
                const int num_classes = features_per_box - 4;// 根据实际情况调整
                classes_num_ = num_classes;
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
                    std::sort(filter_.begin(), filter_.end(), &Ten_yolo::compareConfidence);
                    if(filter_[0].confidence_ < cls_thres_) continue;
                    // 坐标反归一化（假设原始输入为640x640）
                    float cx_ = cx * (float)orig_w /input_shape_[3];
                    float cy_ = cy * (float)orig_h /input_shape_[2];
                    float w_ = w * (float)orig_w /input_shape_[3];
                    float h_ = h * (float)orig_h /input_shape_[2];
                    detections.push_back({cx_, cy_, w_, h_, filter_[0].confidence_, filter_[0].item_});
                }
            }
            else if(output_shape_[1] == 25200)
            {
                std::cout<<"yolov5"<<std::endl;
                const int num_detections = shape[1];  
                const int features_per_box = shape[2];
                const int num_classes = shape[2] - 5;
                classes_num_ = num_classes;       
                //std::cout<<"classes_num_"<<classes_num_<<std::endl;
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
                    std::sort(filter_.begin(), filter_.end(), &Ten_yolo::compareConfidence);
                    if (confidence < conf_thres_) continue;
                    if(filter_[0].confidence_ < cls_thres_) continue;
                    // 坐标反归一化（假设原始输入为640x640）
                    float cx_ = cx * (float)orig_w /input_shape_[3];
                    float cy_ = cy * (float)orig_h /input_shape_[2];
                    float w_ = w * (float)orig_w /input_shape_[3];
                    float h_ = h * (float)orig_h /input_shape_[2];
                    detections.push_back({cx_, cy_, w_, h_, confidence, filter_[0].item_});
                }
            }
        }    
        else if(output_shape_.size() == 2)
        {
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
            std::sort(filter_.begin(), filter_.end(), &Ten_yolo::compareConfidence);
            // if(filter_[0].confidence_ < cls_thres_)
            // {
            //     return detections;
            // }
            if(map_.size() != 0)
            {
                //std::cout<< "detections.push_back({-1, -1, -1, -1, filter_[0].confidence_, map_[filter_[0].item_]});" << std::endl;
                detections.push_back({-1, -1, -1, -1, filter_[0].confidence_, map_[filter_[0].item_]});
            }
            else
            {
                detections.push_back({-1, -1, -1, -1, filter_[0].confidence_, filter_[0].item_});
            }
            
            return detections;
        }
        else
        {
            return detections;
        }
        
        if(way_ == 0)
        {
            if(detections.empty())
            {
                return detections;
            }
            else
            {
                auto best = *std::max_element(detections.begin(), detections.end(),
                    [](const Detection& a, const Detection& b){
                        return a.conf_ < b.conf_;
                    });   
                result_.push_back(best);             
            }
            return result_;
        }
        else if(way_ == 1)
        {
            std::vector<std::vector<Detection>> detections_s;
            detections_s.resize(classes_num_);
            for(int i = 0; i < detections.size(); i++)
            {
                Detection best = detections[i];
                detections_s[best.cls_id_-1].push_back(best);
            }
            for(int i = 0; i < detections_s.size(); i++)
            {
                std::vector<Detection> _detection = detections_s[i];
                if(!_detection.empty())
                {
                    auto best = *std::max_element(_detection.begin(), _detection.end(),[](const Detection& a, const Detection& b){return a.conf_ < b.conf_;});
                    result_.push_back(best);
                }
                else
                {
                    continue;
                } 
            }
            return result_;
        }
        else
        {
            return detections;
        }
    }

}









#endif


