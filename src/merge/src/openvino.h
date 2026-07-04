#ifndef __OPENVINO_H_
#define __OPENVINO_H_
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>  
#include <openvino/openvino.hpp>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>


namespace Ten
{
struct Detection {
    float cx_; 
    float cy_;
    float w_; 
    float h_;
    float conf_;
    int cls_id_;
};
class Ten_yolo
{
public:
    Ten_yolo& operator=(const Ten_yolo& yolo) = delete;
    Ten_yolo(const Ten_yolo& yolo) = delete;
    /** 
        @brief 初始化函数
        @param model_path:模型路径 /xxx/xxx/bin
        @param xpu: cpu or gpu
        @param way: 0 输出最优 1 输出每个类别最优
        @param conf_thres：框置信度
        @param cls_thres: 类别置信度
    */
    Ten_yolo(const std::string model_path, const std::string xpu = "cpu", size_t way = 0, float conf_thres = 0.75, float cls_thres = 0.75);
    /** 
        @brief 模型推理，支持yolov5/yolv11
        @param img: cv::Mat类型三通道图片
        @return std::vector<Detection>: 检测结果
    */
    std::vector<Detection> worker(cv::Mat& img);

    int get_out_shape_for_cls()
    {
        return output_shape_[1];
    }

    void init_map(std::vector<int>& map)
    {
        for(int i = 0; i < map.size(); i++)
        {
            map_.push_back(map[i]);
        }
    }

    ~Ten_yolo()
    {

    }
private:
    struct model
    {
        int item_;
        float confidence_;
    };


    ov::Tensor preprocess(cv::Mat& image);
    //排序
    static bool compareConfidence(const model& a, const model& b) {
        return a.confidence_ > b.confidence_;
    }

    std::vector<Detection> postprocess(ov::Tensor& output, int orig_w, int orig_h);

//openvino
ov::Core core_;
ov::CompiledModel compiled_model_;
// 输入维度 0: 1（批次N）   输出维度 0: 1（批次N）
// 输入维度 1: 3（通道C）   输出维度 1: 8（4坐标+4类别）
// 输入维度 2: 640（高度H） 输出维度 2: 8400（检测框数量）
// 输入维度 3: 640（宽度W）
ov::Shape input_shape_, output_shape_;
std::shared_ptr<ov::Model> model_; 
ov::InferRequest infer_request_;
//others
size_t way_;//筛选方式
float conf_thres_;//框置信度
float cls_thres_;//类别置信度
size_t classes_num_ = 0;
std::vector<int> map_;
};



class Ten_yolo_cls
{
public:
    /** 
        @brief 初始化函数
        @param model_path:模型路径 /xxx/xxx/bin
        @param map:映射关系
        @param xpu: cpu or gpu
        @param way: 0 输出最优 1 输出每个类别最优
        @param conf_thres：框置信度
        @param cls_thres: 类别置信度
    */
    Ten_yolo_cls(const std::string model_path, std::vector<int> map ,const std::string xpu = "cpu", size_t way = 0, float conf_thres = 0.75, float cls_thres = 0.75)
    :yolo_cls_(model_path, xpu, way, conf_thres, cls_thres)
    {
        if(map.size() == yolo_cls_.get_out_shape_for_cls())
        {
            yolo_cls_.init_map(map);
        }
    }

    /** 
        @brief 模型推理，支持yolov5/yolv11
        @param img: cv::Mat类型三通道图片
        @return std::vector<Detection>: 检测结果
    */
    std::vector<Detection> worker(cv::Mat& img)
    {
        return yolo_cls_.worker(img);
    }

    ~Ten_yolo_cls()
    {

    }

private:
Ten_yolo yolo_cls_;
};


class Ten_map
{
public:
    Ten_map()
    {

    }

int object_[13] = {0};
float object_confidence_[13] = {0};
int exist_boxes_[12] = {-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1,-1};  // 由用户自己设置存在方块的数组
int interested_boxes_[12] = {1,1,1,1,1,1,1,1,1,1,1,1};        // 由用户自己设置感兴趣的方块的数组
};



}









#endif


