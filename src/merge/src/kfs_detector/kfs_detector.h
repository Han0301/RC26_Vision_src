#ifndef _KFS_DETECTOR_H_
#define _KFS_DETECTOR_H_
#include <opencv2/opencv.hpp>   
#include <vector>        
#include <unordered_map>   
#include <algorithm>           
#include <iostream>        
#include "../yolo/yolo_v5.h"
#include "./../parameter/parameter.h"

#define AREA_THRESHOLD 25000
namespace Ten
{
namespace r1_recognition
{
class kfs_detect
{
public:
    kfs_detect()
        :detector(Ten::_kfs_path_,"cpu",0.5,0.5,0)
    {}

    // // 输入vector表示路径位置，x,y,yaw表示当前位置信息, 输出调用的相机号, 左1， 右2, 返回0表示异常
    // int get_camera_id
    // (
    //     const std::vector<int>& planned_path,
    //     const double x,
    //     const double y,
    //     const double yaw
    // );

    // 设置目标位置的roi图像
    cv::Rect set_roi_detect(cv::Mat image);
    bool confirm_kfs(const cv::Rect& roi);
private:
    Ten::yolo::yolo_v5 detector;    // 检测器  
    
    std::vector<int> in_differ_li = {-1,-3,1,3};
    std::unordered_map<int,double> differ_to_target_yaw
    {
        {-1, 3.14159 / 2.0},
        { 3, 0},
        { 1,  -3.14159 / 2.0},
        {-3, -3.14159}
    };

    double normalize_angle(double angle)
    {
        const double PI = 3.1415926535;
        const double TWO_PI = 6.283185307;

        // 修复负数取模问题，强制归一化到 [-PI, PI]
        angle = fmod(angle + PI, TWO_PI);
        if (angle < 0)
            angle += TWO_PI;
        return angle - PI;
    }

};



}       // namespace r1_recognition


void kfs_detector_controller(int id);

}       // namespace Ten
#endif