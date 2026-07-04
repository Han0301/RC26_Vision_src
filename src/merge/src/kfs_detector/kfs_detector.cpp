#include <ros/ros.h>
#include "./kfs_detector.h"
#include "./../usb_cam_multi.h"
#include "./../threadpool.h"
#include "./../superstratum/controlR2.h"

namespace Ten
{

void kfs_detector_controller(int id)
{
    static Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
    std::vector<int> idxs = {Ten::_usb_device_num1_, Ten::_usb_device_num2_};
    Ten::Ten_usb_cam_multi& usbcam = Ten::Ten_usb_cam_multi::GetInstance(idxs,640,480,30);
    Ten::r1_recognition::kfs_detect kfs_detector;
    //是否发送
    int flag_is_run = 0;
    int flag_count_id = 0;
    size_t flag_frame = 0;

    //获取相机id
    //int id = 0;
    // while(Ten::_KFS_DECTECTOR_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag())
    // {
    //     nav_msgs::Odometry odo;
    //     if(Ten::_TF_GET_.get_latest(odo))
    //     {
    //         Ten::XYZRPY pose = Ten::Nav_Odometrytoxyzrpy(odo);
    //         Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose);
    //         Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
    //         id = kfs_detector.get_camera_id(Ten::_global_path_, result._xyz._x, result._xyz._y, result._rpy._yaw);
    //         break;
    //     }
    //     sll.sleep();
    // }
    //确定相机

    int idx = 0;
    std::cout << "id: " << id << std::endl;

    if(id == 1)
    {
        idx = Ten::_usb_device_num1_;
    }
    else if(id == 2)
    {
        idx = Ten::_usb_device_num2_;
    }
    else
    {
        flag_is_run = 1;
    }
    
    ros::Rate sl(30);
    while(Ten::_KFS_DECTECTOR_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag())
    {
        if(flag_is_run)
        {
            break;
        }

        cv::Mat img = usbcam.camera_read(idx);
        if(img.empty())
        {
            sl.sleep();
            continue;
        }

        flag_frame++;

        //检测
        cv::Rect roi = kfs_detector.set_roi_detect(img);
        bool is_kfs = kfs_detector.confirm_kfs(roi);
        if(!is_kfs)
        {
            if(flag_frame >= 20)
            {
                if(flag_count_id == 0)
                {
                    flag_count_id = 1;
                    usleep(600 * 1000);
                }
                else
                {
                    flag_is_run = 1;
                }
            }
            else
            {
                if(flag_frame >= 10)
                {
                    flag_is_run = 1;
                }   
            }
            
        }
        else
        {
            flag_count_id = 0;
            //flag_is_run = 0;
        }
        
        
        // if(flag_is_run)
        // {
        //     uint8_t arr[1] = {1};
        //     serial.serial_send(arr, 8, 1);
        // }
        // sl.sleep();
        // std::cout << "flag_is_run: " << flag_is_run << std::endl;
    }


    
    uint8_t arr[1] = {1};
    while(Ten::_KFS_DECTECTOR_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag())
    {
        std::cout << "serial.serial_send(arr, 8, length);" << std::endl;
        serial.serial_send(arr, 8, 1);
        usleep(100 * 1000);
    }

}



namespace r1_recognition
{

// 设置yolo 目标检测的矩形框
cv::Rect kfs_detect::set_roi_detect(cv::Mat image)
{
    // 1 调用worker函数
    std::vector<Ten::yolo::Detection> results = detector.worker(image);
    if(results.empty()) return cv::Rect();

    // 2 取最优结果
    std::sort(results.begin(), results.end(),
                [](const Ten::yolo::Detection &det1, const Ten::yolo::Detection &det2) -> bool
                {
                    double s1 = det1.w_ * det1.h_;
                    double s2 = det2.w_ * det2.h_;
                    return s1 > s2;
                });
    Ten::yolo::Detection best = results[0];       // 检测框中心点坐标和宽高

    // 3 计算框边界坐标
    float x1 = best.cx_ - best.w_ / 2;
    float x2 = best.cx_ + best.w_ / 2;
    float y1 = best.cy_ - best.h_ / 2;
    float y2 = best.cy_ + best.h_ / 2;
    return cv::Rect(cv::Point2i(cvRound(x1), cvRound(y1)),      // 四舍五入给结果
                    cv::Point2i(cvRound(x2), cvRound(y2)));
}

bool kfs_detect::confirm_kfs(const cv::Rect& roi)
{
    double roi_area = roi.area();
    std::cout << "roi_area: " << roi_area << std::endl;
    return roi_area > AREA_THRESHOLD;
}

// int kfs_detect::get_camera_id
// (
//     const std::vector<int>& planned_path,
//     const double x,
//     const double y,
//     const double yaw
// )
// {
//     int row = int((x - 3.2) / 1.2);
//     int col = -int((y + 1.2) / 1.2);
//     if ((x - 3.2) / 1.2 < 0)
//     {
//         row = -1;
//     }
//     if (row < -1 || row > 4 || col < 0 || col > 3)       // 解析的行列数错误
//     {
//         std::cout << "get_camera_id:  row < 0 || row > 4 || col < 0 || col > 3" << std::endl;
//         return 0;
//     }

//     // 确定当前位置在规划容器中
//     std::vector<int> path_begin = {-2,-1,0};
//     int now_place = row * 3 + col + 1;
//     auto is_in_path = std::find(planned_path.begin(), planned_path.end(), now_place);
//     auto is_in_path_begin = std::find(path_begin.begin(), path_begin.end(), now_place);

//     if (is_in_path != planned_path.end() || is_in_path_begin != path_begin.end())
//     {
//         int now_index = is_in_path - planned_path.begin();      // 当前位置所在规划路径容器中的索引
//         if (now_index == planned_path.size() - 1)       // 最后一个元素
//         {
//             std::cout << "get_camera_id: now_index == planned_path.size() - 1" << std::endl;
//             return 0;
//         }
//         else
//         {
//             int target_place;
//             if (is_in_path_begin != path_begin.end())       // 在前三个位置
//             {
//                 target_place = planned_path[0];
//             }
//             else
//             {
//                 target_place = planned_path[now_index + 1];
//             }
//             // 相机左右朝向
//             double yaw_left = normalize_angle(yaw + 3.14159 / 2.0);
//             double yaw_right = normalize_angle(yaw - 3.14159 / 2.0);

//             // 确定目标朝向
//             int differ = (target_place + 3) - (now_place + 3);
//             bool is_in_differ_li = (std::find(in_differ_li.begin(), in_differ_li.end(), differ) != in_differ_li.end());
//             if (is_in_differ_li)        // 当前位置和目标位置的差值符合台阶规律, 在{+3,+1,-1,-3}中
//             {
//                 double target_yaw = differ_to_target_yaw[differ];
//                 if (fabs(yaw_left - target_yaw) < fabs(yaw_right - target_yaw))
//                 {
//                     return 1;
//                 }
//                 else
//                 {
//                     return 2;
//                 }
//             }
//             else
//             {
//                 std::cout << "get_camera_id: (target_place - now_place) not in differ_li" << std::endl;
//             }
//         }
//     }
//     else        // 当前位置不在规划路径中
//     {
//         std::cout << "get_camera_id:  is_in_path == planned_path.end()" << std::endl;
//         return 0;
//     }
// }

}       


}

