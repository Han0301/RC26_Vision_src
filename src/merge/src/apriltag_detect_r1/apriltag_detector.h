#ifndef __APRILTAG_DETECTOR_H_
#define __APRILTAG_DETECTOR_H_
#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/common/image_u8.h>
#include <apriltag/common/zarray.h>
#include <vector>
#include <string>
#include "./../camera.h"
#include "./../usb_cam.h"
#include "./../usb_cam_multi.h"

namespace Ten::apriltag_detect
{ 
class AprilTagDetector
{
public:
    // 构造和析构
    explicit AprilTagDetector(int nthreads = 1);
    ~AprilTagDetector();

    // 禁止拷贝构造和赋值
    AprilTagDetector(const AprilTagDetector&) = delete;
    AprilTagDetector& operator=(const AprilTagDetector&) = delete;

    // 核心检测器
    void detR1(Ten::camera_virtual& camera);

    // 获取状态
    int get_is_r1() const {return is_r1;} 
private:
    std::string           state_;
    apriltag_family_t*    tf_      = nullptr;
    apriltag_detector_t*  td_      = nullptr;
    image_u8_t*           im_gray_ = nullptr;
    std::vector<int>      results_;
    int is_r1 = -1;      // 初始状态

    bool detect(cv::Mat& image);
};

// 构造实现
inline AprilTagDetector::AprilTagDetector(int nthreads)
    : state_("init")
{
    tf_ = tagStandard41h12_create();
    if (!tf_)
    {
        state_ = "init_fail: tagStandard41h12_create";
        return;
    }

    td_ = apriltag_detector_create();
    if (!td_)
    {
        tagStandard41h12_destroy(tf_);
        tf_ = nullptr;
        state_ = "init_fail: apriltag_detector_create";
        return;
    }

    apriltag_detector_add_family(td_, tf_);

    td_->nthreads          = nthreads;
    td_->quad_decimate     = 1.0f;
    td_->quad_sigma        = 0.8f;
    td_->refine_edges      = 1;
    td_->decode_sharpening = 0.5f;

    td_->qtp.min_cluster_pixels   = 5;
    td_->qtp.max_nmaxima          = 10;
    td_->qtp.critical_rad         = (float)(20.0 * M_PI / 180.0);
    td_->qtp.max_line_fit_mse     = 10.0f;
    td_->qtp.min_white_black_diff = 5;
    td_->qtp.deglitch             = 0;

    im_gray_ = nullptr;  // 首次 detect() 时惰性分配

    state_ = "AprilTagDetector ok";
}

// 析构实现
inline AprilTagDetector::~AprilTagDetector()
{
    if (im_gray_) { image_u8_destroy(im_gray_); }
    if (td_)      { apriltag_detector_destroy(td_); }
    if (tf_)      { tagStandard41h12_destroy(tf_); }
}

// 检测实现
inline bool AprilTagDetector::detect(cv::Mat& image)
{
    // 1. 状态与输入检查
    results_.clear();
    if (state_ != "AprilTagDetector ok") { return false; }

    if (image.empty())
    {
        state_ = "empty_frame";
        return false;
    }

    // 2. BGR → Gray
    cv::Mat gray;
    if (image.channels() == 3)
    {
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    }
    else
    {
        gray = image;
    }

    // 3. 自适应分辨率 (首次分配或分辨率变化)
    if (!im_gray_ || im_gray_->width != gray.cols || im_gray_->height != gray.rows)
    {
        if (im_gray_) { image_u8_destroy(im_gray_); }
        im_gray_ = image_u8_create(gray.cols, gray.rows);
        if (!im_gray_)
        {
            state_ = "gray_realloc_fail";
            return false;
        }
    }

    // 4. 拷贝灰度数据 (逐行, 适配 image_u8 stride 对齐)
    for (int r = 0; r < gray.rows; ++r)
    {
        std::memcpy(im_gray_->buf + r * im_gray_->stride, gray.ptr<uint8_t>(r), gray.cols);
    }

    // 5. 调用 apriltag 检测
    zarray_t* detections = apriltag_detector_detect(td_, im_gray_);
    if (!detections)
    {
        state_ = "detect_returned_null";
        return false;
    }

    // 6. 判断结果
    int n = zarray_size(detections);
    apriltag_detections_destroy(detections);
    state_ = "AprilTagDetector ok";

    return (n > 0);
}


inline void AprilTagDetector::detR1(Ten::camera_virtual& camera)
{
    int frame_count = 0;
    int frame_ok = 0;
    is_r1 = -1;
    int isnt_frame = 0;

    while (Ten::_TREADPOOL_FLAG_.read_flag())
    {
        cv::Mat frame = camera.camera_read();
        if (frame.empty())
        {
            isnt_frame += 1;
            std::cout << "frame.empty()" << std::endl;
            continue;
        }
        if (isnt_frame > 90) 
        {
            is_r1 = 0;
            break;
        }

        bool is_apriltag = detect(frame);

        if (is_apriltag)
        {
            frame_ok += 1;
        }
        frame_count += 1;

        // 置 is_r1
        if (frame_count > 10)
        {
            if (frame_ok > 3)
            {
                is_r1 = 1;
            }
            else
            {
                is_r1 = 0;
            }
            break;
        }
        std::cout << "frame_ok: " <<frame_ok << std::endl;
        cv::imshow("AprilTag Detection", frame);
        cv::waitKey(1);
    }
}
}  // namespace Ten::apriltag_detect
#endif  // __APRILTAG_DETECTOR_H_