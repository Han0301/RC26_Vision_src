#ifndef __APRILTAG_DETECTOR_AREA_H_
#define __APRILTAG_DETECTOR_AREA_H_
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

const int MAX_AREA = 100;  // 最大检测面积阈值

struct AprilTagResult
{
    int id;                  // 标签 ID
    cv::Point2f center;      // 标签中心点像素坐标
    std::array<cv::Point2f, 4> corners; // 标签4个角点像素坐标（顺时针顺序）
    double area;             // 标签四边形轮廓的像素面积
    int hamming;             // 解码汉明距离（0为无错码，数值越大错误越多）
    float decision_margin;   // 识别置信度边距（数值越高识别越可靠）
};

class AprilTagDetector_AREA
{
public:
    // 构造和析构
    explicit AprilTagDetector_AREA(int nthreads = 1);
    ~AprilTagDetector_AREA();

    // 禁止拷贝构造和赋值
    AprilTagDetector_AREA(const AprilTagDetector_AREA&) = delete;
    AprilTagDetector_AREA& operator=(const AprilTagDetector_AREA&) = delete;

    // 核心检测器
    void detR1(Ten::camera_virtual& camera);

    // 获取状态
    int get_is_r1() const {return is_r1;} 
private:
    std::string           state_;
    apriltag_family_t*    tf_      = nullptr;
    apriltag_detector_t*  td_      = nullptr;
    image_u8_t*           im_gray_ = nullptr;
    int is_r1 = -1;      // 初始状态
    std::vector<AprilTagResult> results_; // 检测结果

    bool detect(cv::Mat& image);
    bool set_is_distance();
    void drawResults(cv::Mat& image, const std::string& win_name = "AprilTag Detection");
};

// 构造实现
inline AprilTagDetector_AREA::AprilTagDetector_AREA(int nthreads)
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

    state_ = "AprilTagDetector_AREA ok";
}

// 析构实现
inline AprilTagDetector_AREA::~AprilTagDetector_AREA()
{
    if (im_gray_) { image_u8_destroy(im_gray_); }
    if (td_)      { apriltag_detector_destroy(td_); }
    if (tf_)      { tagStandard41h12_destroy(tf_); }
}

// 检测实现
inline bool AprilTagDetector_AREA::detect(cv::Mat& image)
{
    // 1. 状态与输入检查
    if (state_ != "AprilTagDetector_AREA ok") { return false; }

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

    // 6. 取结果
    results_.clear();
    int n = zarray_size(detections);
    for (int i = 0; i < n; ++i)
    {
        apriltag_detection_t* det = nullptr;
        zarray_get(detections, i, &det);
        if (!det) continue;

        AprilTagResult res;
        // 基础识别信息
        res.id = det->id;
        res.hamming = det->hamming;
        res.decision_margin = det->decision_margin;
        res.center = cv::Point2f(
            static_cast<float>(det->c[0]),
            static_cast<float>(det->c[1])
        );

        // 填充4个角点
        for (int j = 0; j < 4; ++j)
        {
            res.corners[j] = cv::Point2f(
                static_cast<float>(det->p[j][0]),
                static_cast<float>(det->p[j][1])
            );
        }

        // 鞋带公式计算四边形面积（像素单位）
        double cross_sum = 0.0;
        for (int j = 0; j < 4; ++j)
        {
            int next = (j + 1) % 4;
            cross_sum += det->p[j][0] * det->p[next][1]
                       - det->p[next][0] * det->p[j][1];
        }
        res.area = std::fabs(cross_sum) * 0.5;

        results_.push_back(std::move(res));
    }

    apriltag_detections_destroy(detections);
    state_ = "AprilTagDetector_AREA ok";

    return (n > 0);
}

inline bool AprilTagDetector_AREA::set_is_distance()
{
    if (results_.empty())
    {
        return false;
    }
    auto max_it = std::max_element(
        results_.begin(),
        results_.end(),
        [](const AprilTagResult& a, const AprilTagResult& b) {
            return a.area < b.area;
        });
    std::cout << "max_id: " << max_it->id << std::endl;
    std::cout << "max_area: " << max_it->area << std::endl;
    std::cout << "hamming: " << max_it->hamming << std::endl;
    std::cout << "decision_margin: " << max_it->decision_margin << std::endl;
    double max_area = max_it->area;
    return max_area > MAX_AREA; // 示例阈值，可根据需要调整
}

inline void AprilTagDetector_AREA::detR1(Ten::camera_virtual& camera)
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

        // 识别到apriltag
        if (is_apriltag)        
        {
            // 判断距离是否合适
            bool is_distance_ok = set_is_distance();
            if (is_distance_ok)
            {
                frame_ok += 1;
            }
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
        drawResults(frame);
        cv::waitKey(1); 
    }
}

inline void AprilTagDetector_AREA::drawResults(cv::Mat& image, const std::string& win_name)
{
    // 空图直接返回
    if (image.empty()) return;

    // 无检测结果也显示原图，保证窗口正常刷新
    if (results_.empty())
    {
        cv::imshow(win_name, image);
        return;
    }

    // 先找到面积最大的标签索引，用于高亮区分
    size_t max_idx = 0;
    double max_area = results_[0].area;
    for (size_t i = 1; i < results_.size(); ++i)
    {
        if (results_[i].area > max_area)
        {
            max_area = results_[i].area;
            max_idx = i;
        }
    }

    // 遍历绘制所有标签
    for (size_t i = 0; i < results_.size(); ++i)
    {
        const AprilTagResult& res = results_[i];
        bool is_max_area = (i == max_idx);

        // 样式：面积最大的用红色加粗，其余用绿色细线
        cv::Scalar color = is_max_area ? cv::Scalar(0, 0, 255) : cv::Scalar(0, 255, 0);
        int line_thick = is_max_area ? 2 : 1;

        // 1. 绘制标签四边形边框
        for (int j = 0; j < 4; ++j)
        {
            int next = (j + 1) % 4;
            cv::line(image, res.corners[j], res.corners[next], color, line_thick);
        }

        // 2. 绘制标签中心点
        cv::circle(image, res.center, 3, color, -1);

        // 3. 标注文字：ID + 像素面积
        std::string label = "ID:" + std::to_string(res.id) 
                          + " Area:" + std::to_string(static_cast<int>(res.area));
        cv::Point text_pos(res.center.x - 40, res.center.y - 10);
        cv::putText(image, label, text_pos, 
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }

    // 显示绘制后的图像
    cv::imshow(win_name, image);
}
}  // namespace Ten::apriltag_detect
#endif  // __APRILTAG_DETECTOR_H_