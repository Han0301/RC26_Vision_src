#ifndef __SET_RESULT_CPP_
#define __SET_RESULT_CPP_
#include "set_result.h"

namespace Ten::kfs_locator
{
void test()
{
    Ten::Ten_camera& _CAMERA_ = Ten::Ten_camera::GetInstance();
    _CAMERA_.reset_camera_depth(640, 480,30);
    rs2_intrinsics color_intr = _CAMERA_.get_color_intrinsics();    // 彩色内参 → 绘图用
    Ten::kfs_locator::Ten_set_result plane_fiter(color_intr);
    Ten::XYZRPY bias;

    while (ros::ok())
    {
        Ten::camera_frame frame = _CAMERA_.camera_read_depth();
        // 前处理
        bool is_pre_ok = plane_fiter.preprocess(frame);
        std::cout << "state: " <<  plane_fiter.get_state() << std::endl;
        if (is_pre_ok)
        {
            // 后处理
            bool is_post_ok = plane_fiter.postprocess();        // 置空输入点云，使用点云列表中的第一个点云
            std::cout << "state: " <<  plane_fiter.get_state() << std::endl;
            if (is_post_ok)
            {
                // 设置结果
                Ten::kfs_locator::result out = plane_fiter.set_result(bias);
                std::cout << "angle: " <<  ((out.bia_radian) * 180.0 / M_PI) << std::endl;
                std::cout << "res.x: " <<  out.x << std::endl;
                std::cout << "res.y: " <<  out.y << std::endl;
                std::cout << "res.z: " <<  out.z << std::endl;
            }
        }
    }
}
} // namespace Ten::kfs_locator
#endif