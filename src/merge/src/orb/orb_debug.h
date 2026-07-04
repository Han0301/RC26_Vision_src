#ifndef __ORB_DEBUG_H_
#define __ORB_DEBUG_H_

#include "orb_filter.h"
#include <opencv2/imgproc/imgproc.hpp> // 绘图需要的头文件
#include <iostream>                    // 标准库输入输出

namespace Ten
{
    namespace ORB
    {
        struct orb_points
        {
            cv::Point2d point_ = cv::Point2d(-1,-1);
            cv::Point3d point_last_ = cv::Point3d(-1,-1,-1);
            int id_ = -1; //方块id
        };


        class orb_transform
        {
        public:
            // 清空保存的像素点（复用类实例时调用）
            void clearPixelPoints() 
            {
                pixel_points_.clear();
            }

            /**
             * @brief 投影有效角点到像素坐标系（核心接口，直接输入rvec/tvec）
             * @param cube_list    立方体列表（含valid标记的有效角点）
             * @param K            相机内参矩阵 (3x3)
             * @param dist_coeffs  相机畸变系数 (1x5/1x8，无畸变传空Mat)
             * @param rvec         相机外参旋转向量（OpenCV格式，3x1，世界→相机）
             * @param tvec         相机外参平移向量（OpenCV格式，3x1，世界→相机）
             * @param img_width    图像宽度（用于校验像素点范围，默认1920）
             * @param img_height   图像高度（用于校验像素点范围，默认1080）
             * @return 投影后的有效2D像素点列表（含方块ID）
             */
            std::vector<orb_points> projectValidCorners(
                const std::vector<Cube>& cube_list, 
                const cv::Mat K, 
                const cv::Mat distCoeffs,
                const cv::Mat rvec,          
                const cv::Mat tvec,          
                int img_width = 1920, 
                int img_height = 1080
            )
            {
                clearPixelPoints(); // 先清空历史数据

                // 参数合法性校验
                if (K.empty() || K.rows !=3 || K.cols !=3) {
                    std::cerr << "[Error] Invalid K matrix in projectValidCorners!" << std::endl;
                    return pixel_points_;
                }
                if (rvec.empty() || rvec.rows !=3 || rvec.cols !=1) {
                    std::cerr << "[Error] Invalid rvec in projectValidCorners!" << std::endl;
                    return pixel_points_;
                }
                if (tvec.empty() || tvec.rows !=3 || tvec.cols !=1) {
                    std::cerr << "[Error] Invalid tvec in projectValidCorners!" << std::endl;
                    return pixel_points_;
                }

                // 遍历所有立方体 / 2，投影有效角点
                for (size_t cube_idx = 0; cube_idx < cube_list.size() / 2; ++cube_idx)
                {
                    const Cube& cube = cube_list[cube_idx];
                    for (int corner_idx = 0; corner_idx < 8; ++corner_idx)
                    {
                        // 只处理valid=1的有效角点
                        if (cube.valid[corner_idx] != 1)
                        {
                            // std::cout<< "cube.valid[corner_idx] != 1: " << std::endl;
                            // std::cout<< "cube_idx: " << cube_idx << " corner_idx: " << corner_idx << std::endl;
                            continue;
                        }
                            
                        const Eigen::Vector3d& world_pt = cube.world_corners[corner_idx];
                        cv::Point2d pixel_pt_2d(-1,-1); // 临时存储Point2f类型的投影结果
                        // 世界点投影到像素坐标系（直接用传入的rvec/tvec）
                        if (projectWorldToPixel(world_pt, K, distCoeffs, rvec, tvec, pixel_pt_2d))
                        {
                            // // 校验像素点是否在图像范围内（防止绘图越界）
                            // if (pixel_pt_2f.x >= 0 && pixel_pt_2f.x < img_width &&
                            //     pixel_pt_2f.y >= 0 && pixel_pt_2f.y < img_height)
                            // {
                            // 构造orb_points对象，记录方块ID和像素点
                            // std::cout<< "projectWorldToPixel(world_pt, K, distCoeffs, rvec, tvec, pixel_pt_2f): " << std::endl;
                            // std::cout<< "cube_idx: " << cube_idx << " corner_idx: " << corner_idx << std::endl;
                            // std::cout<< "pixel_pt_2d: " << pixel_pt_2d << std::endl;
                            orb_points orb_pt;
                            orb_pt.id_ = static_cast<int>(cube_idx); // 方块ID = cube的索引
                            orb_pt.point_ = pixel_pt_2d; // 转换为Point2d
                            orb_pt.point_last_ = cv::Point3d(world_pt.x(), world_pt.y(), world_pt.z());
                            pixel_points_.push_back(orb_pt);
                            //}
                        }
                    }
                }

                return pixel_points_;
            }

            /**
             * @brief 调试函数：在图片上绘制有效像素点的空心圆（半径固定为2）
             * @param src_img     原始输入图片（CV_8UC3/RGB格式）
             * @param color       圆的颜色（默认红色 BGR格式）
             * @param thickness   线条宽度（默认1，空心需>=1）
             * @return 绘制后的图片
             */
            cv::Mat drawValidPoints(
                const cv::Mat& src_img, 
                const cv::Scalar& color = cv::Scalar(255, 0, 0), 
                int thickness = 1
            )
            {
                // 拷贝原图，避免修改输入
                cv::Mat dst_img = src_img.clone();

                // 空值保护：无有效点时直接返回原图（改用标准库输出）
                if (pixel_points_.empty())
                {
                    std::cerr << "[Warning] No valid pixel points to draw!" << std::endl;
                    return dst_img;
                }

                // 遍历所有有效点，绘制空心圆（半径固定为2，空心）
                const int radius = 3; // 要求的空心圆半径
                for (const orb_points& pt : pixel_points_)
                {
                    // 绘制空心圆：使用orb_points中的point_字段
                    cv::circle(dst_img, pt.point_, radius, color, thickness, cv::LINE_AA); // LINE_AA抗锯齿
                }

                return dst_img;
            }

            /**
             * @brief 获取保存的有效像素点列表（含方块ID）
             * @return 2D像素点列表（含ID）
             */
            std::vector<orb_points> getPixelPoints() const
            {
                return pixel_points_;
            }

        private:
            // 核心修改：从vector<cv::Point2f>改为vector<orb_points>
            std::vector<orb_points> pixel_points_; // 保存有效角点的2D像素坐标+所属方块ID

            /**
             * @brief 内部函数：单个世界点投影到像素坐标系
             * @param world_pt    世界坐标系3D点（Eigen格式）
             * @param K           相机内参（3x3）
             * @param dist_coeffs 畸变系数（1x5/1x8）
             * @param rvec        旋转向量（OpenCV格式，3x1）
             * @param tvec        平移向量（OpenCV格式，3x1）
             * @param pixel_pt    输出像素点（2D，Point2f）
             * @return 投影是否成功
             */
            bool projectWorldToPixel(
                const Eigen::Vector3d& world_pt, 
                const cv::Mat K, 
                const cv::Mat distCoeffs, 
                const cv::Mat rvec, 
                const cv::Mat tvec,
                cv::Point2d& pixel_pt
            )
            {
                try
                {
                    std::vector<cv::Point3d> obj_pts;
                    obj_pts.emplace_back(world_pt.x(), world_pt.y(), world_pt.z());
                    std::vector<cv::Point2d> img_pts;

                    //OpenCV原生投影函数（直接使用传入的rvec/tvec）
                    cv::projectPoints(obj_pts, rvec, tvec, K, distCoeffs, img_pts);
                    pixel_pt = img_pts[0]; // Point2d转Point2f（隐式转换，安全）
                    return true;
                }
                catch (const std::exception& e)
                {
                    // 改用标准库输出异常信息
                    std::cerr << "[Error] Project world point to pixel failed: " << e.what() << std::endl;
                    return false;
                }
            }
        };

        

    } 
}

#endif 
