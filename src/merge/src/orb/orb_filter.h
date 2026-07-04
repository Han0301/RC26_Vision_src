#ifndef __ORB_FILTER_H_
#define __ORB_FILTER_H_

#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>  // 关键修复：添加Eigen-OpenCV互转头文件
#include <opencv2/calib3d/calib3d.hpp>
#include <iostream>
#include <iomanip>
#include <vector>
#include <limits>
#include "./../global/zbuffer.h"
#include "./../recognition/camera_calibration.h"


namespace Ten
{

    namespace ORB
    {

        // #define L_ 1.2f                 // 台阶长度
        // #define H_ 0.2f                 // 台阶高度
        // #define lx1_ 0.425f             // 台阶到方块的间距
        // #define ly1_ 0.425f             // 台阶到方块的间距
        // #define lh_ 0.35f               // 方块的长度
        // #define X_ 3.2f                 // 初始位置到梅花林1号位置边角的x轴距离
        // #define Y_ -1.2f                // 初始位置到梅花林1号位置边角的y轴距离
        // #define LIDAR_HEIGHT_ 0         // 雷达的高度 
        // #define box_half_length_ 0.175  // 方块长度的一半
        // #define step_half_length_ 0.6   // 台阶水平边长的一半
        // #define offset_x_ 0             // x方向上的偏移量
        // #define offset_y_ 0             // y方向上的偏移量
        // #define offset_z_ 0             // z方向上的偏移量


        // 长方体（正方体）定义：8个世界坐标角点 + OBB包围盒（用于遮挡检测）
        struct Cube {
            std::vector<Eigen::Vector3d> world_corners; // 8个角点（世界坐标系）
            Eigen::Vector3d center;                     // 中心（OBB用）
            Eigen::Vector3d axes[3];                    // OBB正交轴（世界坐标系）
            Eigen::Vector3d extents;                    // OBB半长度（世界坐标系）
            int valid[8] = {0};                         //0没有角点， 1有角点
        };

        // 射线定义（世界坐标系）
        struct Ray {
            Eigen::Vector3d origin;   // 起点（相机光心，世界坐标系）
            Eigen::Vector3d direction;// 方向（单位向量，世界坐标系）
        };

        struct orb_init_cube_points
        {
            std::vector<Cube> cube_list_;

            /**
             * @brief 初始化世界3d点
             */
            orb_init_cube_points()
            {
                cube_list_.resize(24);
                //台阶高度
                float hight_[12] {0.4, 0.2, 0.4, 0.2, 0.4, 0.6, 0.4, 0.6, 0.4, 0.2, 0.4, 0.2};

                for(size_t i = 0; i < 4; i++)
                {
                    for(size_t j = 0; j < 3; j++)
                    {
                        // 立方体1：中心(x,y,z)，尺寸(0.35,0.35,0.35)，无旋转
                        Cube box;
                        box.center = Eigen::Vector3d(X_ + i*L_ + lx1_ + box_half_length_ + offset_x_, Y_ - j*L_- ly1_ - box_half_length_ + offset_y_, hight_[i*3 + j] + box_half_length_ + offset_z_ - LIDAR_HEIGHT_);
                        box.axes[0] = Eigen::Vector3d(1,0,0);
                        box.axes[1] = Eigen::Vector3d(0,1,0);
                        box.axes[2] = Eigen::Vector3d(0,0,1);
                        box.extents = Eigen::Vector3d(box_half_length_, box_half_length_, box_half_length_);
                        box.world_corners = generateCubeCorners(box.center, Eigen::Vector3d(2*box_half_length_, 2*box_half_length_, 2*box_half_length_), Eigen::Matrix3d::Identity());
                        cube_list_[i*3 + j] = box;
                    }
                }

                for(size_t i = 0; i < 4; i++)
                {
                    for(size_t j = 0; j < 3; j++)
                    {
                        Cube step;
                        step.center = Eigen::Vector3d(X_ + i*L_ + step_half_length_  + offset_x_, Y_ - j*L_ - step_half_length_  + offset_y_, hight_[i*3 + j] / 2.0  + offset_z_ - LIDAR_HEIGHT_);
                        step.axes[0] = Eigen::Vector3d(1,0,0);
                        step.axes[1] = Eigen::Vector3d(0,1,0);
                        step.axes[2] = Eigen::Vector3d(0,0,1);
                        step.extents = Eigen::Vector3d(step_half_length_, step_half_length_, hight_[i*3 + j] / 2.0);
                        step.world_corners = generateCubeCorners(step.center, Eigen::Vector3d(2*step_half_length_, 2*step_half_length_, hight_[i*3 + j]), Eigen::Matrix3d::Identity());
                        cube_list_[12 + i*3 + j] = step;
                    }
                }
            }

            /**
             * @brief 生成长方体的8个世界坐标角点
             * @param center 长方体中心（世界坐标系）
             * @param size   长方体尺寸（x/y/z轴长度）
             * @param R      长方体姿态（旋转矩阵，世界坐标系）
             * @return 8个角点
             */
            std::vector<Eigen::Vector3d> generateCubeCorners(const Eigen::Vector3d& center, const Eigen::Vector3d& size,const Eigen::Matrix3d& R)
            {
                std::vector<Eigen::Vector3d> corners;
                Eigen::Vector3d half_size = size / 2.0;
                // 8个角点的局部偏移（以中心为原点）
                std::vector<Eigen::Vector3d> offsets = {
                    {-half_size.x(), -half_size.y(), -half_size.z()},
                    { half_size.x(), -half_size.y(), -half_size.z()},
                    { half_size.x(),  half_size.y(), -half_size.z()},
                    {-half_size.x(),  half_size.y(), -half_size.z()},
                    {-half_size.x(), -half_size.y(),  half_size.z()},
                    { half_size.x(), -half_size.y(),  half_size.z()},
                    { half_size.x(),  half_size.y(),  half_size.z()},
                    {-half_size.x(),  half_size.y(),  half_size.z()}
                };
                // 转换到世界坐标系
                for (const auto& off : offsets) {
                    corners.push_back(center + R * off);
                }
                return corners;
            }

        };

        class orb_filter
        {
        public:
            Ten_camerainfo camerainfo_;

            /**
             * @brief 核心接口：筛选所有立方体的有效角点，并设置valid数组
             * @return 筛选后的立方体列表
             */
            std::vector<Cube> filterValidCorners()
            {
                // 获取初始化的立方体列表
                std::vector<Cube> cube_list = orb_points_.cube_list_;
                int cube_count = cube_list.size();

                // 遍历每个立方体
                for (int cube_idx = 0; cube_idx < cube_count; ++cube_idx)
                {
                    Cube& curr_cube = cube_list[cube_idx];
                    // 遍历当前立方体的8个角点
                    for (int corner_idx = 0; corner_idx < 8; ++corner_idx)
                    {
                        const Eigen::Vector3d& world_corner = curr_cube.world_corners[corner_idx];
                        cv::Point2d pixel_corner;

                        // 条件1：投影到像素坐标系且在图像范围内
                        if (!projectWorldToPixel(world_corner, pixel_corner))
                        {
                            curr_cube.valid[corner_idx] = 0;
                            // std::cout<< "pixel_corner: " << pixel_corner << std::endl;
                            // std::cout<< "projectWorldToPixel: " << std::endl;
                            // std::cout<< "cube_idx: " << cube_idx << " corner_idx: " << corner_idx << std::endl;
                            continue;
                        }

                        // 条件2：无遮挡（自身+其他立方体）
                        if (isCornerOccluded(cube_list, cube_idx, corner_idx))
                        {
                            curr_cube.valid[corner_idx] = 0;
                            // std::cout<< "isCornerOccluded: " << std::endl;
                            // std::cout<< "cube_idx: " << cube_idx << " corner_idx: " << corner_idx << std::endl;
                            continue;
                        }

                        // 所有条件满足，标记为有效
                        curr_cube.valid[corner_idx] = 1;
                    }
                }

                // 更新内部的立方体列表
                orb_points_.cube_list_ = cube_list;
                return cube_list;
            }

            /**
             * @brief 由用户自己设置存在方块的数组
             * @param exist_boxes 输入 int， 12 数组， 1 表示存在， 0 表示不存在
             */
            void set_exist_boxes(int exist_boxes[12])
            {
                for(int i = 0; i < 12; i++)
                {
                    exist_boxes_[i] = exist_boxes[i];
                }
            }

            /**
             * @brief 获取初始化/筛选后的立方体列表
             * @return 立方体列表
             */
            std::vector<Cube> getCubeList() const
            {
                return orb_points_.cube_list_;
            }

        private:
            orb_init_cube_points orb_points_;
            int exist_boxes_[12] = {1,1,1,1,1,1,1,1,1,1,1,1};

             /**
             * @brief 世界点投影到像素坐标系（适配Ten_camerainfo）
             * @param world_pt   世界点
             * @param pixel_pt   输出像素点
             * @return 是否投影有效（在图像范围内）
             */
            bool projectWorldToPixel(const Eigen::Vector3d& world_pt, cv::Point2d& pixel_pt)
            {
                // 修复：参数非空/有效性校验
                cv::Mat K = camerainfo_.K();
                cv::Mat distCoeffs = camerainfo_.distCoeffs();
                cv::Mat rvec = camerainfo_.rvec();
                cv::Mat tvec = camerainfo_.tvec();

                // 校验矩阵非空且维度合法
                if (K.empty() || K.rows !=3 || K.cols !=3) {
                    std::cerr << "[Error] Invalid K matrix!" << std::endl;
                    return false;
                }
                if (rvec.empty() || rvec.rows !=3 || rvec.cols !=1) {
                    std::cerr << "[Error] Invalid rvec matrix!" << std::endl;
                    return false;
                }
                if (tvec.empty() || tvec.rows !=3 || tvec.cols !=1) {
                    std::cerr << "[Error] Invalid tvec matrix!" << std::endl;
                    return false;
                }

                std::vector<cv::Point3d> obj_pts;
                obj_pts.push_back(cv::Point3d(world_pt.x(), world_pt.y(), world_pt.z()));
                std::vector<cv::Point2d> img_pts;

                try {
                    // OpenCV投影：世界→相机→像素（考虑畸变）
                    // std::cout << "rvec: " << std::endl;
                    // std::cout << rvec << std::endl;
                    // std::cout << "tvec: " << std::endl;
                    // std::cout << tvec << std::endl;
                    // std::cout << "K: " << std::endl;
                    // std::cout << K << std::endl;
                    // std::cout << "distCoeffs: " << std::endl;
                    // std::cout << distCoeffs << std::endl;

                    cv::projectPoints(obj_pts, rvec, tvec, K, distCoeffs, img_pts);
                    pixel_pt = img_pts[0];
                    //std::cout << "pixel_pt: " << pixel_pt << std::endl;
                    // 判断是否在图像范围内（边界容错：>=0 且 < 宽/高）
                    int img_width = camerainfo_.W();
                    int img_height = camerainfo_.H();

                    // std::cout<< "img_width: " << img_width << std::endl;
                    // std::cout<< "img_height: " << img_height << std::endl;


                    return (pixel_pt.x >= 0 && pixel_pt.x < img_width &&
                            pixel_pt.y >= 0 && pixel_pt.y < img_height);
                } catch (const cv::Exception& e) {
                    std::cerr << "[Error] projectPoints failed: " << e.what() << std::endl;
                    return false;
                }
            }


            /**
             * @brief Slab算法：射线与OBB相交检测（世界坐标系）
             * @param ray 射线
             * @param cube 待检测的立方体
             * @param t_out 输出相交点的t参数（P = origin + t*direction）
             * @return 是否相交
             */
            bool rayIntersectsCubeOBB(const Ray& ray, const Cube& cube, double& t_out)
            {
                double t_min = 0.0;
                double t_max = std::numeric_limits<double>::infinity();
                Eigen::Vector3d origin_local = ray.origin - cube.center;

                for (int i = 0; i < 3; ++i)
                {
                    const Eigen::Vector3d& axis = cube.axes[i];
                    double ext = cube.extents[i];
                    double dir_dot = ray.direction.dot(axis);
                    double orig_dot = origin_local.dot(axis);

                    // 射线与当前轴平行
                    if (std::fabs(dir_dot) < 1e-8)
                    {
                        // 射线在OBB外，无相交
                        if (std::fabs(orig_dot) > ext) return false;
                        continue;
                    }

                    double t1 = (ext - orig_dot) / dir_dot;
                    double t2 = (-ext - orig_dot) / dir_dot;
                    if (t1 > t2) std::swap(t1, t2);

                    t_min = std::max(t_min, t1);
                    t_max = std::min(t_max, t2);
                    if (t_min > t_max) return false;
                }

                t_out = t_min;
                // 排除射线起点（相机光心）的误判
                return t_min > 1e-8;
            }

            /**
             * @brief 检测角点是否被遮挡（自身/其他方块）
             * @param cube_list  所有立方体列表
             * @param target_cube_idx 目标立方体索引
             * @param corner_idx      目标角点索引
             * @return 是否被遮挡
             */
            bool isCornerOccluded(const std::vector<Cube>& cube_list, int target_cube_idx, int corner_idx)
            {

                if(target_cube_idx < 12)
                {
                    if (exist_boxes_[target_cube_idx] != 1)
                    {
                        return true;
                    }
                }
                
                // 1. 获取目标角点和相机光心（世界坐标系）
                const Cube& target_cube = cube_list[target_cube_idx];
                const Eigen::Vector3d& corner = target_cube.world_corners[corner_idx];
                
                // 相机光心计算：外参R是世界→相机的旋转矩阵，T是平移向量
                // 相机光心的世界坐标 = -R^T * T
                Eigen::Matrix3d R_cam = camerainfo_.R();
                Eigen::Vector3d T_cam = camerainfo_.T();
                Eigen::Vector3d cam_center = -R_cam.transpose() * T_cam;
                // std::cout<< "cam_center: " << std::endl;
                // std::cout<< cam_center << std::endl;

                // 2. 生成从相机光心到角点的射线（单位化方向）
                Ray ray;
                ray.origin = cam_center;
                Eigen::Vector3d dir = corner - cam_center;
                if (dir.norm() < 1e-8) return true; // 角点与相机光心重合（异常）
                ray.direction = dir.normalized();
                double corner_dist = dir.norm(); // 角点到相机的距离

                // 3. 检测是否被其他立方体遮挡
                for (int i = 0; i < cube_list.size(); ++i)
                {
                    if (i == target_cube_idx) continue; // 跳过自身（先检测跨立方体遮挡）

                    if(i < 12)
                    {
                        if (exist_boxes_[i] != 1) continue; // 跳过空的方块
                    }

                    

                    double t_intersect;
                    if (rayIntersectsCubeOBB(ray, cube_list[i], t_intersect))
                    {
                        // 相交点距离 < 角点距离 → 被遮挡
                        if (t_intersect < corner_dist - 1e-8)
                        {
                            //std::cout<< "其他立方体遮挡: " << std::endl;
                            //std::cout<< "target_cube_idx: " << target_cube_idx << " corner_idx: " << corner_idx << std::endl;
                            return true;
                        }
                    }
                }

                // 4. 检测是否被自身遮挡（角点在相机视角的“背面”）
                // Eigen::Vector3d corner_rel = corner - target_cube.center;
                // Eigen::Vector3d view_dir = (cam_center - target_cube.center).normalized();
                // // 点积<0 → 角点在立方体朝向相机的反面，被自身遮挡
                // if (corner_rel.dot(view_dir) < -1e-8)
                // {
                //     std::cout<< "自身: " << std::endl;
                //     std::cout<< "target_cube_idx: " << target_cube_idx << " corner_idx: " << corner_idx << std::endl;
                //     return true;
                // }

                // 4. 修复后的自身遮挡检测逻辑（仅修改这部分）
                // 核心逻辑：射线从相机到角点，若在到达角点前先与自身立方体相交（非角点本身），则判定为自身遮挡
                double t_self;
                if (rayIntersectsCubeOBB(ray, target_cube, t_self))
                {
                    // 仅当相交点在相机和角点之间（且排除浮点误差），才判定为自身遮挡
                    if (t_self > 1e-8 && t_self < corner_dist - 1e-8)
                    {
                        //std::cout<< "自身: " << std::endl;
                        //std::cout<< "target_cube_idx: " << target_cube_idx << " corner_idx: " << corner_idx << std::endl;
                        return true;
                    }
                }


                return false;
            }
        };

    }

}


#endif
