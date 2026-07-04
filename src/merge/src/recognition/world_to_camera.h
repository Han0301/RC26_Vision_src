#ifndef __WORLD_TO_CAMERA_H_
#define __WORLD_TO_CAMERA_H_

#include "./../method_math.h"
#include "camera_calibration.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>

namespace Ten
{

    class Ten_worldtocamera
    {
    public:
        Ten_worldtocamera(const Ten_worldtocamera& wtc) = delete;
        Ten_worldtocamera& operator=(const Ten_worldtocamera& wtc) = delete;
        Ten_worldtocamera(){}
        ~Ten_worldtocamera(){}

        /**
         * @brief 设置当前世界坐标系到雷达坐标系的坐标变换
         * @param worldtocurrent: 当前世界坐标系到雷达坐标系的旋转平移
         */
        void set_worldtolidar(Ten::XYZRPY worldtocurrent)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            worldtocurrent_ = worldtocurrent;
        }

        /**
         * @brief 设置上一个世界坐标系到当前世界坐标系的坐标变换
         * @param world2toworld1: 上一个世界坐标系到当前世界坐标系的旋转平移
         */
        void set_world2toworld1(Ten::XYZRPY world2toworld1)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            world2toworld1_ = world2toworld1;
        }   

        /**
         * @brief 设置稳态误差
         * @param error: 稳态误差
         */
        void set_error(Ten::XYZRPY error)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            error_ = error;
        }

        /**
         * @brief 将世界点转换到相机坐标和像素坐标
         * @param world: 世界坐标点
         * @param camera: 相机坐标点
         * @param object_2d_points: 像素坐标点
         */
        void pcl_transform_world_to_camera(const pcl::PointCloud<pcl::PointXYZ>::Ptr& world, pcl::PointCloud<pcl::PointXYZ>::Ptr& camera, std::vector<cv::Point2f>& object_2d_points)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            //点云转换
            Ten::XYZRPY nworldtocurrent = worldtocurrent_ - error_;
            pcl::PointCloud<pcl::PointXYZ>::Ptr current(new pcl::PointCloud<pcl::PointXYZ>(world->size(), 1));
            Eigen::Matrix4d T_world_to_current = worldtocurrent(nworldtocurrent._xyz, nworldtocurrent._rpy);
            Eigen::Matrix4d T_world2_to_world = worldtocurrent(world2toworld1_._xyz, world2toworld1_._rpy);
            Eigen::Matrix4d T = T_world_to_current * T_world2_to_world;
            pcl::transformPointCloud(*world, *current, T);
            pcl::transformPointCloud(*current, *camera, camerainfo_.extrinsic());
            //相机转换
            std::vector<cv::Point3f> object_3d_points;
            for(size_t i = 0; i < camera->points.size(); i++)
            {
                cv::Point3f obj;
                obj.x = current->points[i].x;
                obj.y = current->points[i].y;
                obj.z = current->points[i].z;
                object_3d_points.push_back(obj);
                //std::cout<< "camera->points[i]" << camera->points[i] << std::endl;
            }
            // std::cout<< "camerainfo_.revc()" << camerainfo_.revc() << std::endl;
            // std::cout<< "camerainfo_.tevc()" << camerainfo_.tevc() << std::endl;
            // std::cout<< "camerainfo_.K()" << camerainfo_.K() << std::endl;
            // std::cout<< "camerainfo_.distCoeffs()" << camerainfo_.distCoeffs() << std::endl;

            //std::cout<< "object_3d_points[i]" << object_3d_points[0] << std::endl;
            cv::projectPoints(object_3d_points, camerainfo_.revc(), camerainfo_.tevc(), camerainfo_.K(), camerainfo_.distCoeffs(), object_2d_points);
            //std::cout<< "object_2d_points[i]" << object_2d_points[0] << std::endl;
            // for(size_t i = 0; i < object_2d_points.size(); i++)
            // {
            //     std::cout<< "object_2d_points[i]" << object_2d_points[i] << std::endl;
            // }
        }



    Ten::Ten_camerainfo camerainfo_;
    private:
    Ten::XYZRPY worldtocurrent_;
    Ten::XYZRPY world2toworld1_;
    Ten::XYZRPY error_;
    mutable std::mutex mtx_;
    };

    extern Ten::Ten_worldtocamera _CAMERA_TRANSFORMATION_;


}





#endif

