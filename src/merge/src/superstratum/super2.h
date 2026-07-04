#ifndef __SUPPER2_H_
#define __SUPPER2_H_

#include"super.h"
#include"./../yolo/yolo_han2.h"

namespace Ten
{
    namespace superstratum
    {
        // 用于存储多次在不同位置的 set_img 的记录
        struct super_init{

            std::vector<std::vector<box>> time_box_lists_;
            std::vector<std::vector<float>> time_ps_w_;                 // 有效点的权重， 由该位置 point_size / 该行最大像素值 计算得到

            std::vector<float> set_ps_w(
                std::vector<box>& box_lists
            )
            {
                std::vector<float>ps_w = {0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f,0.0f};
                if (box_lists.size() != 12)
                {
                    std::cout << "[Error]from func set_ps_w: box_lists.size() != 12, make sure box_lists is set" << std::endl;
                    return;
                }
                // 找到每行的最大值
                std::vector<int>max_point_size = {0,0,0,0};
                for(int i = 0;i < 4; i++)
                {
                    for (int j = 0;j < 3;j++)
                    {
                        if (box_lists[i * 3 + j].point_size > max_point_size[i])
                        {
                            max_point_size[i] = box_lists[i * 3 + j].point_size;
                        }
                    }
                }
                // 填充 point_size_weight 字段
                for(int i = 0;i < 4; i++)
                {
                    for (int j = 0;j < 3;j++)
                    {
                        ps_w[i * 3 + 1] = box_lists[i * 3 + j].point_size / max_point_size[i];
                    }
                }
                return ps_w;
            }

        };


        class supper2
        {
        public:
            supper2()
            {
                //设置稳态误差
                Ten::XYZRPY xyzrpy_error;
                xyzrpy_error._xyz._x = 0;
                xyzrpy_error._xyz._y = 0;
                xyzrpy_error._xyz._z = 0;
                xyzrpy_error._rpy._roll = 0;
                xyzrpy_error._rpy._pitch = 0;
                xyzrpy_error._rpy._yaw = 0;
                //coordinate_transformation_.set_stead_state_error(xyzrpy_error);
                _CAMERA_TRANSFORMATION_.set_error(xyzrpy_error);
                lidar_to_camera_transform_matrix_ << 
                -0.0293067,  -0.999359,  -0.0205564,  0.0519757,  
                0.0195515,  0.0199882,  -0.999609,  0.47424,  
                0.999379,  -0.0296971,  0.0189532,  0.336381,  
                0.0         ,  0.0        ,  0.0        ,  1.0; 
                _CAMERA_TRANSFORMATION_.camerainfo_.set_Extrinsic_Matrix(lidar_to_camera_transform_matrix_);
               
            }

            /**
             * @brief 输入图像和r,t信息， 设置12个roi图像, 填充当前的box_lists_, 并保存到 time_box_lists_, 用于输入的 筛选
             * @param image 输入的全局图像
             * @param rvec  旋转矩阵
             * @param tvec  平移矩阵
             * @param exist_boxes int12数组, 表示存在的列表
            */
            void set_img(cv::Mat image, 
                        cv::Mat rvec, 
                        cv::Mat tvec, 
                        int *exist_boxes = nullptr)
            {
                //世界点和box_list的类对象，对里面数据进行处理
                Ten::init_3d_box world_point;
                camera_transformation_.camerainfo_.set_RT(rvec, tvec);
                camera_transformation_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
                world_point.pcl_to_C();
                if (exist_boxes == nullptr) 
                {
                    int exist_boxes[12] = {1,1,1,1,1,1,1,1,1,1,1,1};
                }
                zbuffer_.set_exist_boxes(exist_boxes);
                zbuffer_.set_box_lists_(image, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);

                super_init_.time_ps_w_.push_back(super_init_.set_ps_w(world_point.box_lists_));
                super_init_.time_box_lists_.push_back(world_point.box_lists_);
            }


            /**
             * @brief 输入多组box_lists， 写入一组当前最优(结合ps_w筛选)的图像，用于输入的 预处理
             * @param time_box_lists 存储多次在不同位置的 set_img 的记录
             * @param time_ps_w 存储多次在不同位置的 ps_w 记录
             * @param roi_images 待写入的当前最优的一组roi图像， 内部会初始化
             * @param min_ps_w 能容忍的最小point_size_weight 权重大小， 默认为0.1
            */
            void process_img(
                std::vector<std::vector<box>>& time_box_lists,
                std::vector<std::vector<float>>& time_ps_w,
                std::vector<cv::Mat>& roi_images,
                float min_ps_w = 0.1f
            )
            {
                roi_images.clear();
                roi_images.assign(12, cv::Mat::zeros(64, 64, CV_8UC1));

                std::vector<int> is_filled(12, 0);     // 表示是否填充图像列表的标志位
                // 表示各位置最大ps_w
                std::vector<float> max_ps_w(12, min_ps_w);     
                
                if (time_ps_w.size() != time_box_lists.size()) {
                    std::cout << "[Error] time_ps_w.size() != time_box_lists.size()" << std::endl;
                    return;
                }

                for (int time = 0; time < time_box_lists.size(); time++)
                {
                    std::vector<box>& box_lists = time_box_lists[time];
                    if (box_lists.size() != 12 || time_ps_w[time].size() != 12)
                    {
                        std::cout << "[Error]from func supper2::process_img: box_lists.size() != 12 || time_ps_w[time].size() != 12" << std::endl;
                        return;
                    }
                    for (int place = 0; place < 12; place++)
                    {
                        if (time_ps_w[time][place] > 0.1f && time_ps_w[time][place] >= max_ps_w[place])
                        {
                            roi_images[place] = box_lists[place].roi_image.clone();
                            is_filled[place] = 1;
                            max_ps_w[place] = time_ps_w[time][place];
                        }
                    }
                }
                bool full_filled = true;
                std::vector<int> isnt_filled_idx;
                for (int i = 0;i < is_filled.size(); i++)
                {
                    if (!is_filled[i])
                    {
                        full_filled = false;
                        isnt_filled_idx.push_back(i + 1);
                    }
                }
                if (!full_filled)
                {
                    std::cout << "[Error]from func supper2::process_img: isn't full_filled" << std::endl;
                    std::cout << "isnt_filled: ";
                    for (int i = 0; i < isnt_filled_idx.size();i++)
                    {
                        std::cout << isnt_filled_idx[i] << " ";
                    }
                    std::cout << std::endl;
                }else{std::cout << "[Info] all 12 roi images filled successfully!" << std::endl;}
            }

        private:
            Eigen::Matrix4d lidar_to_camera_transform_matrix_ = Eigen::Matrix4d::Identity(); //雷达到相机
            Ten::Ten_worldtocamera camera_transformation_; //坐标点转换器，用于将世界坐标系下的点变换到当前坐标系，以及像素坐标系
            Ten::Ten_occlusion_handing zbuffer_; //zb处理器，用于处理遮挡关系
            Ten::superstratum::super_init super_init_;
        };

    }


}


#endif
