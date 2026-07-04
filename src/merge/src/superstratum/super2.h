#ifndef __SUPPER2_H_
#define __SUPPER2_H_

#include"super.h"
#include"./../yolo/yolo_han2.h"
#include <iomanip>      // 打印限制浮点数

// #define _roi12_path_ "/home/h/下载/yolo11s_roi12_atten_17/yolo11s_roi12_atten_17"
// #define _juanzhou_path_ "/home/h/下载/卷轴分类1_仿真+现实_32类/best"
// #define _box_num_ 8

namespace Ten
{
namespace superstratum
{
    // 用于存储多次在不同位置的 set_img 的记录
    struct super_init{
        
        std::vector<std::vector<box>> time_box_lists_;
        std::vector<std::vector<int>> time_ps_;                 // 有效点的权重， 由该位置 point_size / 该行最大像素值 计算得到
        std::string model_path = _roi12_path_;
        std::string device = "cpu";
        std::vector<int> mapping_ = {1, 10, 11, 12, 13, 14, 15, 16, 17,18, 19, 2 ,20 ,21 ,22 ,23 ,24, 25 ,26 ,27 ,28 ,29, 3 ,30, 31,32, 4 ,5 ,6, 7, 8 ,9}; //yolocls模型类别映射关系
        // 用到的模型
        Ten::yolo::yolo_han2 yolo_han2_;        //yoloroi12处理器，用于筛空
        Ten::yolo::yolo_v11_cls yolo_detector_; //yolo处理器，用于判断方块类别
        
        super_init() : yolo_han2_(model_path,device)
                        ,yolo_detector_(_juanzhou_path_, "cpu", mapping_)        // 用【初始化列表】调用 yolo_han2_ 的构造函数
        {
            
        }
        std::vector<int> set_ps_(
            std::vector<box>& box_lists
        )
        {
            std::vector<int>ps_(12,0.00f);
            if (box_lists.size() != 12)
            {
                std::cout << "[Error]from func set_ps_: box_lists.size() != 12, make sure box_lists is set" << std::endl;
                return ps_;
            }

            // 填充 point_size_ 字段
            for(int i = 0;i < 4; i++)
            {
                for (int j = 0;j < 3;j++)
                {
                    ps_[i * 3 + j] = box_lists[i * 3 + j].point_size;
                }
            }
            return ps_;
        }
    };

    class supper2
    {
    public:
        // ------------------------------------------------------------------------------------------- 初始化误差和变换
        // 构造函数， is_debug 是否启用debug, true则不初始化相机
        supper2(bool is_debug = false)
        :camera_(is_debug ? nullptr : &Ten::Ten_camera::GetInstance()),
        log_(&Ten::Ten_logger::GetInstance(std::string(ROOT_DIR) + std::string("log")))
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

        // ------------------------------------------------------------------------------------------- 填充 batch_images 的方式
        // 1 填充 batch_images, 直接从相机和雷达的数据中设置
        void set_batch_images()
        {
            //获取图片
            Ten::ORB::orb_exhaust_element oee;
            camera_->camera_read().copyTo(oee.image_);
            //获得雷达定位坐标
            nav_msgs::Odometry odo = Ten::_TF_GET_.read_data();
            Ten::XYZRPY lidar_of_world1 = Ten::Nav_Odometrytoxyzrpy(odo);
            if(std::isnan(lidar_of_world1._rpy._yaw)) 
            {
                return;
            }
            _CAMERA_TRANSFORMATION_.set_worldtolidar(lidar_of_world1);
            Eigen::Matrix4d world_to_camera = _CAMERA_TRANSFORMATION_.getXYZRPY_matrix();
            transverter_.set_Extrinsic_Matrix(world_to_camera);

            transverter_.rvec().copyTo(oee.rvec_);
            transverter_.tvec().copyTo(oee.tvec_);
            batch_images.push_back(oee);
        }

        // 2 填充 batch_images， 从datasets中设置
        void set_debug_batch_images(
            const std::vector<Ten::ORB::debug_orb_exhaust_element>& batch_images_labels
        )
        {
            batch_images.clear();
            for (int j = 0; j < batch_images_labels.size(); j++)
            {
                batch_images.push_back(batch_images_labels[j].oee);
            }
        }

        // ------------------------------------------------------------------------------------------- 模型调用 和 后处理
        /**
         * @brief 设置place 和 per_loss, sure_loss
         * @param count_1_limit 1类数量约束上限
         * @param min_ps_w 能容忍的最小point_size_weight 权重大小， 默认为0.2
         * @param min_sure_loss 能容忍的最小整体损失平均， 低于这个阈值， 不会retry， 直接使用第一次的结果
         * @param is_print 是否打印调试的标志位
        */
        void set_roi12_place(
            const int count_1_limit = _box_num_,
            const float min_sure_loss = 0.08,
            const bool is_print = false
        )
        {
            std::vector<int> place_1(12,-1);
            std::vector<int> place_2(12,-1);
            place.assign(12,-1);
            std::vector<Ten::yolo::han2> det_1;
            std::vector<Ten::yolo::han2> det_2;
            std::vector<float>per_loss_1;
            std::vector<float>per_loss_2;
            per_loss.assign(12,-1.0f);

            // 表示整体的强行取到8个1的整体损失平均
            float sure_loss_1;
            float sure_loss_2;

            // 设置图片
            set_img(batch_images);
            set_roi_images(roi_images);

            // 第一次 进行筛空
            det_1 = super_init_.yolo_han2_.worker(roi_images);
            set_place(det_1,count_1_limit,place_1,sure_loss_1);

            // retry
            if (sure_loss_1 > min_sure_loss)
            {
                for(int i = 0; i < 12; i++) {
                    retry_exists[i] = place_1[i];
                }

                // 设置图片
                set_img(batch_images,retry_exists);
                set_roi_images(roi_images);

                // 进行推理处理
                det_2 = super_init_.yolo_han2_.worker(roi_images);
                set_place(det_2,count_1_limit,place_2,sure_loss_2);

                // 写入结果 place，per_loss, sure_loss
                place = place_2;
                set_per_loss(place,det_2,per_loss);
                sure_loss = sure_loss_2;
            }
            else    // 没有第二次尝试， 直接使用第一次的结果 写入结果 place，per_loss, sure_loss
            {
                place = place_1;
                set_per_loss(place,det_1,per_loss);
                sure_loss = sure_loss_1;
            }

            // 打印和调试部分
            if (is_print)
            {
                print_roi12_place(place_1,place_2,det_1,det_2,sure_loss_1,sure_loss_2,place,per_loss,sure_loss);
            }
        }

        /**
         * @brief 由分类模型来设置类别， 请确保 roi_images 已被填充
        */
        void set_cls()
        {
            classifier_.assign(12, 0);
            confidence_.assign(12, 0.0f);

            for(size_t i = 0; i < roi_images.size(); i++)
            {
                //推理
                std::vector<Ten::yolo::Detection> result = super_init_.yolo_detector_.worker(roi_images[i]);

                confidence_[i] = result[0].conf_;

                // 类别映射
                if (result[0].cls_id_ == 1)
                {
                    classifier_[i] = 1;
                }
                else if (result[0].cls_id_ >= 2 && result[0].cls_id_ <= 16)
                {
                    classifier_[i] = 2;
                }
                else if (result[0].cls_id_ >= 17 && result[0].cls_id_ <= 31)
                {
                    classifier_[i] = 3;
                }
                else if (result[0].cls_id_ == 32)
                {
                    classifier_[i] = 4;
                }
                else
                {
                    std::cout << "[error] set_cls: result[0].cls_id_ isn't in [1,32]" << std::endl;
                }
            }   
        }

        // -------------------------------------------------------------------------------------------get 模块 和 print 调试打印
        // 返回整体损失
        const float get_sure_loss()
        {
            return sure_loss;
        }

        // 返回每个位置损失
        const std::vector<float> get_per_loss()
        {
            return per_loss;
        }

        // 返回每个位置是否有方块
        const std::vector<int> get_place()
        {
            return place;
        }

        // 返回每个位置的类别
        const std::vector<int> get_classifier_()
        {
            return classifier_;
        }

        // 返回每个位置类别识别的置信度
        const std::vector<float> get_confidence_()
        {
            return confidence_;
        }

        // 返回roi_images最优roi12图像
        const std::vector<cv::Mat> get_roi_images()
        {
            return roi_images;
        }

        // 返回存储的历史 time_ps_
        const std::vector<std::vector<int>> get_time_ps_()
        {
            return super_init_.time_ps_;
        }

        /**
         * @brief 日志保存
        */
        void save_roi_log()
        {
            std::vector<Ten::box> box_lists_save;
            box_lists_save.resize(12);
            for(int i = 0; i < 12; i++)
            {
                box_lists_save[i].idx = i + 1;
                box_lists_save[i].roi_image = cv::Mat::zeros(160, 160, CV_8UC3);
            }

            for(int i = 0; i < 12; i++)
            {
            box_lists_save[i].cls = classifier_[i];
            box_lists_save[i].confidence = confidence_[i];
            roi_images[i].copyTo(box_lists_save[i].roi_image);
            }
            log_->record_image(box_lists_save);
        }

        // 调试打印， 打印 两个模型的结果 和 后处理之后的最终结果
        void print_post_cls(const std::vector<int>& final_result = {})
        {
            if (!final_result.empty())
            {
                for (int i =0;i < 12;i++)
                {
                    if (i + 1 < 10)
                    {
                        std::cout << std::fixed << std::setprecision(3) << "pl: " << i + 1 << ",  place: " << place[i] << ", per_loss: " << per_loss[i] << ", cls: " << classifier_[i] << ", conf: " << confidence_[i] << ", final_result: " << final_result[i]<< std::endl;
                    }
                    else
                    {
                        std::cout << std::fixed << std::setprecision(3) << "pl: " << i + 1 << ", place: " << place[i] << ", per_loss: " << per_loss[i] << ", cls: " << classifier_[i] << ", conf: " << confidence_[i] << ", final_result: " << final_result[i]<< std::endl;
                    }
                }
            }
        }
    // ------------------------------------------------------------------------------------------- 私有变量 和 子功能
    private:
        Ten::Ten_worldtocamera camera_transformation_; //坐标点转换器，用于将世界坐标系下的点变换到当前坐标系，以及像素坐标系
        Ten::Ten_occlusion_handing zbuffer_; //zb处理器，用于处理遮挡关系
        Ten::Ten_camera* camera_; //相机实例的引用
        Ten::Ten_logger* log_; //日志实例的引用
        Ten::Ten_camerainfo transverter_; //转换器，用于将4*4的变化矩阵转为rvec,tvec
        super_init super_init_;         // 模型路径， 输入等参数加载器
        Ten::init_3d_box world_point;

        std::vector<Ten::ORB::orb_exhaust_element> batch_images; // 原生图像数据， r,t
        std::vector<cv::Mat> roi_images;                         // 给模型输入的最优图像
        Eigen::Matrix4d lidar_to_camera_transform_matrix_ = Eigen::Matrix4d::Identity(); //雷达到相机
        int default_boxes[12] = {1,1,1,1,1,1,1,1,1,1,1,1};      // 默认值
        int retry_exists[12];                                   // retry 重新填充的值

        // 推理生成的数据
        std::vector<int> classifier_ = {0,0,0,0, 0,0,0,0, 0,0,0,0}; //每个位置对应的类别信息，由模型直接生成  0：未知 、1：R1 、2：R2 、3：fake 、4：空
        std::vector<float> confidence_ = {0,0,0,0, 0,0,0,0, 0,0,0,0}; //每个位置对应的置信度
        std::vector<int> place = {-1,-1,-1,-1, -1,-1,-1,-1, -1,-1,-1,-1}; //每个位置是否有方块
        std::vector<float> per_loss = {-1.0f,-1.0f,-1.0f, -1.0f,-1.0f,-1.0f, -1.0f,-1.0f,-1.0f, -1.0f,-1.0f,-1.0f}; //每个位置是否有方块的损失
        float sure_loss;

        // ------------------------------------------------------------------------------------------- 设置 图像 的功能部分
        /**
         * @brief 输入图像和r,t信息， 设置12个roi图像, 填充当前的box_lists_, 并保存到 time_box_lists_, 用于输入的 筛选
         * @param batch_images 一定批次的全局图像
         * @param exist_boxes int12数组, 表示存在的列表
        */
        void set_img(
            const std::vector<Ten::ORB::orb_exhaust_element>& batch_images,
            int *exist_boxes = nullptr)
        {
            for(size_t j = 0; j < batch_images.size(); j++)
            {
                world_point = Ten::init_3d_box();
                //世界点和box_list的类对象，对里面数据进行处理
                camera_transformation_.camerainfo_.set_RT(batch_images[j].rvec_, batch_images[j].tvec_);
                camera_transformation_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
                world_point.pcl_to_C();

                if (j == 0)
                {
                    super_init_.time_box_lists_.clear();
                    super_init_.time_ps_.clear();
                }

                if (exist_boxes == nullptr) 
                {
                    exist_boxes = default_boxes;
                    zbuffer_.set_exist_boxes(exist_boxes);
                }
                else
                {
                    zbuffer_.set_exist_boxes(exist_boxes);
                }
                zbuffer_.set_box_lists_(batch_images[j].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
                
                super_init_.time_ps_.push_back(super_init_.set_ps_(world_point.box_lists_));
                super_init_.time_box_lists_.push_back(world_point.box_lists_);
            }
        }

        /**
         * @brief 由 time_box_lists_和 time_ps_ 写入 roi_images， 仅在 set_roi12_place 中调用
         * @param roi_images 要写入的最优图像列表
        */
        void set_roi_images(
            std::vector<cv::Mat>& roi_images
        )
        {
            roi_images.assign(12, cv::Mat::zeros(64, 64, CV_8UC3));

            std::vector<int> is_filled(12, 0);     // 表示是否填充图像列表的标志位
            // 表示各位置最大ps_
            std::vector<int> max_ps_(12, 0);     
            
            if (super_init_.time_ps_.size() != super_init_.time_box_lists_.size()) {
                std::cout << "[Error] time_ps_.size() != time_box_lists.size()" << std::endl;
                return;
            }

            for (int time = 0; time < super_init_.time_box_lists_.size(); time++)
            {
                std::vector<box>& box_lists = super_init_.time_box_lists_[time];
                if (box_lists.size() != 12 || super_init_.time_ps_[time].size() != 12)
                {
                    std::cout << "[Error]from func supper2::process_img: box_lists.size() != 12 || time_ps_[time].size() != 12" << std::endl;
                    return;
                }
                for (int pl = 0; pl < 12; pl++)
                {
                    if (super_init_.time_ps_[time][pl] >= max_ps_[pl])
                    {
                        roi_images[pl] = box_lists[pl].roi_image.clone();
                        is_filled[pl] = 1;
                        max_ps_[pl] = super_init_.time_ps_[time][pl];
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
                std::cout << "[Error]from func supper2::set_roi_images: isn't full_filled" << std::endl;
                std::cout << "isnt_filled: ";
                for (int i = 0; i < isnt_filled_idx.size();i++)
                {
                    std::cout << isnt_filled_idx[i] << " ";
                }
                std::cout << std::endl;
            }
        }

        // ------------------------------------------------------------------------------------------- set_roi12_place 的 后处理 和 调试打印
        /**
         * @brief 根据检测结果设置 place， 仅在 set_roi12_place 中调用
         * @param dets 输入检测结果
         * @param count_1_limit 1类数量约束
         * @param place 写入 place
         * @param sure_loss 写入 本次配准的整体损失
        */
        void set_place(
            const std::vector<Ten::yolo::han2>& dets,
            const int& count_1_limit,
            std::vector<int>& place,
            float& sure_loss
        )
        {
            sure_loss = 0.0f;
            int sure_1 = 0;
            std::vector<float> per_los(12,0.0f);    // 衡量每个位置的可信程度， loss在【0，1】，越小越好
            place.assign(12,-1);                     // 基本可信的列表

            for (int pl = 0; pl < dets.size(); pl++)
            {
                // 填充 loss 列表
                if (dets[pl].valid_exist_ > 0.5)
                {
                    per_los[pl] = 2 * (1 - dets[pl].valid_exist_);
                }
                else
                {
                    per_los[pl] = 2 * dets[pl].valid_exist_;
                }
                // 填充 sure 列表
                if (per_los[pl] < 0.4)
                {
                    place[pl] = (dets[pl].valid_exist_ > 0.5)? 1 : -1;
                    if (dets[pl].valid_exist_ > 0.5)
                    {
                        sure_loss += per_los[pl];
                        sure_1 += 1;
                    }
                }
            }
            while (Ten::_TREADPOOL_FLAG_.read_flag())
            {
                if (sure_1 >= count_1_limit)
                {
                    break;
                }
                // 找到当前的最大1类阈值
                int max_place = -1;
                float max_1_conf = 0.0f;
                for (int pl = 0; pl < dets.size(); pl++)
                {
                    if (place[pl] == 1) continue;
                    if (dets[pl].valid_exist_ > max_1_conf)
                    {
                        max_1_conf = dets[pl].valid_exist_;
                        max_place = pl;
                    }
                }
                // 把目前认为最有可能为1 类的填充进sure
                if (max_place != -1)
                {
                    place[max_place] = 1;
                    sure_1 += 1;
                    sure_loss += per_los[max_place];
                } 
            }

            // 赋0类
            for (int pl = 0; pl < place.size(); pl++)
            {
                if (place[pl] != 1)
                {
                    place[pl] = 0;
                }
            }
            sure_loss = sure_loss / count_1_limit;
        }

        /**
         * @brief 设置最终输出的per_loss， 仅在 set_roi12_place 调用
         * @param place 输入最终的place
         * @param det   检测置信度
         * @param per_loss 写入的每个位置损失
        */
        void set_per_loss(
            const std::vector<int>& place,
            const std::vector<Ten::yolo::han2>& det,
            std::vector<float>& per_loss
        )
        {
            per_loss.assign(12,-1.0f);

            if (place.size() != 12 || det.size() != 12)
            {
                std::cout << "[error]: set_per_loss:  place.size() != 12 || det.size() != 12" << std::endl;
                return;
            }
            else
            {
                for (int i = 0; i < place.size(); i++)
                {
                    if (place[i] == 1) 
                    {
                        per_loss[i] = 1.0f - det[i].valid_exist_;
                    }
                    else
                    {
                        per_loss[i] = det[i].valid_exist_;
                    }
                }
            }
        }

        /**
         * @brief 调试打印 set_roi12_place 中的相关中间变量， roi12模型的处理过程， 仅在 set_roi12_place 通过标志位 调用
        */
        void print_roi12_place(
            const std::vector<int>& place_1,
            const std::vector<int>& place_2,
            const std::vector<Ten::yolo::han2>& det_1,
            const std::vector<Ten::yolo::han2>& det_2,
            const float& sure_loss_1,
            const float& sure_loss_2,
            const std::vector<int>& place,
            const std::vector<float>& per_loss,
            const float& sure_loss
        )
        {
            std::cout << "-----------first--------------" << std::endl;
            // 各个位置置信度
            for(auto e : det_1)
            {
                std::cout << "exist: " << e.valid_exist_ << ", empty: " << e.valid_empty_ << std::endl;
            } 
            // 各个位置后处理后给的标签
            std::cout << "place_1: ";
            for(auto& e : place_1)
            {
                std::cout << e << " ";
            }
            std::cout << std::endl;
            std::cout << "sure_loss: " << sure_loss_1 << std::endl;  

            if (sure_loss_1 > 0.08)
            {
                std::cout << "-----------retry--------------" << std::endl;
                // 各个位置置信度
                for(auto e : det_2)
                {
                    std::cout << "exist: " << e.valid_exist_ << ", empty: " << e.valid_empty_ << std::endl;
                } 
                // 各个位置后处理后给的标签
                std::cout << "place_2: ";
                for(auto& e : place_2)
                {
                    std::cout << e << " ";
                }
                std::cout << std::endl;
                std::cout << "retry: sure_loss: " << sure_loss_2 << std::endl;  
            }
            // 最终的结果
            std::cout << "-----------result--------------" << std::endl;
            std::cout << "place: ";
            for(auto& e : place)
            {
                std::cout << std::fixed << std::setprecision(3) << e << " ";
            }
            std::cout << std::endl;
            std::cout << "per_loss: ";
            for(auto& e : per_loss)
            {
                std::cout << e << " ";
            }
            std::cout << std::endl;
            std::cout << "sure_loss: " << sure_loss << std::endl;
        }
    };
    
}   // namespace superstratum
}   // namespace Ten

#endif