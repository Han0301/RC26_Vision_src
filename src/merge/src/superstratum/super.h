#ifndef __SUPPER_H_
#define __SUPPER_H_
#include "./../serial.h"
#include "./../threadpool.h"
#include "./../livox_ros_driver2/src/livox_ros_driver.h"
#include "./../point_lio/src/laserMapping2.h"
#include "./../lidar.h"
#include "./../camera.h"
#include "./../camera2.h"
#include "./../relocation.h"
#include "./../coordinate.h"
#include "./../recognition/camera_calibration.h"
#include "./../recognition/world_to_camera.h"
#include "./../recognition/occlusion_handing.h"
#include "./../velocity.h"
#include "./../calibration.h"
#include "./../log/logger.h"
#include "./../yolo/yolo_v5.h"
#include "./../yolo/yolo_han.h"
#include "./../orb/orb_overall_exhaust.h"
#include "./../method_math.h"
#include "./../yolo/yolo_v11_cls.h"
#include "../parameter/parameter.h"
#include "./../nine/detector.h"


namespace Ten
{
    namespace superstratum
    {
        // #define _coner_path_ "/home/robocon/rc2026/model/corner5/best"
        // #define _juanzhou_path_ "/home/robocon/rc2026/model/juanZhou_cls1/best"
        // #define _box_num_ 5  //8

        class super
        {
        public:
        
            /**
            * @brief 比赛
            */
            super()
                :camera_(&Ten::Ten_camera::GetInstance())
                ,log_(&Ten::Ten_logger::GetInstance(std::string(ROOT_DIR) + std::string("log")))
                ,orb_determine_position_(_coner_path_, "cpu", 0.3, 0.3, 0, _box_num_)
                ,yolo_detector_(_juanzhou_path_, "cpu", mapping_)
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
                lidar_to_camera_transform_matrix_ = _lidar_to_camera_transform_matrix_; 
                _CAMERA_TRANSFORMATION_.camerainfo_.set_Extrinsic_Matrix(lidar_to_camera_transform_matrix_);
           }

           /**
            * @brief 调试用的debug函数
            */
           super(int)
                :camera_(nullptr)
                ,log_(&Ten::Ten_logger::GetInstance(std::string(ROOT_DIR) + std::string("log")))
                ,orb_determine_position_(_coner_path_, "cpu", 0.3, 0.3, 0)
                ,yolo_detector_(_juanzhou_path_, "cpu", mapping_)
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
                lidar_to_camera_transform_matrix_ = _lidar_to_camera_transform_matrix_; 
                _CAMERA_TRANSFORMATION_.camerainfo_.set_Extrinsic_Matrix(lidar_to_camera_transform_matrix_);
           }

            /**
             * @brief 设置调试图像
             */
            void debug_set_image(Ten::ORB::orb_exhaust_element oee)
            {
                //设置orb推理要素
                orb_reasoning_image_.push_back(oee);
                //同时也设置yolocls推理要素
                yolocls_raw_reasoning_image_.push_back(oee);
            }

            /**
             * @brief 设置图像for orb
             */
            void set_image_for_orb()
            {
                //获取图片
                Ten::ORB::orb_exhaust_element oee;
                camera_->camera_read().copyTo(oee.image_);
                //获得雷达定位坐标
                // 位置变化
                nav_msgs::Odometry odo;
                if(!Ten::_TF_GET_.get_latest(odo))
                {
                    return;
                }
                Ten::XYZRPY lidar_of_world1 = Ten::Nav_Odometrytoxyzrpy(odo);
                // std::cout << "--------------lidar_of_world1-------------" << std::endl; 
                // std::cout << "x: " << lidar_of_world1._xyz._x << std::endl;
                // std::cout << "y: " << lidar_of_world1._xyz._y << std::endl;
                // std::cout << "z: " << lidar_of_world1._xyz._z << std::endl;
                // std::cout << "roll: " << lidar_of_world1._rpy._roll << std::endl;
                // std::cout << "pitch: " << lidar_of_world1._rpy._pitch << std::endl;
                // std::cout << "yaw: " << lidar_of_world1._rpy._yaw << std::endl;
                //判断是否无效
                if(std::isnan(lidar_of_world1._rpy._yaw)) 
                {
                    return;
                }
                //设置坐标变换
                //coordinate_transformation_.set_worldtolidar(lidar_of_world1);
                _CAMERA_TRANSFORMATION_.set_worldtolidar(lidar_of_world1);
                Eigen::Matrix4d world_to_camera = _CAMERA_TRANSFORMATION_.getXYZRPY_matrix();
                // std::cout << "-------------lidar_of_world2--------------" << std::endl; 
                // std::cout << "x: " << lidar_of_world2._xyz._x << std::endl;
                // std::cout << "y: " << lidar_of_world2._xyz._y << std::endl;
                // std::cout << "z: " << lidar_of_world2._xyz._z << std::endl;
                // std::cout << "roll: " << lidar_of_world2._rpy._roll << std::endl;
                // std::cout << "pitch: " << lidar_of_world2._rpy._pitch << std::endl;
                // std::cout << "yaw: " << lidar_of_world2._rpy._yaw << std::endl;
                //设置世界到相机
                //Eigen::Matrix4d world_to_camera = lidar_to_camera_transform_matrix_ * worldtocurrent(lidar_of_world2._xyz, lidar_of_world2._rpy);
                // std::cout << "world_to_camera: " << std::endl;
                // std::cout << world_to_camera << std::endl;
                transverter_.set_Extrinsic_Matrix(world_to_camera);
                //打印RT
                // std::cout << "rvec: " << std::endl;
                // std::cout << transverter_.rvec() << std::endl;
                // std::cout << "tvec: " << std::endl;
                // std::cout << transverter_.tvec() << std::endl;
                //设置orb推理要素
                transverter_.rvec().copyTo(oee.rvec_);
                transverter_.tvec().copyTo(oee.tvec_);
                orb_reasoning_image_.push_back(oee);
                // Ten::ORB::orb_exhaust_element oee2;
                // camera_->camera_read().copyTo(oee2.image_);
                // transverter_.rvec().copyTo(oee2.rvec_);
                // transverter_.tvec().copyTo(oee2.tvec_);
                //同时也设置yolocls推理要素
                yolocls_raw_reasoning_image_.push_back(oee);
            }

            /**
             * @brief 设置图像for yolocls
             */
            void set_image_for_yolo()
            {
                //获取图片
                Ten::ORB::orb_exhaust_element oee;
                camera_->camera_read().copyTo(oee.image_);
                //获得雷达定位坐标
                nav_msgs::Odometry odo;
                if(!Ten::_TF_GET_.get_latest(odo))
                {
                    return;
                }
                Ten::XYZRPY lidar_of_world1 = Ten::Nav_Odometrytoxyzrpy(odo);
                // std::cout << "---------------------------" << std::endl; 
                // std::cout << "x: " << lidar_of_world1._xyz._x << std::endl;
                // std::cout << "y: " << lidar_of_world1._xyz._y << std::endl;
                // std::cout << "z: " << lidar_of_world1._xyz._z << std::endl;
                // std::cout << "roll: " << lidar_of_world1._rpy._roll << std::endl;
                // std::cout << "pitch: " << lidar_of_world1._rpy._pitch << std::endl;
                // std::cout << "yaw: " << lidar_of_world1._rpy._yaw << std::endl;
                //判断是否无效
                if(std::isnan(lidar_of_world1._rpy._yaw)) 
                {
                    return;
                }
                //设置坐标变换
                //coordinate_transformation_.set_worldtolidar(lidar_of_world1);
                //Ten::XYZRPY lidar_of_world2 = coordinate_transformation_.getXYZRPY();
                _CAMERA_TRANSFORMATION_.set_worldtolidar(lidar_of_world1);
                Eigen::Matrix4d world_to_camera = _CAMERA_TRANSFORMATION_.getXYZRPY_matrix();
                std::cout << "world_to_camera" << std::endl;
                std::cout << world_to_camera << std::endl;
                // std::cout << "---------------------------" << std::endl; 
                // std::cout << "x: " << lidar_of_world2._xyz._x << std::endl;
                // std::cout << "y: " << lidar_of_world2._xyz._y << std::endl;
                // std::cout << "z: " << lidar_of_world2._xyz._z << std::endl;
                // std::cout << "roll: " << lidar_of_world2._rpy._roll << std::endl;
                // std::cout << "pitch: " << lidar_of_world2._rpy._pitch << std::endl;
                // std::cout << "yaw: " << lidar_of_world2._rpy._yaw << std::endl;
                //设置世界到相机
                //Eigen::Matrix4d world_to_camera = lidar_to_camera_transform_matrix_ * worldtocurrent(lidar_of_world2._xyz, lidar_of_world2._rpy);
                // std::cout << "world_to_camera: " << std::endl;
                // std::cout << world_to_camera << std::endl;
                transverter_.set_Extrinsic_Matrix(world_to_camera);
                //设置yolocls推理要素
                transverter_.rvec().copyTo(oee.rvec_);
                transverter_.tvec().copyTo(oee.tvec_);
                yolocls_raw_reasoning_image_.push_back(oee);
            }

            /**
             * @brief 检测位置，如果place判断成功返回true, 如果失败自动调用重定位并清除之前记录的所有数据
             * @param flag: 是否启动重地位，是 1 否 0；
             * @return 损失
             */
            double estimate_square_position()
            {
                //记录日志
                // for(size_t i = 0; i < orb_reasoning_image_.size(); i++)
                // {
                //     std::cout << "----orb-----" << i << "-----------" << std::endl;
                //     std::cout << "rvec: " << std::endl;
                //     std::cout << orb_reasoning_image_[i].rvec_ << std::endl;
                //     std::cout << "tvec: " << std::endl;
                //     std::cout << orb_reasoning_image_[i].tvec_ << std::endl;
                // }
                // for(size_t i = 0; i < yolocls_raw_reasoning_image_.size(); i++)
                // {
                //     std::cout << "---yolocls---" << i << "-----------" << std::endl;
                //     std::cout << "rvec: " << std::endl;
                //     std::cout << yolocls_raw_reasoning_image_[i].rvec_ << std::endl;
                //     std::cout << "tvec: " << std::endl;
                //     std::cout << yolocls_raw_reasoning_image_[i].tvec_ << std::endl;
                // }

                //保存原始图像以及rt
                //save_logRT();
                //进行orb推理，返回位置以及损失
                std::vector<int> place = orb_determine_position_.getplace(orb_reasoning_image_);
                std::cout << "place: ";
                for(auto it : place)
                {
                    std::cout << it << ",";
                }
                std::cout << std::endl;
                std::vector<double> losses = orb_determine_position_.get_loss();
                for(size_t i = 0; i < losses.size(); i++)
                {
                    std::cout << "loss[" << i << "]: " << losses[i] << std::endl;
                }            

                //记录方块位置
                place_ = place;
                for(size_t i = 0; i < place_.size(); i++)
                {
                    //如果位置是空
                    if(place_[i] == 0)
                    {
                        //类别为4
                        classifier_[i] = 4;
                        classifier2_[i] = 4;
                    }
                }
                return losses[0];
            }

            /**
             * @brief 检测类别
             * @return std::vector<int>
             */
            std::vector<int> estimate_classifier(int flag_clear = 0)
            {
                //优化yolo推理需要用到原生数据的位置姿态
                std::vector<Ten::ORB::orb_exhaust_element> oprt = orb_determine_position_.get_RT(yolocls_raw_reasoning_image_);
                //保存原始图像以及优化后的rt
                //log_->record_imageRT(oprt);
                //调试box位置
                // int debug_exist_box[12] = {0};
                // for(auto& e : debug_exist_box)
                // {
                //     e = 1;
                // }

                //设置位置控制
                int exist_box[12] = {0};
                std::cout<< "exist_box: ";
                for(size_t k = 0; k < place_.size(); k++)
                {
                    exist_box[k] = place_[k];
                    std::cout << exist_box[k] << " ";
                }
                std::cout << std::endl;
                std::cout << "oprt.size(): " << oprt.size() << std::endl;

                // //保存旧数据方便调试
                // for(size_t i = 0; i < yolocls_raw_reasoning_image_.size(); i++)
                // {
                //     //世界点和box_list的类对象，对里面数据进行处理
                //     Ten::init_3d_box world_point;
                //     camera_transformation_.camerainfo_.set_RT(yolocls_raw_reasoning_image_[i].rvec_, yolocls_raw_reasoning_image_[i].tvec_);
                //     camera_transformation_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
                //     world_point.pcl_to_C();
                //     zbuffer_.set_exist_boxes(debug_exist_box);
                //     zbuffer_.set_box_lists_(yolocls_raw_reasoning_image_[i].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
                //     log_->record_image(world_point.box_lists_);
                //     //设置yolocls推理的12个roi数据
                //     //yolocls_ripe_roi_reasoning_image_.push_back(world_point);
                // }

                //设置yolocls推理的12个roi数据
                for(size_t i = 0; i < oprt.size(); i++)
                {
                    //世界点和box_list的类对象，对里面数据进行处理
                    Ten::init_3d_box world_point;
                    camera_transformation_.camerainfo_.set_RT(oprt[i].rvec_, oprt[i].tvec_);
                    camera_transformation_.pcl_transform_world_to_camera(world_point.pcl_LM_plum_object_points_, world_point.pcl_C_plum_object_points_, world_point.object_plum_2d_points_);
                    world_point.pcl_to_C();
                    zbuffer_.set_exist_boxes(exist_box);
                    zbuffer_.set_box_lists_(oprt[i].image_, world_point.C_object_plum_points_, world_point.object_plum_2d_points_, world_point.box_lists_);
                    //log_->record_image(world_point.box_lists_);
                    //设置yolocls推理的12个roi数据
                    yolocls_ripe_roi_reasoning_image_.push_back(world_point);
                }

                //yolo推理
                std::vector<Ten::box> box_lists_save;
                box_lists_save.resize(12);
                for(int i = 0; i < 12; i++)
                {
                    box_lists_save[i].idx = i + 1;
                    box_lists_save[i].roi_image = cv::Mat::zeros(160, 160, CV_8UC3);
                }
                
                for(size_t epoch = 0; epoch < yolocls_ripe_roi_reasoning_image_.size(); epoch++)
                {
                    std::vector<Ten::box>& box_lists = yolocls_ripe_roi_reasoning_image_[epoch].box_lists_;
                    for(size_t i = 0; i < box_lists.size(); i++)
                    {
                        //如果位置有方块
                        if(place_[i] == 1)
                        {
                            //推理
                            std::vector<Ten::yolo::Detection> result = yolo_detector_.worker(box_lists[i].roi_image);
                            box_lists[i].cls = result[0].cls_id_;
                            box_lists[i].confidence = result[0].conf_;
                            //忽略32类
                            if(box_lists[i].cls == 32)
                                continue;
                            //std::cout << "last conf"<< confidence_[i] << std::endl;
                            //std::cout << "now conf" << result[0].conf_ <<std::endl;
                            if(confidence_[i] < result[0].conf_)
                            {
                                confidence_[i] = result[0].conf_;
                                classifier_[i] = result[0].cls_id_;
                                //std::cout << "result[0].cls_id_: "<< i+1 << " " << result[0].cls_id_ << std::endl;
                                classifier2_[i] = mapping2_[result[0].cls_id_];
                                box_lists_save[i].cls = result[0].cls_id_;
                                box_lists_save[i].confidence = result[0].conf_;
                                box_lists[i].roi_image.copyTo(box_lists_save[i].roi_image);
                            }
                        }
                    }   
                    //保存每一轮
                    //log_->record_image(box_lists);
                }
                //保存最佳结果
                //log_->record_image(box_lists_save);

                // //清理数据
                // orb_reasoning_image_.clear();
                // yolocls_raw_reasoning_image_.clear();
                // yolocls_ripe_roi_reasoning_image_.clear();

                if(flag_clear)
                {
                    //清空记忆
                    for(size_t i = 0; i < confidence_.size(); i++)
                    {
                        confidence_[i] = 0;
                    }
                    //清空记忆
                    for(size_t i = 0; i < classifier_.size(); i++)
                    {
                        classifier_[i] = 0;
                        classifier2_[i] = 0;
                    }
                }
                return classifier2_;
            }

            /**
             * @brief 清理数据
             */
            void clear()
            {
                //清理数据
                orb_reasoning_image_.clear();
                yolocls_raw_reasoning_image_.clear();
                yolocls_ripe_roi_reasoning_image_.clear();
                
            }


            /**
             * 保存图像
             */
            void save_log()
            {
                log_->record_imageRT(yolocls_raw_reasoning_image_, place_);
            }

            /**
             * @brief 普通重定位
             */
            static void use_relocation()
            {
                //重定位处理器，基于点云地图的重定位
                Ten::Ten_relocation<pcl::PointXYZI> relocation_base_of_map_(std::string(ROOT_DIR) + std::string("map/map.pcd")); 
                //获得这个世界的坐标系在地图坐标系的x,y,z,roll,pitch,yaw
                Ten::XYZRPY world2toworld1 = relocation_base_of_map_.get_transformation();
                _CAMERA_TRANSFORMATION_.set_world2toworld1(world2toworld1);
            }

            /**
             * @brief 特殊的重定位
             */
            static void use_relocation2()
            {
                //重定位处理器，基于点云地图的重定位
                Ten::Ten_relocation<pcl::PointXYZI> relocation_base_of_map_(std::string(ROOT_DIR) + std::string("map/map.pcd")); 
                //获得这个世界的坐标系在地图坐标系的x,y,z,roll,pitch,yaw
                Ten::XYZRPY xyzrpy = relocation_base_of_map_.get_transformation();
            
                std::cout << "---------------------------" << std::endl; 
                std::cout << "x: " << xyzrpy._xyz._x << std::endl;
                std::cout << "y: " << xyzrpy._xyz._y << std::endl;
                std::cout << "z: " << xyzrpy._xyz._z << std::endl;
                std::cout << "roll: " << xyzrpy._rpy._roll << std::endl;
                std::cout << "pitch: " << xyzrpy._rpy._pitch << std::endl;
                std::cout << "yaw: " << xyzrpy._rpy._yaw << std::endl;
            
                Ten::XYZRPY xyzrpy_error;
                xyzrpy_error._xyz._x = 0.025;
                xyzrpy_error._xyz._y = -0.045;
                xyzrpy_error._xyz._z = 0.10;
                xyzrpy_error._rpy._roll = 0;
                xyzrpy_error._rpy._pitch = 0;
                xyzrpy_error._rpy._yaw = 0;
                // T3 = T3‘ * T3’‘
                Ten::XYZRPY world_origin = Ten::transform_matrixtoXYZRPY(Ten::worldtocurrent(xyzrpy._xyz, xyzrpy._rpy) * Ten::worldtocurrent(xyzrpy_error._xyz, xyzrpy_error._rpy));
                Ten::_CAMERA_TRANSFORMATION_.set_world2toworld1(world_origin);
            }

            /**
             * @brief 设置重定位后的坐标
             * @param world2toworld1: 获得这个世界的坐标系在地图坐标系的x,y,z,roll,pitch,yaw
             */
            void set_relocation(Ten::XYZRPY world2toworld1)
            {
                //获得这个世界的坐标系在地图坐标系的x,y,z,roll,pitch,yaw
                //Ten::XYZRPY world2toworld1 = relocation_base_of_map_.get_transformation();
                _CAMERA_TRANSFORMATION_.set_world2toworld1(world2toworld1);
            }

            /**
             * @brief 保存rt
             */
            void save_logRT()
            {
                // std::string map_path = std::string(ROOT_DIR) + std::string("path/map.txt");
                // std::vector<int> map = Ten::readNumberFile(map_path);
                //log_->record_imageRT(orb_reasoning_image_, map);
                log_->record_imageRT(yolocls_raw_reasoning_image_, place_);
            }

            /**
             * @brief 获得位置
             * @return std::vector<int>
             */
            std::vector<int> get_place()
            {
                return place_;
            }


        private:
        Ten::Ten_camera* camera_; //相机实例的引用
        Ten::Ten_logger* log_; //日志实例的引用
        std::vector<int> mapping_ = {1, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 2, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 3, 30, 31, 32, 4, 5, 6, 7, 8, 9}; //yolocls模型类别映射关系
        std::vector<int> mapping2_ = {-1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, -1};//映射表
        std::vector<Ten::ORB::orb_exhaust_element> orb_reasoning_image_; //orb推理方块所位置需要用到的要素
        std::vector<Ten::ORB::orb_exhaust_element> yolocls_raw_reasoning_image_; //yolocls推理方块类别所需要用到的原生数据
        std::vector<Ten::init_3d_box> yolocls_ripe_roi_reasoning_image_; //yolocls推理方块类别所需要用到的roi图像
        Ten::Ten_camerainfo transverter_; //转换器，用于将4*4的变化矩阵转为rvec,tvec
        Ten::Ten_occlusion_handing zbuffer_; //zb处理器，用于处理遮挡关系
        Ten::ORB::orb_optimize_exhaust orb_determine_position_; //orb处理器，用于判断位置以及优化位姿变换
        Ten::yolo::yolo_v11_cls yolo_detector_; //yolo处理器，用于判断方块类别
        Ten::Ten_worldtocamera camera_transformation_; //坐标点转换器，用于将世界坐标系下的点变换到当前坐标系，以及像素坐标系
        //Ten::Ten_coordinate coordinate_transformation_; //坐标系转换器，用于得到当前坐标系的相对于世界坐标系的坐标
        std::vector<int> place_ = {1,1,1,1, 1,1,1,1, 1,1,1,1}; //每个位置是否有方块
        std::vector<int> classifier_ = {0,0,0,0, 0,0,0,0, 0,0,0,0}; //每个位置对应的类别信息，1~32
        std::vector<int> classifier2_ = {0,0,0,0, 0,0,0,0, 0,0,0,0}; //每个位置对应的类别信息，0：未知 、1：R1 、2：R2 、3：fake 、4：空
        std::vector<double> confidence_ = {0,0,0,0, 0,0,0,0, 0,0,0,0}; //每个位置对应的置信度
        Eigen::Matrix4d lidar_to_camera_transform_matrix_; //雷达到相机的变化矩阵
        };
    }


}


#endif
