#include "controlR1.h"



namespace Ten
{

    const double _vmax_ = 0.2;
    const double _vyawmax_ = 3.14;

    /**
     * @brief 读取txt中所有整数（空格/换行任意分隔），返回int数组
     * @param file_path： 参数：txt文件路径
     * @return 存储所有数字的vector<int>
     */
    std::vector<int> readIntTxtR1(const std::string& file_path)
    {
        std::vector<int> result;
        // 创建文件输入流
        std::ifstream fin(file_path);

        // 判断文件是否成功打开
        if (!fin.is_open())
        {
            std::cerr << "错误：无法打开文件 " << file_path << std::endl;
            return result;
        }

        int num;
        // while循环持续读取整数，cin/ifstream默认自动忽略空格、换行、制表符
        while (fin >> num)
        {
            result.push_back(num);
        }

        // 关闭文件流（离开作用域会自动析构，手动关闭更规范）
        fin.close();
        return result;
    }

    /**
     * @brief 发布 std_msgs::Int32 消息
     * @param data 输入：需要发布的 int 类型数据
     */
    void responseR1(int data)
    {
        static ros::Publisher int_pub;
        if (!int_pub)
        {
            ros::NodeHandle nh;
            int_pub = nh.advertise<std_msgs::Int32>("/scripts/response", 3);
        }
        std_msgs::Int32 int_msg;
        int_msg.data = data;
        int_pub.publish(int_msg);
    }

    void lidarrestartCallbackR1(const std_msgs::Int32::ConstPtr& msg)
    {
        static int receive = 0;
        static int last = 0;

        receive = msg->data;
        if(receive != last)
        {
            last = receive;
            Ten::_LASERMAPPING_FLAG_.set_flag(true);
            Ten::_POINT_LIO_RUN_FLAG_.set_flag(false);
            Ten::_FAST_LIO_MAPING_FLAG_.set_flag(false);
            responseR1(last);
        }
        else
        {
            responseR1(last);
        }
    }

    void parameterCallbackR1(const std_msgs::Int32::ConstPtr& msg)
    {
        // static Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
        static int receive = 0;
        static int last = 0;
        static uint8_t MFparameter[30];

        receive = msg->data;
        if(receive != last)
        {
            last = receive;
            std::vector<int> param = readIntTxtR1(std::string(ROOT_DIR) + std::string("path/parameter.txt"));
            uint8_t length = param.size();
            if(length)
            {
                for(size_t i = 0; i < param.size() && i < 30; i++)
                {
                    MFparameter[i] = (uint8_t)param[i];
                    Ten::_input_parameter_[i] = param[i];
                    // std::cout << "------------------" << std::endl;
                    // for(size_t i = 0; i < param.size() && i < 30; i++)
                    // {
                    //     MFparameter[i] = (uint8_t)param[i];
                    //     Ten::_input_parameter_[i] = param[i];
                    //     std::cout << " " << param[i];
                    // }
                    // std::cout << std::endl;
                    // std::cout << "------------------" << std::endl;

                }
                // Ten::_PARAMETER_FLAG_.set_flag(1);
                if(Ten::_input_parameter_[5] == 1)
                {
                    if(Ten::_input_parameter_[0] == 0)
                    {
                        Ten::superstratum::_r1_xyzrpy_init_error_xyz_x_.store(0.505);
                        Ten::superstratum::_r1_xyzrpy_init_error_xyz_y_.store(0.45);
                    }
                    else if(Ten::_input_parameter_[0] == 1)
                    {
                        Ten::superstratum::_r1_xyzrpy_init_error_xyz_x_.store(0.505);
                        Ten::superstratum::_r1_xyzrpy_init_error_xyz_y_ .store(-0.45);
                    }
                }
                // while(Ten::_PARAMETER_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag())
                // {
                //     std::cout << "serial.serial_send(MFparameter, 9, length);" << std::endl;
                //     //serial.serial_send(MFparameter, 9, length);
                //     usleep(100*1000);
                // }
                responseR1(last);
            }
        }
        else
        {
            responseR1(last);
        }
    }

    void script_controlR1()
    {
        // 1. 创建当前线程私有独立回调队列，和其他线程完全隔离
        ros::CallbackQueue queue_script;
        // 2. 创建句柄并绑定私有队列，该nh下所有订阅消息都进这个私有队列
        ros::NodeHandle nh;
        nh.setCallbackQueue(&queue_script);

        // 3. 订阅话题，消息只会投递到 queue_script，不走全局默认队列
        ros::Subscriber restart_lidar = nh.subscribe("/scripts/restart_lidar", 1, lidarrestartCallbackR1);
        ros::Subscriber parameter_path = nh.subscribe("/scripts/send_param", 3, parameterCallbackR1);
        // 循环控频100Hz
        ros::Rate loop_rate(100);
        while (Ten::_TREADPOOL_FLAG_.read_flag())
        {
            // 只处理当前线程私有队列，不处理全局其他话题回调
            // 超时0.01s，无消息最多阻塞10ms就返回，避免卡死
            queue_script.callAvailable(ros::WallDuration(0.01));
            loop_rate.sleep();
        }
    }

    //位置监控
    void tf_monitorR1(nav_msgs::Odometry msg)
    {
        static float arr[9] = {0};
        static Ten::PV debug;
        static Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
        Ten::XYZRPY pose_fastlio = Ten::Nav_Odometrytoxyzrpy(msg);
        debug.pose = pose_fastlio;
        size_t size = sizeof(arr);

        if(Ten::_POINT_LIO_RUN_FLAG_.read_flag())
        {
            Ten::XYZRPY pose_point_lio;
            double v = 0;
            double vyaw = 0;
            ros::Rate sl(Ten::_laser_pub_hz_*2);
            while(Ten::_TREADPOOL_FLAG_.read_flag())
            {
                nav_msgs::Odometry odo;
                if(!Ten::_TF_GET_.get_latest(odo))
                {
                    sl.sleep();
                    continue;
                }
                pose_point_lio = Ten::Nav_Odometrytoxyzrpy(odo);
                v = std::sqrt(odo.twist.twist.linear.x*odo.twist.twist.linear.x + odo.twist.twist.linear.y*odo.twist.twist.linear.y);
                vyaw = std::abs(odo.twist.twist.angular.z);
                break;
            }
            //距离大于某个阈值
            double distance = pose_fastlio.Eclidean_distance(pose_point_lio);
            if(distance > 1.0)
            {            
                Ten::_POINT_LIO_RUN_FLAG_.set_flag(false);
            }

            if(v < _vmax_ && vyaw <= _vyawmax_)
            {
                Ten::_POINT_LIO_CHANGE_FLAG_.set_flag(false);
            }
            else
            {
                Ten::_POINT_LIO_CHANGE_FLAG_.set_flag(true);
            }
        }
        else
        {
            //变化
            Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose_fastlio);
            Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
            result._xyz._x += Ten::superstratum::_r1_xyzrpy_init_error_xyz_x_.load();
            result._xyz._y += Ten::superstratum::_r1_xyzrpy_init_error_xyz_y_.load();
            //debug.pose = result;
            arr[0] = result._xyz._x;
            arr[1] = result._xyz._y;
            arr[2] = result._xyz._z;
            arr[5] = result._rpy._yaw * 180.0 / M_PI;
            serial.serial_send(arr, 1, size);
            Ten::superstratum::controlR1::publishOdometry(debug, msg.header.stamp);
        }

        if(Ten::_POINT_LIO_RUN_FLAG_.read_flag() && !Ten::_POINT_LIO_CHANGE_FLAG_.read_flag())
        {
            //变化
            Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose_fastlio);
            Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
            //debug.pose = result;
            result._xyz._x += Ten::superstratum::_r1_xyzrpy_init_error_xyz_x_.load();
            result._xyz._y += Ten::superstratum::_r1_xyzrpy_init_error_xyz_y_.load();

            //发车
            debug.pose = result;

            arr[0] = result._xyz._x;
            arr[1] = result._xyz._y;
            arr[2] = result._xyz._z;
            arr[5] = result._rpy._yaw * 180.0 / M_PI;
            serial.serial_send(arr, 1, size);
            Ten::superstratum::controlR1::publishOdometry(debug, msg.header.stamp);
            //std::cout << "Ten::_POINT_LIO_RUN_FLAG_.read_flag() && !Ten::_POINT_LIO_CHANGE_FLAG_.read_flag()" << std::endl;
        }
       

    }

    // 里程计消息回调函数
    void odomCallbackR1(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf_monitorR1(*msg);
    }

    // 线程1：只处理odom话题，用独立私有队列
    void LoopcallbackR1()
    {
        urcu_memb_register_thread();
        ros::CallbackQueue queue_odom;   // 线程私有，不与其他线程共享
        ros::NodeHandle nh;
        nh.setCallbackQueue(&queue_odom); // 当前句柄的所有订阅/服务都绑定到该队列
        ros::Subscriber odom_sub = nh.subscribe("/fast_lio2/odom", 10, odomCallbackR1);

        ros::Rate loop_rate(20);
        while (Ten::_TREADPOOL_FLAG_.read_flag())
        {
            queue_odom.callAvailable(ros::WallDuration(0.03)); // 只消费自己的队列
            loop_rate.sleep();
        }
        urcu_memb_unregister_thread();
    }


    void R1_mapping_fast()
    {
        urcu_memb_register_thread();
        // ========== 1. 实例化建图模块 ==========
        // 测试时可把 max_points 改小（比如 20000），快速验证点数超限压缩逻辑
        Ten::mapping mapper;
        // ========== 2. 循环处理回调 ==========
        ros::Rate loop_rate(20);  // 20Hz 循环，与你其他线程频率保持一致
        while(Ten::_TREADPOOL_FLAG_.read_flag())
        {
            Ten::_FAST_LIO_MAPING_FLAG_.set_flag(true);
            while (Ten::_FAST_LIO_MAPING_FLAG_.read_flag())
            {
                mapper.callback(0.03);
                loop_rate.sleep(); 
            }
            mapper.clearMap();
        }
        urcu_memb_unregister_thread();
    }

    

}

