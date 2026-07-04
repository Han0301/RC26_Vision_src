#include "controlR2.h"



namespace Ten
{
    std::atomic<double> _r2_x_;
    std::atomic<double> _r2_y_;
    std::atomic<double> _r2_z_;
    std::atomic<double> _r2_yaw_;

    /**
     * @brief 读取txt文件，数字直接读数值，符号转ASCII码，返回int向量，并打印原始文件内容
     * @param filePath txt文件的路径字符串
     * @return std::vector<int> 数字存原值，符号存ASCII码（忽略空格/换行/回车）
     */
    std::vector<int> readFileToAsciiVector(const std::string& filePath)
    {
        // 1. 以只读模式打开文件
        std::ifstream fileStream(filePath, std::ios::in);
        std::vector<int> result;

        // 判断文件是否成功打开
        if (!fileStream.is_open())
        {
            std::cerr << "错误：无法打开文件！路径：" << filePath << std::endl;
            return result;
        }

        // 2. 一次性读取文件全部内容到字符串中
        std::string fileContent((std::istreambuf_iterator<char>(fileStream)),
                                std::istreambuf_iterator<char>());
        fileStream.close(); // 读取完成后关闭文件

        // 3. 核心要求：打印【读取到的原始文件字符串】（不变）
        std::cout << "=== 读取到的文件原始内容 ===" << std::endl;
        std::cout << fileContent << std::endl;
        std::cout << "==========================" << std::endl;

        // 4. 遍历处理：严格按分隔符解析数字，符号转ASCII
        std::string numBuffer;
        for (char ch : fileContent)
        {
            // ===================== 核心分隔规则 =====================
            // 1. 空格/换行/回车：仅作为分隔符，跳过，同时终止当前数字拼接
            if (ch == ' ' || ch == '\n' || ch == '\r')
            {
                // 如果有未保存的数字，先存入结果
                if (!numBuffer.empty())
                {
                    result.push_back(std::stoi(numBuffer));
                    numBuffer.clear();
                }
                continue;
            }

            // 2. 数字字符：连续拼接（多位数合并）
            if (std::isdigit(static_cast<unsigned char>(ch)))
            {
                numBuffer += ch;
            }
            // 3. 符号字符（( ) 等）：作为分隔符，先存数字，再存符号ASCII
            else
            {
                if (!numBuffer.empty())
                {
                    result.push_back(std::stoi(numBuffer));
                    numBuffer.clear();
                }
                result.push_back(static_cast<int>(ch));
            }
        }

        // 处理文件末尾最后一个数字
        if (!numBuffer.empty())
        {
            result.push_back(std::stoi(numBuffer));
        }

        int flag  = 1;
        Ten::_global_path_.clear();
        for(size_t i = 0; i < result.size(); i++)
        {
            if(result[i] == '(')
            {
                flag = 0;
                continue;
            }
            else if(result[i] == ')')
            {
                flag = 1;
                continue;
            }
            else if(result[i] == 0)
            {
                flag = 0;
                continue;
            }

            if(flag)
            {
                Ten::_global_path_.push_back(result[i]);
            }
        }
        return result;
    }

    /**
     * @brief 读取txt中所有整数（空格/换行任意分隔），返回int数组
     * @param file_path： 参数：txt文件路径
     * @return 存储所有数字的vector<int>
     */
    std::vector<int> readIntTxtR2(const std::string& file_path)
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
    void responseR2(int data)
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

    void pathCallbackR2(const std_msgs::Int32::ConstPtr& msg)
    {
        static Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
        static int receive = 0;
        static int last = 0;
        static uint8_t MFpath[30];

        receive = msg->data;
        if(receive != last)
        {
            last = receive;
            std::vector<int> path = readFileToAsciiVector(std::string(ROOT_DIR) + std::string("path/map.txt"));
            uint8_t length = path.size();

            if(length)
            {
                for(size_t i = 0; i < path.size() && i < 30; i++)
                {
                    MFpath[i] = (uint8_t)path[i];
                }
                Ten::_MAP_FLAG_.set_flag(1);
                while(Ten::_MAP_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag())
                {
                    std::cout << "serial.serial_send(MFpath, 7, length);" << std::endl;
                    serial.serial_send(MFpath, 7, length);
                    usleep(100*1000);
                }
                responseR2(last);
            }
        }
        else
        {
            responseR2(last);
        }
    }

    void lidarrestartCallbackR2(const std_msgs::Int32::ConstPtr& msg)
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
            responseR2(last);
        }
        else
        {
            responseR2(last);
        }
    }

    void parameterCallbackR2(const std_msgs::Int32::ConstPtr& msg)
    {
        static Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
        static int receive = 0;
        static int last = 0;
        static uint8_t MFparameter[30];

        receive = msg->data;
        if(receive != last)
        {
            last = receive;
            std::vector<int> param = readIntTxtR2(std::string(ROOT_DIR) + std::string("path/parameter.txt"));
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
                Ten::_PARAMETER_FLAG_.set_flag(1);
                if(Ten::_input_parameter_[5] == 1)
                {
                    if(Ten::_input_parameter_[0] == 0)
                    {
                        Ten::superstratum::_r2_xyzrpy_init_error_xyz_x_.store(0.375);
                        Ten::superstratum::_r2_xyzrpy_init_error_xyz_y_.store(4.58);
                    }
                    else if(Ten::_input_parameter_[0] == 1)
                    {
                        Ten::superstratum::_r2_xyzrpy_init_error_xyz_x_.store(0.375);
                        Ten::superstratum::_r2_xyzrpy_init_error_xyz_y_ .store(-4.58);
                    }
                }
                while(Ten::_PARAMETER_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag())
                {
                    std::cout << "serial.serial_send(MFparameter, 9, length);" << std::endl;
                    serial.serial_send(MFparameter, 9, length);
                    usleep(100*1000);
                }
                responseR2(last);
            }
        }
        else
        {
            responseR2(last);
        }
    }

    // void script_control2()
    // {
    //     ros::NodeHandle nh;
    //     // 订阅int话题
    //     ros::Subscriber send_path = nh.subscribe("/scripts/send_path", 3, pathCallback);
    //     ros::Rate loop_rate(100);  
    //     // 你的线程循环条件
    //     while (Ten::_TREADPOOL_FLAG_.read_flag())  
    //     {
    //         ros::spinOnce();  
    //         loop_rate.sleep();  
    //     }
    // }


    void cameraside()
    {
        Ten::inspection sp;
        sp.cameraside();
    }

    void selfinspection(const std_msgs::Int32::ConstPtr& msg)
    {
        static int receives[10] = {0};
        static int lasts[10] = {0};
        static ThreadPool pool(1);

        int receive = msg->data;
        if(receive == 1 || receive == 2)
        {
            receives[0] = receive;
            if(lasts[0] != receives[0])
            {
                lasts[0] = receives[0];
                Ten::_CAMERA_CLOSE_FLAG_.set_flag(true);
                pool.enqueue(cameraside);
            }
            responseR2(receive);
        }
    }

    void selfinspectionclose(const std_msgs::Int32::ConstPtr& msg)
    {
        static int receives[10] = {0};
        static int lasts[10] = {0};
        static ThreadPool pool(1);

        int receive = msg->data;
        if(receive == 1 || receive == 2)
        {
            receives[0] = receive;
            if(lasts[0] != receives[0])
            {
                Ten::_CAMERA_CLOSE_FLAG_.set_flag(false);
            }
            responseR2(receive);
        }
    }
  

    void script_controlR2()
    {
        // 1. 创建当前线程私有独立回调队列，和其他线程完全隔离
        ros::CallbackQueue queue_script;
        // 2. 创建句柄并绑定私有队列，该nh下所有订阅消息都进这个私有队列
        ros::NodeHandle nh;
        nh.setCallbackQueue(&queue_script);

        // 3. 订阅话题，消息只会投递到 queue_script，不走全局默认队列
        ros::Subscriber send_path = nh.subscribe("/scripts/send_path", 3, pathCallbackR2);
        ros::Subscriber restart_lidar = nh.subscribe("/scripts/restart_lidar", 1, lidarrestartCallbackR2);
        ros::Subscriber parameter_path = nh.subscribe("/scripts/send_param", 3, parameterCallbackR2);
        ros::Subscriber inspection_camera = nh.subscribe("/scripts/check", 3, selfinspection);
        ros::Subscriber inspection_close = nh.subscribe("/scripts/global_check_off", 3, selfinspectionclose);

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
    void tf_monitorR2(nav_msgs::Odometry msg)
    {
        static float arr[4] = {0};
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
            result._xyz._x += Ten::superstratum::_r2_xyzrpy_init_error_xyz_x_.load();
            result._xyz._y += Ten::superstratum::_r2_xyzrpy_init_error_xyz_y_.load();
            //debug.pose = result;
            arr[0] = result._xyz._x;
            arr[1] = result._xyz._y;
            arr[2] = result._xyz._z;
            arr[3] = result._rpy._yaw;
            serial.serial_send(arr, 1, size);
            Ten::superstratum::controlR2::publishOdometry(debug, msg.header.stamp);
        }

        if(Ten::_POINT_LIO_RUN_FLAG_.read_flag() && !Ten::_POINT_LIO_CHANGE_FLAG_.read_flag())
        {
            //变化
            Ten::_COORDINATE_TRANSFORMATION_.set_worldtolidar(pose_fastlio);
            Ten::XYZRPY result = Ten::_COORDINATE_TRANSFORMATION_.getXYZRPY_incline();
            //debug.pose = result;
            result._xyz._x += Ten::superstratum::_r2_xyzrpy_init_error_xyz_x_.load();
            result._xyz._y += Ten::superstratum::_r2_xyzrpy_init_error_xyz_y_.load();
            // arr[0] = result._xyz._x;
            // arr[1] = result._xyz._y;
            // arr[2] = result._xyz._z;
            // arr[3] = result._rpy._yaw;
            // serial.serial_send(arr, 1, size);
            _r2_x_.store(result._xyz._x);
            _r2_y_.store(result._xyz._y);
            _r2_z_.store(result._xyz._z);
            Ten::superstratum::controlR2::publishOdometry(debug, msg.header.stamp);
            //std::cout << "Ten::_POINT_LIO_RUN_FLAG_.read_flag() && !Ten::_POINT_LIO_CHANGE_FLAG_.read_flag()" << std::endl;
        }
       

    }

    

    // 里程计消息回调函数
    void odomCallbackR2(const nav_msgs::Odometry::ConstPtr& msg)
    {
        tf_monitorR2(*msg);
    }

    // 线程1：只处理odom话题，用独立私有队列
    void LoopcallbackR2()
    {
        urcu_memb_register_thread();
        ros::CallbackQueue queue_odom;   // 线程私有，不与其他线程共享
        ros::NodeHandle nh;
        nh.setCallbackQueue(&queue_odom); // 当前句柄的所有订阅/服务都绑定到该队列
        ros::Subscriber odom_sub = nh.subscribe("/fast_lio2/odom", 10, odomCallbackR2);

        ros::Rate loop_rate(20);
        while (Ten::_TREADPOOL_FLAG_.read_flag())
        {
            queue_odom.callAvailable(ros::WallDuration(0.03)); // 只消费自己的队列
            loop_rate.sleep();
        }
        urcu_memb_unregister_thread();
    }



    void R2_mapping_fast()
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

