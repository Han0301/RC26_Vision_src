#include "controlR2.h"
#include <std_msgs/Int32.h>

namespace Ten
{
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

        return result;
    }


    /**
     * @brief 发布 std_msgs::Int32 消息
     * @param data 输入：需要发布的 int 类型数据
     */
    void response(int data)
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

    void pathCallback(const std_msgs::Int32::ConstPtr& msg)
    {
        static Ten::Ten_serial& serial = Ten::Ten_serial::GetInstance();
        static int receive = 0;
        static int last = 0;
        static uint8_t arr[30];

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
                    arr[i] = (uint8_t)path[i];
                }
                Ten::_MAP_FLAG_.set_flag(1);
                while(Ten::_MAP_FLAG_.read_flag() && Ten::_TREADPOOL_FLAG_.read_flag())
                {
                    std::cout << "serial.serial_send(arr, 7, length);" << std::endl;
                    serial.serial_send(arr, 7, length);
                    usleep(100*1000);
                }
                response(last);
            }
        }
        else
        {
            response(last);
        }
    }

    void script_control()
    {
        ros::NodeHandle nh;
        // 订阅int话题
        ros::Subscriber send_path = nh.subscribe("/scripts/send_path", 3, pathCallback);
        ros::Rate loop_rate(100);  
        // 你的线程循环条件
        while (Ten::_TREADPOOL_FLAG_.read_flag())  
        {
            ros::spinOnce();  
            loop_rate.sleep();  
        }
    }

}

