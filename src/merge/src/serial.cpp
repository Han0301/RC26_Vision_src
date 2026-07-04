#ifndef __SERIAL_CPP_
#define __SERIAL_CPP_
#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <string>
#include <mutex>
#include <unistd.h>
#include "serial.h"

namespace Ten
{

// //全局只有一个对象
// class Ten_serial
// {
// #define FRAME_HEAD_0 0xAA //帧头字节1
// #define FRAME_HEAD_1 0x55  // 帧头字节2
// #define FRAME_END_0 0xEE // 帧尾字节1
// public:
//     //禁用拷贝构造
//     Ten_serial(const Ten_serial& serial) = delete;
//     //禁用赋值
//     Ten_serial& operator=(const Ten_serial& serial) = delete;
//     /**
//         @brief 设置端口和波特率
//         @param port: 串口路径 "/dev/ttyUSB0" , "/dev/ttyACM0" ...
//         @param serial_bund: 波特率
//         @return Ten_serial& 返回Ten_serial实例
//     */
//     static Ten_serial& GetInstance(const std::string& port = "/dev/ttyACM0", const size_t& serial_baud = 9600)
//     {
//         //static Ten_serial* ten_serial = nullptr;
//         static std::unique_ptr<Ten_serial> ten_serial = nullptr;
//         std::call_once(serial_flag_, [port, serial_baud]() 
//         {
//             //ten_serial = new Ten_serial(port, serial_baud);
//             //ten_serial = std::make_unique<Ten_serial>(port, serial_baud); 
//             ten_serial = create(port, serial_baud);
//             std::cout << "init_serial" << std::endl;
//         });
//         if(!ten_serial->serial_.isOpen())
//         {
//             std::cout<<"ten_serial->serial_ is no open!"<<std::endl;
//             Ten_serial* tmp = nullptr;
//             return *tmp;
//         }
//         return *ten_serial;
//     }
//     /**
//         @brief 发送数据
//         @param p: 数据段（一维数组首元数的地址）
//         @param frame_id: id号
//         @param length: 数据长度（sizeof(type)*数组大小）
//         @return size_t 写入数据长度
//     */
//     size_t serial_send(void* p, uint8_t frame_id, uint8_t length)
//     {
//         std::lock_guard<std::mutex> lock(send_mtx_);
//         if(this == nullptr)
//         {
//             return 0;
//         }
//         // 构建数据帧：帧头+ID+数据长度+数据+CRC+帧尾
//         uint8_t buff_msg[length + 6] = {0};
//         buff_msg[0] = FRAME_HEAD_0;
//         buff_msg[1] = FRAME_HEAD_1;
//         buff_msg[2] = frame_id;
//         buff_msg[3] = length;
//         uint8_t* ps = (uint8_t*)p;
//         for (int q = 0; q < length; q++)
//         {
//             buff_msg[4 + q] = ps[q];
//         }
//         //异或校验
//         buff_msg[4 + length] = calculateXORcheck(&buff_msg[4], length);
//         buff_msg[5 + length] = FRAME_END_0;
//         size_t write_num = serial_.write(buff_msg, length + 6);
//         return write_num;
//     }
//     /**
//         @brief 串口数据接收函数
//         @param p: 数据段（一维数组首元数的地址）（返回值）
//         @param received_frame_id: id(返回值)
//         @param received_length: 数据长度（sizeof(type)*数组大小）（返回值）
//         @return bool 是否读取成功
//     */
//     bool serial_read(void* p, uint8_t& received_frame_id, uint8_t& received_length) {
//         std::lock_guard<std::mutex> lock(read_mtx_);
//         if(this == nullptr)
//         {
//             return false;
//         }
//         uint8_t byte;//声明临时变量存储当前读取的字节
//         //uint8_t* buff_msg = new uint8_t[128]{0};
//         uint8_t* buff_msg = (uint8_t*)p;
//         //循环处理串口缓冲区中的所有可用数据
//         while (serial_.available() > 0) {
//             //ros::Rate rate(1000);
//             if (serial_.read(&byte, 1) != 1) continue;
//             if (byte == FRAME_HEAD_0) {
//                 if (serial_.read(&byte, 1) != 1 || byte != FRAME_HEAD_1) continue;
//                 if (serial_.read(&received_frame_id, 1) != 1) continue;
//                 uint8_t data_length;
//                 if (serial_.read(&data_length, 1) != 1) continue;
//                 if(data_length > 128)
//                 {
//                     std::cout<<"data Too long"<<std::endl;
//                     continue;
//                 }
//                 received_length = data_length;
//                 if (serial_.read(buff_msg, received_length) != received_length) continue;
//                 uint8_t end_bytes[2];
//                 if (serial_.read(&end_bytes[0], 1) != 1 || calculateXORcheck(buff_msg, received_length) != end_bytes[0]) continue;
//                 if (serial_.read(&end_bytes[1], 1) != 1 || end_bytes[1] != FRAME_END_0) continue;
//                 //p = buff_msg;
//                 //rate.sleep(); // 休眠至满足100Hz频率
//                 usleep(1000);
//                 return true;
//             }
//         }
//         return false;
//     }
//     /*
//       @brief 检查串口是否打开
//       @return bool 
//     */
//     bool isOpen() const{
//         return serial_.isOpen();
//     }

//     ~Ten_serial()
//     {
//         std::lock_guard<std::mutex> lock_s(send_mtx_);
//         std::lock_guard<std::mutex> lock_r(read_mtx_);
//         if (serial_.isOpen())
//             serial_.close();
//     }


// private:
//     Ten_serial(const std::string& port, const size_t& serial_baud)//禁止外部初始化实例
//     {
//         // 循环检测串口状态，未打开则持续尝试初始化，直到打开成功
//         while (!serial_.isOpen())
//         {
//             // 配置串口设备路径：绑定传入的串口设备文件（如USB转串口设备路径）
//             serial_.setPort(port);
//             // 配置串口波特率：设置与外部设备一致的通信速率（如115200bps）
//             serial_.setBaudrate(serial_baud);
//             // 配置流控制：禁用流控制（无RTS/CTS硬件流控，无XON/XOFF软件流控）
//             serial_.setFlowcontrol(serial::flowcontrol_none);
//             // 配置校验位：禁用奇偶校验（数据传输无校验位，默认配置，显式声明增强可读性）
//             serial_.setParity(serial::parity_none); // default is parity_none
//             // 配置停止位：设置1位停止位（串口通信标准配置，用于标识一个字节传输结束）
//             serial_.setStopbits(serial::stopbits_one);
//             // 配置数据位：设置8位数据位（标准字节长度，支持ASCII码及扩展数据传输）
//             serial_.setBytesize(serial::eightbits);
//             // 创建超时配置对象：使用简单超时模式，设置超时时间为100毫秒
//             // 作用：串口操作（打开、读写）超过100ms未响应则判定为超时，避免无限阻塞
//             serial::Timeout time_out = serial::Timeout::simpleTimeout(100);  
//             // 将超时配置应用到串口对象，使上述超时规则生效
//             serial_.setTimeout(time_out);
//             // 尝试打开串口：根据前面配置的参数初始化串口并建立连接
//             // 若打开失败（如设备占用、路径错误），循环会继续重试
//             serial_.open();
//         }
//         std::cout<<"serial open!"<<std::endl;
//     }

//     static std::unique_ptr<Ten_serial> create(const std::string& port = "/dev/ttyACM0", const size_t& serial_baud = 9600) {
//         // 静态函数可访问私有构造函数，直接new对象后封装为unique_ptr
//         return std::unique_ptr<Ten_serial>(new Ten_serial(port, serial_baud));
//     }

//     //异或校验
//     int calculateXORcheck(const uint8_t* data, size_t length) {

//         uint8_t checksum = 0;
//         if (data == NULL || length == 0) {
//             return checksum; // 空指针或空数组返回0
//         }
//         for (size_t i = 0; i < length; i++) {
//             checksum ^= data[i]; // 逐字节异或累加
//         }
//         return checksum;
//     }
// serial::Serial serial_;
// std::mutex send_mtx_;
// std::mutex read_mtx_;
// static std::once_flag serial_flag_;
// };
// std::once_flag Ten_serial::serial_flag_;

    std::once_flag Ten_serial::serial_flag_;

    size_t Ten_serial::serial_send(void* p, uint8_t frame_id, uint8_t length)
    {
        std::lock_guard<std::mutex> lock(send_mtx_);
        if(this == nullptr)
        {
            return 0;
        }
        // 构建数据帧：帧头+ID+数据长度+数据+CRC+帧尾
        uint8_t buff_msg[length + 6] = {0};
        buff_msg[0] = FRAME_HEAD_0;
        buff_msg[1] = FRAME_HEAD_1;
        buff_msg[2] = frame_id;
        buff_msg[3] = length;
        uint8_t* ps = (uint8_t*)p;
        for (int q = 0; q < length; q++)
        {
            buff_msg[4 + q] = ps[q];
        }
        //异或校验
        buff_msg[4 + length] = calculateXORcheck(&buff_msg[4], length);
        buff_msg[5 + length] = FRAME_END_0;
        size_t write_num = serial_.write(buff_msg, length + 6);
        return write_num;
    }

    bool Ten_serial::serial_read(void* p, uint8_t& received_frame_id, uint8_t& received_length) {
        std::lock_guard<std::mutex> lock(read_mtx_);
        if(this == nullptr)
        {
            return false;
        }
        uint8_t byte;//声明临时变量存储当前读取的字节
        //uint8_t* buff_msg = new uint8_t[128]{0};
        uint8_t* buff_msg = (uint8_t*)p;
        //循环处理串口缓冲区中的所有可用数据
        while (serial_.available() > 0) {
            //ros::Rate rate(1000);
            if (serial_.read(&byte, 1) != 1) continue;
            if (byte == FRAME_HEAD_0) {
                if (serial_.read(&byte, 1) != 1 || byte != FRAME_HEAD_1) continue;
                if (serial_.read(&received_frame_id, 1) != 1) continue;
                uint8_t data_length;
                if (serial_.read(&data_length, 1) != 1) continue;
                if(data_length > 128)
                {
                    std::cout<<"data Too long"<<std::endl;
                    continue;
                }
                received_length = data_length;
                if (serial_.read(buff_msg, received_length) != received_length) continue;
                uint8_t end_bytes[2];
                if (serial_.read(&end_bytes[0], 1) != 1 || calculateXORcheck(buff_msg, received_length) != end_bytes[0]) continue;
                if (serial_.read(&end_bytes[1], 1) != 1 || end_bytes[1] != FRAME_END_0) continue;
                //p = buff_msg;
                //rate.sleep(); // 休眠至满足100Hz频率
                usleep(1000);
                return true;
            }
        }
        return false;
    }

    Ten_serial& Ten_serial::GetInstance(const std::string& port, const size_t& serial_baud)
    {
        //static Ten_serial* ten_serial = nullptr;
        static std::unique_ptr<Ten_serial> ten_serial = nullptr;
        std::call_once(serial_flag_, [port, serial_baud]() 
        {
            //ten_serial = new Ten_serial(port, serial_baud);
            //ten_serial = std::make_unique<Ten_serial>(port, serial_baud); 
            ten_serial = create(port, serial_baud);
            std::cout << "init_serial" << std::endl;
        });
        if(!ten_serial->serial_.isOpen())
        {
            std::cout<<"ten_serial->serial_ is no open!"<<std::endl;
            exit(-1);
        }
        return *ten_serial;
    }

    Ten_serial::Ten_serial(const std::string& port, const size_t& serial_baud)//禁止外部初始化实例
    {
        // 循环检测串口状态，未打开则持续尝试初始化，直到打开成功
        while (!serial_.isOpen())
        {
            // 配置串口设备路径：绑定传入的串口设备文件（如USB转串口设备路径）
            serial_.setPort(port);
            // 配置串口波特率：设置与外部设备一致的通信速率（如115200bps）
            serial_.setBaudrate(serial_baud);
            // 配置流控制：禁用流控制（无RTS/CTS硬件流控，无XON/XOFF软件流控）
            serial_.setFlowcontrol(serial::flowcontrol_none);
            // 配置校验位：禁用奇偶校验（数据传输无校验位，默认配置，显式声明增强可读性）
            serial_.setParity(serial::parity_none); // default is parity_none
            // 配置停止位：设置1位停止位（串口通信标准配置，用于标识一个字节传输结束）
            serial_.setStopbits(serial::stopbits_one);
            // 配置数据位：设置8位数据位（标准字节长度，支持ASCII码及扩展数据传输）
            serial_.setBytesize(serial::eightbits);
            // 创建超时配置对象：使用简单超时模式，设置超时时间为100毫秒
            // 作用：串口操作（打开、读写）超过100ms未响应则判定为超时，避免无限阻塞
            serial::Timeout time_out = serial::Timeout::simpleTimeout(100);  
            // 将超时配置应用到串口对象，使上述超时规则生效
            serial_.setTimeout(time_out);
            // 尝试打开串口：根据前面配置的参数初始化串口并建立连接
            // 若打开失败（如设备占用、路径错误），循环会继续重试
            serial_.open();
        }
        std::cout<<"serial open!"<<std::endl;
    }

    //异或校验
    int Ten_serial::calculateXORcheck(const uint8_t* data, size_t length) {

        uint8_t checksum = 0;
        if (data == NULL || length == 0) {
            return checksum; // 空指针或空数组返回0
        }
        for (size_t i = 0; i < length; i++) {
            checksum ^= data[i]; // 逐字节异或累加
        }
        return checksum;
    }

        /**
     * @brief 清空串口缓冲区（接收+发送），清除残留的脏数据
     * @param clear_type: 清空类型（0=全清，1=仅接收，2=仅发送）
     * @return bool 清空是否成功（串口未打开/空指针返回false）
     */
    bool Ten_serial::clearBuffer(int clear_type) {
        // 加锁：避免和读写操作冲突，保证线程安全
        std::lock_guard<std::mutex> lock(send_mtx_);
        std::lock_guard<std::mutex> lock2(read_mtx_);

        // 空指针检查
        if (this == nullptr) {
            std::cout << "Ten_serial实例为空，无法清空缓冲区！" << std::endl;
            return false;
        }

        // 串口未打开检查
        if (!serial_.isOpen()) {
            std::cout << "串口未打开，无法清空缓冲区！" << std::endl;
            return false;
        }

        try {
            switch (clear_type) {
                case 0: // 清空接收+发送缓冲区（默认）
                    serial_.flush(); // serial库的flush() = 清空接收+发送
                    break;
                case 1: // 仅清空接收缓冲区
                    serial_.flushInput();
                    break;
                case 2: // 仅清空发送缓冲区
                    serial_.flushOutput();
                    break;
                default:
                    std::cout << "清空类型错误，默认清空所有缓冲区！" << std::endl;
                    serial_.flush();
                    break;
            }
            std::cout << "串口缓冲区清空成功（类型：" << clear_type << "）" << std::endl;
            return true;
        } catch (const serial::IOException& e) {
            // 捕获serial库的IO异常（如串口断开）
            std::cout << "清空缓冲区失败：" << e.what() << std::endl;
            return false;
        }
    }



}



#endif

