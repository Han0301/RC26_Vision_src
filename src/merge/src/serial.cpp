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

    std::once_flag Ten_serial::serial_flag_;

    /**
     * @brief 串口通信帧结构（固定 6 字节开销 + 可变数据段）
     *
     *  Byte[0]      Byte[1]      Byte[2]   Byte[3]   Byte[4..4+len-1]  Byte[4+len]  Byte[5+len]
     * ┌──────────┬──────────┬─────────┬─────────┬───────────────────┬────────────┬──────────┐
     * │ 帧头 0xAA │ 帧头 0x55 │ 帧 ID   │ 数据长度 │     数据段        │  异或校验   │ 帧尾 0xEE │
     * │ HEAD_0   │ HEAD_1   │ frame_id│ length  │   p[0..len-1]     │  XOR(data)  │ END_0    │
     * └──────────┴──────────┴─────────┴─────────┴───────────────────┴────────────┴──────────┘
     *                                                                                         
     *  帧总长度 = length + 6
     *  数据段最大长度 = 128（受 serial_read 限制）
     *  校验方式：数据段逐字节异或（XOR）
     */
    size_t Ten_serial::serial_send(void* p, uint8_t frame_id, uint8_t length)
    {
        // 1. 互斥锁（线程安全，保留）
        std::lock_guard<std::mutex> lock(send_mtx_);
        // 2. 关键参数合法性校验（防止野指针/空帧）
        if (this == nullptr || p == nullptr)
        {
            std::cerr << "发送失败：参数为空/数据长度为0" << std::endl;
            return 0;
        }

        try
        {
            // 3. 替换不安全栈数组 → 标准C++ 安全动态缓冲区
            size_t frame_total_len = length + 6; // 帧总长度：6字节固定头+尾+校验 + 数据
            std::vector<uint8_t> buff_msg(frame_total_len, 0);

            // 4. 构建帧头
            buff_msg[0] = FRAME_HEAD_0;
            buff_msg[1] = FRAME_HEAD_1;
            buff_msg[2] = frame_id;
            buff_msg[3] = length;

            // 5. 拷贝数据
            uint8_t* ps = static_cast<uint8_t*>(p);
            for (int q = 0; q < length; q++)
            {
                buff_msg[4 + q] = ps[q];
            }

            // 6. 异或校验（保留你的逻辑）
            buff_msg[4 + length] = calculateXORcheck(&buff_msg[4], length);
            buff_msg[5 + length] = FRAME_END_0;

            // 7. 串口发送（核心：异常捕获区）
            size_t write_num = serial_.write(buff_msg.data(), frame_total_len);
            return write_num;
        }
        // 8. 捕获所有标准异常（串口断开、硬件错误、内存异常等）
        catch (const std::exception& e)
        {
            // 打印错误日志，方便定位问题
            std::cerr << "串口发送异常：" << e.what() 
                    << "（可能是串口已断开/硬件故障）" << std::endl;
            init_serial(port_);
            return 0; // 异常时返回0，程序不崩溃
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            std::cerr << "串口发送：未知致命异常" << std::endl;
            init_serial(port_);
            return 0;
        }
    }


    bool Ten_serial::serial_read(void* p, uint8_t& received_frame_id, uint8_t& received_length) {
        std::lock_guard<std::mutex> lock(read_mtx_);
        if(this == nullptr || p == nullptr)
        {
            return false;
        }

        try
        {
            uint8_t byte;//声明临时变量 存储当前读取的字节
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
                    //usleep(100);
                    return true;
                }
            }
            return false;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            init_serial(port_);
            return false;
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            std::cerr << "串口接受：未知致命异常" << std::endl;
            init_serial(port_);
            return false;
        }   
    }


    /**
        @brief 增强版串口接收函数
        @details 检测到帧头后会**等待缓冲区数据**，完整解析一帧；无数据时阻塞等待（超时退出）
        @param p: 数据存储地址
        @param received_frame_id: 输出-帧ID
        @param received_length: 输出-数据长度
        @return bool 解析成功返回true，超时/异常返回false
    */ 
    bool Ten_serial::serial_read2(void* p, uint8_t& received_frame_id, uint8_t& received_length) 
    {
        std::lock_guard<std::mutex> lock(read_mtx_);
        if (this == nullptr || p == nullptr) {
            return false;
        }

        uint8_t byte;

        try {
            // 核心：把串口缓冲区里**所有可用字节**全部读完
            // 读完就退出，等下次调用继续读
            while (serial_.available() > 0) {
                // 读 1 字节
                if (serial_.read(&byte, 1) != 1) {
                    continue;
                }

                // ====================== 状态机：从上次停下的位置继续解析 ======================
                switch (read_state_) {
                    // 1. 等待帧头 1
                    case WAIT_HEAD_0:
                        if (byte == FRAME_HEAD_0) {
                            read_state_ = WAIT_HEAD_1; // 进入下一个状态
                        }
                        break;

                    // 2. 等待帧头 2
                    case WAIT_HEAD_1:
                        if (byte == FRAME_HEAD_1) {
                            read_state_ = WAIT_FRAME_ID;
                        } else {
                            read_state_ = WAIT_HEAD_0; // 不匹配，重新等帧头
                        }
                        break;

                    // 3. 读取帧ID
                    case WAIT_FRAME_ID:
                        received_frame_id = byte;
                        read_state_ = WAIT_LENGTH;
                        break;

                    // 4. 读取数据长度
                    case WAIT_LENGTH:
                        if (byte > 128) {
                            std::cout << "data Too long" << std::endl;
                            read_state_ = WAIT_HEAD_0; // 长度非法，重置
                            break;
                        }
                        target_length_ = byte;
                        received_length = byte;
                        rx_index_ = 0; // 清空缓存位置
                        read_state_ = WAIT_DATA;
                        break;

                    // 5. 读取数据（核心：累积续读，来一个存一个）
                    case WAIT_DATA:
                        rx_buffer_[rx_index_++] = byte;
                        // 存满长度 → 进入校验
                        if (rx_index_ >= target_length_) {
                            read_state_ = WAIT_CHECKSUM;
                        }
                        break;

                    // 6. 校验和
                    case WAIT_CHECKSUM:
                        if (calculateXORcheck(rx_buffer_, target_length_) == byte) {
                            read_state_ = WAIT_END;
                        } else {
                            read_state_ = WAIT_HEAD_0; // 校验错，重置
                        }
                        break;

                    // 7. 帧结束符 → 一帧完整接收！
                    case WAIT_END:
                        read_state_ = WAIT_HEAD_0; // 解析完成，恢复初始，准备下一帧

                        if (byte == FRAME_END_0) {
                            // 把完整数据拷贝给用户
                            memcpy(p, rx_buffer_, target_length_);
                            return true; // 成功收到一帧
                        }
                        break;

                    default:
                        read_state_ = WAIT_HEAD_0;
                        break;
                }
            }

            // 串口数据读完了，但还没收完一整帧
            // 状态 & 缓存全部保留！下次调用继续从这里解析
            return false;
        }
        catch (const std::exception& e) {
            std::cerr << e.what() << '\n';
            init_serial(port_);
            read_state_ = WAIT_HEAD_0; // 异常重置
            return false;
        }
        catch (...) {
            std::cerr << "串口接收：未知致命异常" << std::endl;
            init_serial(port_);
            read_state_ = WAIT_HEAD_0;
            return false;
        }
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
        // if(!ten_serial->serial_.isOpen())
        // {
        //     std::cout<<"ten_serial->serial_ is no open!"<<std::endl;
        //     exit(-1);
        // }
        return *ten_serial;
    }

    Ten_serial::Ten_serial(const std::string& port, const size_t& serial_baud)//禁止外部初始化实例
    {
        try
        {
            port_ = port;
            port_.pop_back();
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
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            while(!init_serial(port_) && Ten::_TREADPOOL_FLAG_.read_flag())
            {
                usleep(100000);
            }
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            std::cerr << "串口接受：未知致命异常" << std::endl;
            while(!init_serial(port_) && Ten::_TREADPOOL_FLAG_.read_flag())
            {
                usleep(100000);
            }
        }
        
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

    bool Ten_serial::init_serial(const std::string& port, const size_t& serial_baud)
    {
        static char number = '0';
        static int i = 0;
        try
        {
            serial_.close();
            // 循环检测串口状态，未打开则持续尝试初始化，直到打开成功
            if(!serial_.isOpen())
            {
                std::string ports = port + std::string(1, number + i);
                std::cout << ports << std::endl;
                // 配置串口设备路径：绑定传入的串口设备文件（如USB转串口设备路径）
                serial_.setPort(ports);
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
                std::cout << "serial_.open(): " << serial_.isOpen() << std::endl;
            }

            if(serial_.isOpen())
            {
                std::cout<<"serial open!: "<<set_low_latency()<<std::endl;
                return true;
            }
            std::cout<<"serial no open!"<<std::endl;
            return false;
        }
        catch(const std::exception& e)
        {
            std::cerr << e.what() << '\n';
            i++;
            if(i >= _max_serial_num_)
            {
                i = 0;
            }
            return false;
        }
        // 9. 捕获未知异常（终极兜底，绝对防止程序崩溃）
        catch (...)
        {
            std::cerr << "串口接受：未知致命异常" << std::endl;
            i++;
            if(i >= _max_serial_num_)
            {
                i = 0;
            }
            return false;
        }
        
    }



    bool Ten_serial::set_low_latency() 
    {
        if (!serial_.isOpen()) return false;

        // 直接从 serial 库获取当前端口名，最准确，不受类内 port_ 变量修改影响
        std::string current_port = serial_.getPort();

        // 临时打开设备，仅用于配置 ioctl，不影响已有的串口连接
        int fd = open(current_port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd < 0) {
            std::cerr << "[low_latency] 打开设备失败: " 
                    << current_port << " 错误: " << strerror(errno) << std::endl;
            return false;
        }

        struct serial_struct ser_info;
        if (ioctl(fd, TIOCGSERIAL, &ser_info) < 0) {
            std::cerr << "[low_latency] 获取串口信息失败: " << strerror(errno) << std::endl;
            close(fd);
            return false;
        }

        ser_info.flags |= ASYNC_LOW_LATENCY;
        if (ioctl(fd, TIOCSSERIAL, &ser_info) < 0) {
            std::cerr << "[low_latency] 设置失败: " << strerror(errno) << std::endl;
            close(fd);
            return false;
        }

        close(fd); // 设置完成，关闭临时文件描述符
        return true;
    }



}



#endif

