#ifndef __SERIAL_H_
#define __SERIAL_H_
#include <ros/ros.h>
#include <iostream>
#include <serial/serial.h>
#include <string>
#include <mutex>
#include <unistd.h>
namespace Ten
{

//全局只有一个对象
class Ten_serial
{
#define FRAME_HEAD_0 0xAA //帧头字节1
#define FRAME_HEAD_1 0x55  // 帧头字节2
#define FRAME_END_0 0xEE // 帧尾字节1
public:
    //禁用拷贝构造
    Ten_serial(const Ten_serial& serial) = delete;
    //禁用赋值
    Ten_serial& operator=(const Ten_serial& serial) = delete;
    /**
        @brief 设置端口和波特率
        @param port: 串口路径 "/dev/ttyUSB0" , "/dev/ttyACM0" ...
        @param serial_bund: 波特率
        @return Ten_serial& 返回Ten_serial实例
    */
    static Ten_serial& GetInstance(const std::string& port = "/dev/ttyACM0", const size_t& serial_baud = 115200);
    /**
        @brief 发送数据
        @param p: 数据段（一维数组首元数的地址）
        @param frame_id: id号
        @param length: 数据长度（sizeof(type)*数组大小）
        @return size_t 写入数据长度
    */
    size_t serial_send(void* p, uint8_t frame_id, uint8_t length);
    /**
        @brief 串口数据接收函数
        @param p: 数据段（一维数组首元数的地址）（返回值）
        @param received_frame_id: id(返回值)
        @param received_length: 数据长度（sizeof(type)*数组大小）（返回值）
        @return bool 是否读取成功
    */
    bool serial_read(void* p, uint8_t& received_frame_id, uint8_t& received_length);
    /*
      @brief 检查串口是否打开
      @return bool 
    */
    bool isOpen() const{
        return serial_.isOpen();
    }


    /**
     * @brief 清空串口缓冲区（接收+发送），清除残留的脏数据
     * @param clear_type: 清空类型（0=全清，1=仅接收，2=仅发送）
     * @return bool 清空是否成功（串口未打开/空指针返回false）
     */
    bool clearBuffer(int clear_type = 0);

    ~Ten_serial()
    {
        std::lock_guard<std::mutex> lock_s(send_mtx_);
        std::lock_guard<std::mutex> lock_r(read_mtx_);
        if (serial_.isOpen())serial_.close();
    }


private:
    Ten_serial(const std::string& port, const size_t& serial_baud);//禁止外部初始化实例

    static std::unique_ptr<Ten_serial> create(const std::string& port = "/dev/ttyACM0", const size_t& serial_baud = 9600) {
        // 静态函数可访问私有构造函数，直接new对象后封装为unique_ptr
        return std::unique_ptr<Ten_serial>(new Ten_serial(port, serial_baud));
    }

    //异或校验
    int calculateXORcheck(const uint8_t* data, size_t length);

    
serial::Serial serial_;
std::mutex send_mtx_;
std::mutex read_mtx_;
static std::once_flag serial_flag_;
};

}
#endif

