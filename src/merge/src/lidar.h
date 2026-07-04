#ifndef __LIDAR_H_
#define __LIDAR_H_

#include<ros/ros.h>
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <atomic>
#include <vector>
#include <mutex>
#include <memory>  // 新增：智能指针头文件
#include <urcu/urcu-memb.h> 
#include <nav_msgs/Odometry.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

namespace Ten
{

template<typename T>
class Ten_one_write_multiple_read
{
public:
    // 保留你的删除拷贝逻辑（不动原有代码）
    Ten_one_write_multiple_read(const Ten_one_write_multiple_read& wr) = delete;
    Ten_one_write_multiple_read& operator=(const Ten_one_write_multiple_read& wr) = delete;

    Ten_one_write_multiple_read()
    :data_(static_cast<T*>(malloc(sizeof(T))))
    {
        static bool rcu_inited = false;
        if (!rcu_inited) {
            urcu_memb_init();
            rcu_inited = true;
        }
        if (data_) {
            new (data_) T();
        }
    }

    ~Ten_one_write_multiple_read()
    {
        urcu_memb_synchronize_rcu();
        if (data_) {
            data_->~T();
            free(data_);
            data_ = nullptr;
        }
    }

    void write_data(const T& new_data)
    {
        T* old_data = rcu_dereference(data_);
        T* new_data_ptr = static_cast<T*>(malloc(sizeof(T)));
        if (!new_data_ptr) {
            perror("malloc failed in write_data");
            return;
        }
        new (new_data_ptr) T(new_data);
        rcu_assign_pointer(data_, new_data_ptr);
        urcu_memb_synchronize_rcu();
        if (old_data) {
            old_data->~T();
            free(old_data);
        }
    }

    T read_data()
    {
        urcu_memb_read_lock();
        T* data = rcu_dereference(data_);
        T ret;
        if (data) {
            ret = *data;
        }
        urcu_memb_read_unlock();
        return ret;
    }

private:
    T* data_;
};

// ===================== 修复版 RCUQueue：用智能指针解决vector不可拷贝问题 =====================
template<typename T>
class RCUQueue
{
public:
    explicit RCUQueue(size_t queue_size = 10)
    : max_size_(queue_size)
    {
        // 核心修复：vector存储智能指针，循环初始化对象
        storage_.reserve(max_size_);
        for (size_t i = 0; i < max_size_; ++i) {
            storage_.emplace_back(std::make_unique<Ten_one_write_multiple_read<T>>());
        }
        head_ = 0;
        tail_ = 0;
        count_ = 0;
    }

    // 写入接口完全不变
    void push(const T& msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);
        
        if (count_.load() >= max_size_) {
            head_ = (head_.load() + 1) % max_size_;
            count_--;
        }

        // 指针访问 -> 不变更外部接口
        storage_[tail_.load()]->write_data(msg);
        tail_ = (tail_.load() + 1) % max_size_;
        count_++;
    }

    // 读取接口完全不变
    bool pop(T& msg)
    {
        if (count_.load() == 0) {
            return false;
        }

        msg = storage_[head_.load()]->read_data();
        head_ = (head_.load() + 1) % max_size_;
        count_--;
        
        return true;
    }

    // 获取最新数据接口完全不变
    bool get_latest(T& msg)
    {
        if (count_.load() == 0) {
            return false;
        }
        size_t latest_idx = (tail_.load() - 1 + max_size_) % max_size_;
        msg = storage_[latest_idx]->read_data();
        return true;
    }

private:
    // 核心修复：vector存储 智能指针（可拷贝，兼容STL）
    std::vector<std::unique_ptr<Ten_one_write_multiple_read<T>>> storage_;
    size_t max_size_;

    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
    std::atomic<size_t> count_;
    std::mutex mtx_;
};

// 全局变量声明完全不变
extern RCUQueue<nav_msgs::Odometry> _TF_GET_;
extern RCUQueue<nav_msgs::Odometry> _TF_GET2_;
extern RCUQueue<livox_ros_driver::CustomMsg> _LIVOX_GET_;
extern RCUQueue<sensor_msgs::PointCloud2> _Map_GET_;
extern RCUQueue<sensor_msgs::Imu> _IMU_GET_;

}

#endif
