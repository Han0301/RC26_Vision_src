#ifndef __LIDAR_H_
#define __LIDAR_H_

#include<ros/ros.h>
// #include <atomic>
// #include <iostream>
// #include <mutex>
// 引入标准输入输出库，提供printf/snprintf等打印函数，用于输出线程读写日志
#include <stdio.h>
// 引入标准库，提供malloc/free（内存分配/释放）、usleep（微秒级休眠）等函数，支撑RCU的内存操作和耗时模拟
#include <stdlib.h>
// 引入POSIX线程库，提供pthread_create（创建线程）、pthread_join（等待线程）等函数，实现多线程读写
#include <pthread.h>
// 引入Userspace RCU的memb模块（memory barrier），是urcu中性能最优的模块，仅依赖内存屏障，无额外锁开销
// 核心提供rcu_read_lock/rcu_read_unlock（读临界区）、rcu_dereference/rcu_assign_pointer（指针安全操作）、synchronize_rcu（宽限期等待）
#include <urcu/urcu-memb.h> 
#include <nav_msgs/Odometry.h>
#include <livox_ros_driver/CustomMsg.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>

namespace Ten
{

// class T
// {
// public:
//     int test = 0;
// };

// template<typename T>
// class Ten_Node
// {
// public:
//     // Ten_Node(const Ten_Node& node) = delete;
//     // Ten_Node& operator=(const Ten_Node& node) = delete;
//     Ten_Node()
//     :next_(nullptr)
//     {

//     }

//     Ten_Node(const T& msg)
//     :data_(msg),
//     next_(nullptr)
//     {

//     }

//     Ten_Node(const Ten_Node& node)
//     :data_(node.data_),
//     next_(node.next_)
//     {

//     }

//     Ten_Node& operator=(const Ten_Node& node)
//     {
//         data_ = node.data_;
//         next_ = node.next_;
//         return *this;
//     }

//     ~Ten_Node()
//     {

//     }


// //private:
// T data_;
// Ten_Node* next_;
// };



// //template<typename T>
// class Ten_lock_list
// {
// public:
//     Ten_lock_list(Ten_lock_list& list) = delete;
//     Ten_lock_list& operator=(Ten_lock_list& list) = delete;
//     Ten_lock_list(size_t max_size = 3)
//     :head_(nullptr),
//     tail_(nullptr),
//     size_(0),
//     max_size_(max_size >= 3 ? max_size : 3)
//     {
//         Ten_Node<T>* head = new Ten_Node<T>();
//         Ten_Node<T>* tail = new Ten_Node<T>();
//         head->next_ = tail;
//         head_.store(head);
//         tail_.store(tail);
//         size_ = 2;
//     }
    
//     T Get_data()
//     {
//         // if(size_ < 2)
//         // {
//         //     return T();
//         // }
//         std::lock_guard<std::mutex> lock(mtx_get_);
//         if(size_.load() == 2)
//         {
//             return head_.load()->data_;
//         }
//         else
//         {
//             Ten_Node<T>* tmp = head_.load();
//             T data = tmp->data_;
//             PopFront();
//             return data;
//         }
//     }

//     void Push_data(const T& msg)
//     {
//         if(size_.load() > max_size_)
//         {
//             //PopFront();
//             Get_data();
//         }
//         PushBack(msg);
//     }

//     ~Ten_lock_list()
//     {
//         Ten_Node<T>* cur = head_.load();
//         while(cur != nullptr)
//         {
//             Ten_Node<T>* tmp = cur->next_;
//             delete cur;
//             cur = tmp;
//         }
//     }
// private:
//     void PopFront()
//     {
//         if(size_.load() <= 2)
//         {
//             return;
//         }
//         Ten_Node<T>* head = head_.load();
//         Ten_Node<T>* next = head->next_;
//         head_.store(next);
//         delete head;
//         //size_--;
//         size_.fetch_sub(1); // 修正：原子减操作，避免计数错误
//     }

//     void PushBack(const T& msg)
//     {
//         std::lock_guard<std::mutex> lock(mtx_push_);
//         Ten_Node<T>* tmp = new Ten_Node<T>(msg);
//         Ten_Node<T>* tail = tail_.load();
//         tail->next_ = tmp;
//         tail_.store(tmp);
//         //size_++;
//         size_.fetch_add(1);
//     }

// std::atomic<Ten_Node<T>*> head_;
// std::atomic<Ten_Node<T>*> tail_;
// std::mutex mtx_get_;
// std::mutex mtx_push_;
// std::atomic<size_t> size_;
// const size_t max_size_; 
// };


// class T
// {
// public:
//     int test = 0;
// };


template<typename T>
class Ten_one_write_multiple_read
{
public:
    Ten_one_write_multiple_read(const Ten_one_write_multiple_read& wr) = delete;
    Ten_one_write_multiple_read& operator=(const Ten_one_write_multiple_read& wr) = delete;
    /**
        @brief 初始化函数
    */
    Ten_one_write_multiple_read()
    :data_(static_cast<T*>(malloc(sizeof(T))))
    {
        // 初始化URCU的memb模块：必须在创建线程前执行，初始化RCU核心机制
        static bool rcu_inited = false;
        if (!rcu_inited) {
            urcu_memb_init();
            rcu_inited = true;
        }
        // 分配初始数据内存（默认构造T类型对象）
        //data_ = static_cast<T*>(malloc(sizeof(T)));
        if (data_) {
            // 调用T的默认构造函数（针对非POD类型）
            new (data_) T();
        }
        // 注册当前线程（如果构造函数在读写线程中调用，需确保RCU线程注册）
        //urcu_memb_register_thread();        
    }

    // 析构函数：清理RCU资源和数据内存
    ~Ten_one_write_multiple_read()
    {
        // 等待RCU宽限期结束，确保所有读线程已退出临界区
        urcu_memb_synchronize_rcu();
        
        // 释放数据内存（调用T的析构函数）
        if (data_) {
            data_->~T();
            free(data_);
            data_ = nullptr;
        }

        // 注销RCU线程
        //urcu_memb_unregister_thread();
    }


    /**
        @brief 写数据
        @param new_data: 写入对象
    */
    // 写数据：RCU写拷贝+原子替换+宽限期回收
    void write_data(const T& new_data)
    {
        // 1. 读旧数据（RCU安全读取）
        T* old_data = rcu_dereference(data_);
        
        // 2. 分配新内存并拷贝数据（写拷贝核心）
        T* new_data_ptr = static_cast<T*>(malloc(sizeof(T)));
        if (!new_data_ptr) {
            perror("malloc failed in write_data");
            return;
        }

        // 拷贝新数据（调用T的拷贝构造/赋值）
        new (new_data_ptr) T(new_data);

        // 3. 原子替换全局指针（RCU写操作核心）
        rcu_assign_pointer(data_, new_data_ptr);

        // 4. 等待RCU宽限期结束，确保旧数据不再被读线程引用
        urcu_memb_synchronize_rcu();

        // 5. 释放旧数据内存
        if (old_data) {
            old_data->~T(); // 调用析构函数
            free(old_data);
        }
    }
    /** 
        @brief 读取数据
        @return T: 返回对象的拷贝
    */
    // 读数据：RCU无锁读（线程安全）
    T read_data()
    {
        // 1. 进入RCU读临界区（无锁，仅内存屏障）
        urcu_memb_read_lock();
        // 2. 安全读取RCU保护的指针
        T* data = rcu_dereference(data_);
        T ret;
        if (data) {
            ret = *data; // 拷贝数据（线程安全，旧数据不会被修改）
        }
        // 3. 退出RCU读临界区
        urcu_memb_read_unlock();
        return ret;
    }

private:
T* data_;

};

extern Ten_one_write_multiple_read<nav_msgs::Odometry> _TF_GET_;
extern Ten_one_write_multiple_read<nav_msgs::Odometry> _TF_CAMERA_GET_;
// extern Ten_one_write_multiple_read<livox_ros_driver::CustomMsg> _LIVOX_GET_;

// extern Ten_one_write_multiple_read<sensor_msgs::Imu> _IMU_GET_;

extern Ten_one_write_multiple_read<sensor_msgs::PointCloud2> _Map_GET_;

}

#endif


