#ifndef __THREADPOOL_H_
#define __THREADPOOL_H_

#include <iostream>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <functional>
#include <future>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#include <cstring>
#include <cerrno>
#include <sys/resource.h>

namespace Ten
{

class ThreadPool
{
public:
    /**
        @brief 初始化线程函数
        @param ThreadNum： 线程数量
        @param task_max： 最多任务数量
    */
    explicit ThreadPool(size_t ThreadNum, size_t task_max = 10)
    :_stop(false),
    _tasks_max(task_max)
    {
        for (size_t i = 0; i < ThreadNum; i++)
        {
            _threads.emplace_back([this] {
                while (true)
                {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(_mtx);
                        _condition.wait(lock, [this] {return !_tasks.empty() || _stop; });
                        if (_stop && _tasks.empty())
                        {
                            return;
                        }
                        task = std::move(_tasks.front());
                        _tasks.pop();
                    }
                    task();
                }

                });
        }
    }

    ~ThreadPool()
    {
        {
            std::unique_lock<std::mutex> lock(_mtx);
            _stop = true;
        }
        _condition.notify_all();
        for (auto& t : _threads)
        {
            t.join();
        }
    }
    /**
        @brief 添加要执行的函数
        @param F： 函数
        @param Args： 传递给函数的参数
    */
    template<typename F, typename... Args>
    void enqueue(F&& f, Args&&... args)
    {
        std::function<void()> task = std::bind(std::forward<F>(f), std::forward<Args>(args)...);
        {
            std::unique_lock<std::mutex> lock(_mtx);
            if(_tasks.size() < _tasks_max) _tasks.emplace(std::move(task));
            else
            {
                std::cout<< "function full can't increase!"<<std::endl;
                return;
            }
        }
        _condition.notify_one();
    }





private:
    std::vector<std::thread> _threads;
    std::queue<std::function<void()>> _tasks;
    size_t _tasks_max;

    std::mutex _mtx;
    std::condition_variable _condition;

    bool _stop;


};

class ThreadPool_flag
{
public:

    ThreadPool_flag()
    :flag_(true)
    {

    }
    /**
        @brief 设置flag标志位
        @param flag：true运行，false终止
    */
    void set_flag(bool flag)
    {
        flag_.store(flag);
    }

   /**
       @brief 读取flag标志位
       @return bool：true运行，false终止
   */
    bool read_flag()
    {
        return flag_.load();
    }
private:
    //兼容ros
    std::atomic<bool> flag_;

};

/**
 * @brief 将当前调用该函数的线程设置为高优先级（分配更多CPU资源）
 * @param realtime_priority 实时调度优先级（1-99，建议10-50，默认50）
 * @param nice_value 普通调度的nice值（-20~19，越小优先级越高，默认-10）
 * @return int 0=成功（实时调度）, 1=降级为nice值调整成功, -1=所有尝试失败
 */
int set_thread_as_important(int realtime_priority = 10, int nice_value = -10);
/**
 * @brief 绑定当前线程到指定CPU核心（解决CPU密集型线程核心漂移/缓存失效问题）
 * @param cpu_core 目标CPU核心编号（从0开始，如0/1/2...，需小于系统总核心数）
 * @return bool true=绑定成功，false=绑定失败（核心非法/权限不足）
 */
bool bind_thread_to_cpu(int cpu_core);


// /**
//  * @brief 定时函数：无参数
//  * @return 第一次调用返回0.0，后续返回【本次-上次】的时间差（单位：秒）
//  */
// double timer();

class Timetester
{
public:
    /**
     * @brief 定时函数：无参数
     * @return 第一次调用返回0.0，后续返回【本次-上次】的时间差（单位：秒）
     */
    double timer() 
    {
        // 获取当前系统时间（稳定时钟，不受系统时间修改影响）
        auto current_time = std::chrono::steady_clock::now();
        // 核心逻辑：第一次调用
        if (is_first) {
            is_first = false;
            last_time = current_time; // 初始化基准时间
            return 0.0;               // 第一次固定返回0
        }

        // 核心逻辑：非第一次调用 → 计算时间差（秒）
        std::chrono::duration<double> interval = current_time - last_time;
        last_time = current_time; // 更新上一次时间为当前时间
        return interval.count();  // 返回秒数（支持小数精度）
    }
private:
   std::chrono::steady_clock::time_point last_time;
   bool is_first = true; 
};

extern Ten::ThreadPool_flag _TREADPOOL_FLAG_;
extern Ten::ThreadPool_flag _LASERMAPPING_FLAG_;
extern Ten::ThreadPool_flag _PUB_CLOUD_FLAG_;
extern Ten::ThreadPool_flag _CAMERA_KFS_FLAG_;


}










#endif


