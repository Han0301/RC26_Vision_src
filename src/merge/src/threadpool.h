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



extern Ten::ThreadPool_flag _TREADPOOL_FLAG_;

}










#endif


