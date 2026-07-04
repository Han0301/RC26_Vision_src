#ifndef __THREADPOOL_CPP_
#define __THREADPOOL_CPP_
#include "threadpool.h"


namespace Ten
{

    Ten::ThreadPool_flag _TREADPOOL_FLAG_;
    Ten::ThreadPool_flag _LASERMAPPING_FLAG_;
    Ten::ThreadPool_flag _PUB_CLOUD_FLAG_;
    Ten::ThreadPool_flag _PUB_CLOUD_FLAG2_;
    Ten::ThreadPool_flag _CAMERA_KFS_FLAG_;
    Ten::ThreadPool_flag _APRILTAG_FLAG_;
    Ten::ThreadPool_flag _MAP_FLAG_;
    Ten::ThreadPool_flag _PARAMETER_FLAG_;
    Ten::ThreadPool_flag _KFS_DECTECTOR_FLAG_;
    Ten::ThreadPool_flag _POINT_LIO_RUN_FLAG_;
    Ten::ThreadPool_flag _POINT_LIO_CHANGE_FLAG_;
    Ten::ThreadPool_flag _FAST_LIO_MAPING_FLAG_;
    Ten::ThreadPool_flag _CAMERA_CLOSE_FLAG_;

    
    /**
     * @brief 将当前调用该函数的线程设置为高优先级（分配更多CPU资源）
     * @param realtime_priority 实时调度优先级（1-99，建议10-50，默认50）
     * @param nice_value 普通调度的nice值（-20~19，越小优先级越高，默认-10）
     * @return int 0=成功（实时调度）, 1=降级为nice值调整成功, -1=所有尝试失败
     */
    int set_thread_as_important(int realtime_priority, int nice_value) {
        // 1. 校验参数合法性
        if (realtime_priority < 1 || realtime_priority > 99) {
            std::cerr << "[错误] 实时优先级必须在1-99之间，已自动修正为50" << std::endl;
            realtime_priority = 50;
        }
        if (nice_value < -20 || nice_value > 19) {
            std::cerr << "[错误] nice值必须在-20~19之间，已自动修正为-10" << std::endl;
            nice_value = -10;
        }

        // 2. 获取当前线程ID
        pthread_t tid = pthread_self();

        // 3. 优先尝试设置实时调度策略（SCHED_FIFO，最高优先级）
        struct sched_param param;
        std::memset(&param, 0, sizeof(param)); // 改用std::memset，避免命名空间问题
        param.sched_priority = realtime_priority;

        int ret = pthread_setschedparam(tid, SCHED_FIFO, &param);
        if (ret == 0) {
            // 验证设置结果并输出
            int policy;
            pthread_getschedparam(tid, &policy, &param);
            std::cout << "[成功] 当前线程已设为实时调度(SCHED_FIFO)，优先级：" 
                    << param.sched_priority << std::endl;
            return 0; // 实时调度设置成功
        }

        // 4. 实时调度失败，降级调整nice值（普通调度的优先级）
        std::cerr << "[提示] 实时调度设置失败(" << std::strerror(ret) 
                << ")，尝试调整nice值..." << std::endl;
        
        if (setpriority(PRIO_PROCESS, 0, nice_value) == 0) {
            std::cout << "[成功] 当前线程nice值已设为：" << nice_value 
                    << "（普通调度高优先级）" << std::endl;
            return 1; // nice值调整成功
        }

        // 5. 所有尝试失败
        std::cerr << "[失败] 调整nice值也失败：" << std::strerror(errno) << std::endl;
        std::cerr << "  可能原因：1. 未使用sudo运行 2. 系统权限限制" << std::endl;
        return -1; // 全部失败
    }


    /**
     * @brief 绑定当前线程到指定CPU核心（解决CPU密集型线程核心漂移/缓存失效问题）
     * @param cpu_core 目标CPU核心编号（从0开始，如0/1/2...，需小于系统总核心数）
     * @return bool true=绑定成功，false=绑定失败（核心非法/权限不足）
     */
    bool bind_thread_to_cpu(int cpu_core) 
    {
        // 先检查核心是否合法（替换为你的CPU核心数，比如8核则0-7）
        int cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
        if (cpu_core < 0 || cpu_core >= cpu_count) {
            std::cerr << "[错误] CPU核心" << cpu_core << "非法，当前CPU核心数：" << cpu_count << std::endl;
            return false;
        }

        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_core, &cpuset); // 绑定到指定核心
        pthread_t tid = pthread_self();
        int ret = pthread_setaffinity_np(tid, sizeof(cpu_set_t), &cpuset);
        if (ret != 0) {
            std::cerr << "[错误] 绑定CPU核心失败：" << std::strerror(ret) << std::endl;
            return false;
        }

        std::cout << "[成功] CPU密集线程已绑定到核心：" << cpu_core << std::endl;
        return true;
    }

    // /**
    //  * @brief 定时函数：无参数
    //  * @return 第一次调用返回0.0，后续返回【本次-上次】的时间差（单位：秒）
    //  */
    // double timer() 
    // {
    //     // 静态变量：保存【上一次调用】的时间点（程序运行中永久生效）
    //     static std::chrono::steady_clock::time_point last_time;
    //     // 静态标记：判断是否为【第一次调用】
    //     static bool is_first = true;

    //     // 获取当前系统时间（稳定时钟，不受系统时间修改影响）
    //     auto current_time = std::chrono::steady_clock::now();

    //     // 核心逻辑：第一次调用
    //     if (is_first) {
    //         is_first = false;
    //         last_time = current_time; // 初始化基准时间
    //         return 0.0;               // 第一次固定返回0
    //     }

    //     // 核心逻辑：非第一次调用 → 计算时间差（秒）
    //     std::chrono::duration<double> interval = current_time - last_time;
    //     last_time = current_time; // 更新上一次时间为当前时间
    //     return interval.count();  // 返回秒数（支持小数精度）
    // }


}




#endif



