#ifndef __PREDICTED_H_
#define __PREDICTED_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <queue>
#include <cmath>
#include <vector>
#include "./../method_math.h"


namespace Ten
{

class ImuOdometry
{
public:
    /**
     * @brief 构造函数
     */
    ImuOdometry(size_t max_queue = 20, 
                const XYZ& gravity = XYZ{0.0, 0.0,  -9.81215},
                double ang_deadzone = 0.02,
                double acc_deadzone = 0.01,
                double vel_deadzone = 0.01)  // 【新增】初始速度死区默认值
        : max_queue_size_(max_queue),
          gravity_world_(gravity),
          angular_vel_deadzone_(ang_deadzone),
          linear_acc_deadzone_(acc_deadzone),
          velocity_deadzone_(vel_deadzone)  // 【新增】初始化速度死区
    {
        // 初始化实时旋转矩阵
        current_rot_.assign(3, std::vector<double>(3, 0.0));
    }

    // ==================== 新增：独立插入IMU数据函数 ====================
    void addImuData(const sensor_msgs::Imu& imu_msg)
    {
        cacheImuData(imu_msg);
    }

    // ==================== 修改后：processImu 移除 imu_msg ====================
    /**
     * @param initial_vel 小段积分初始速度
     * @param base_rpy IMU坐标系相对于Z轴朝上世界系的固定轴旋转
     * @return 小段相对位姿
     */
    XYZRPY processImu(const XYZ& initial_vel, const RPY& base_rpy)
    {
        // 直接积分缓存的IMU数据
        return integrateWindow(initial_vel, base_rpy);
    }

    // 重置队列（清空所有IMU数据）
    void resetQueue()
    {
        std::queue<sensor_msgs::Imu> empty_q;
        std::swap(imu_queue_, empty_q);
    }

    // 参数配置接口
    void setMaxQueueSize(size_t size) { max_queue_size_ = size; }
    void setWorldGravity(const XYZ& gravity) { gravity_world_ = gravity; }
    void setAngularVelDeadzone(double threshold) { angular_vel_deadzone_ = std::fabs(threshold); }
    void setLinearAccDeadzone(double threshold) { linear_acc_deadzone_ = std::fabs(threshold); }
    void setVelocityDeadzone(double threshold) { velocity_deadzone_ = std::fabs(threshold); }  // 【新增】速度死区配置接口

private:
    // 缓存IMU数据（私有，外部通过addImuData调用）
    void cacheImuData(const sensor_msgs::Imu& imu_msg)
    {
        imu_queue_.push(imu_msg);
        // 保持队列最大长度
        while (imu_queue_.size() > max_queue_size_) {
            imu_queue_.pop();
        }
    }

    /**
     * @brief 核心积分：动态更新旋转矩阵
     * @param base_rpy 外部传入：IMU→世界系 基础固定轴RPY
     */
    XYZRPY integrateWindow(const XYZ& initial_vel, const RPY& base_rpy)
    {
        XYZ rel_pos;        // 输出：相对位置
        RPY rel_rpy;        // 输出：相对姿态
        
        // ==================== 【核心新增】初始速度死区处理 ====================
        XYZ filtered_vel = initial_vel;
        if (std::fabs(filtered_vel._x) < velocity_deadzone_) filtered_vel._x = 0.0;
        if (std::fabs(filtered_vel._y) < velocity_deadzone_) filtered_vel._y = 0.0;
        if (std::fabs(filtered_vel._z) < velocity_deadzone_) filtered_vel._z = 0.0;
        
        XYZ rel_vel = filtered_vel;  // 使用死区过滤后的初始速度

        double last_ts = 0.0;

        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;
        while (!temp_q.empty())
        {
            const auto imu = temp_q.front();
            temp_q.pop();

            const double curr_ts = imu.header.stamp.toSec();
            if (last_ts == 0.0) {
                last_ts = curr_ts;
                continue;
            }
            const double dt = curr_ts - last_ts;

            // ==================== 关键：实时计算总姿态旋转矩阵 ====================
            // 总姿态 = 外部基础RPY + 积分的相对RPY
            const double total_roll  = base_rpy._roll  + rel_rpy._roll;
            const double total_pitch = base_rpy._pitch + rel_rpy._pitch;
            const double total_yaw   = base_rpy._yaw   + rel_rpy._yaw;
            // 每帧重新计算变换矩阵
            computeRotationMatrix(total_roll, total_pitch, total_yaw, current_rot_);

            // Step1：用最新旋转矩阵变换IMU数据到世界坐标系
            XYZ angular_vel, linear_acc;
            transformImuToWorld(imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z, angular_vel);
            transformImuToWorld(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, linear_acc);

            // 角速度死区
            if (std::fabs(angular_vel._x) < angular_vel_deadzone_) angular_vel._x = 0.0;
            if (std::fabs(angular_vel._y) < angular_vel_deadzone_) angular_vel._y = 0.0;
            if (std::fabs(angular_vel._z) < angular_vel_deadzone_) angular_vel._z = 0.0;

            // Step2：积分相对姿态
            rel_rpy._roll  += angular_vel._x * dt;
            rel_rpy._pitch += angular_vel._y * dt;
            rel_rpy._yaw   += angular_vel._z * dt;

            // Step3：去除重力加速度
            linear_acc._x -= gravity_world_._x;
            linear_acc._y -= gravity_world_._y;
            linear_acc._z -= gravity_world_._z;

            // 线加速度死区
            if (std::fabs(linear_acc._x) < linear_acc_deadzone_) linear_acc._x = 0.0;
            if (std::fabs(linear_acc._y) < linear_acc_deadzone_) linear_acc._y = 0.0;
            if (std::fabs(linear_acc._z) < linear_acc_deadzone_) linear_acc._z = 0.0;

            // Step4：速度+位置积分
            rel_vel._x += linear_acc._x * dt;
            rel_vel._y += linear_acc._y * dt;
            rel_vel._z += linear_acc._z * dt;

            rel_pos._x += rel_vel._x * dt;
            rel_pos._y += rel_vel._y * dt;
            rel_pos._z += rel_vel._z * dt;

            last_ts = curr_ts;
        }

        // 输出小段相对位姿
        XYZRPY relative_pose;
        relative_pose._xyz = rel_pos;
        relative_pose._rpy = rel_rpy;
        return relative_pose;
    }

    // 旋转矩阵计算（固定轴）
    void computeRotationMatrix(double roll, double pitch, double yaw, std::vector<std::vector<double>>& rot)
    {
        const double cr = cos(roll), sr = sin(roll);
        const double cp = cos(pitch), sp = sin(pitch);
        const double cy = cos(yaw), sy = sin(yaw);

        rot[0][0] = cp*cy;   rot[0][1] = sr*sp*cy - cr*sy; rot[0][2] = cr*sp*cy + sr*sy;
        rot[1][0] = cp*sy;   rot[1][1] = sr*sp*sy + cr*cy; rot[1][2] = cr*sp*sy - sr*cy;
        rot[2][0] = -sp;     rot[2][1] = sr*cp;            rot[2][2] = cr*cp;
    }

    // 坐标变换：IMU系 → 世界系（实时旋转矩阵）
    void transformImuToWorld(double x, double y, double z, XYZ& out)
    {
        out._x = current_rot_[0][0]*x + current_rot_[0][1]*y + current_rot_[0][2]*z;
        out._y = current_rot_[1][0]*x + current_rot_[1][1]*y + current_rot_[1][2]*z;
        out._z = current_rot_[2][0]*x + current_rot_[2][1]*y + current_rot_[2][2]*z;
    }

private:
    // IMU数据缓存队列
    std::queue<sensor_msgs::Imu> imu_queue_;
    size_t max_queue_size_;

    // 重力参数
    XYZ gravity_world_;

    // 死区参数
    double angular_vel_deadzone_;
    double linear_acc_deadzone_;
    double velocity_deadzone_;  // 【新增】初始速度死区成员变量

    // 实时旋转矩阵（每帧更新）
    std::vector<std::vector<double>> current_rot_;
};

}


#endif
