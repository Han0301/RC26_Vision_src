#ifndef __PREDICTED_H_
#define __PREDICTED_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <queue>
#include <cmath>
#include <vector>
#include <algorithm>
#include "./../method_math.h"

namespace Ten {
// 全局常量：单位转换系数（g → m/s²）
constexpr double G_TO_MPS2 = 9.81;
// 自适应阈值：加速度变化率超过此值判定为剧烈运动
constexpr double DYNAMIC_THRESHOLD = 2.0; // m/s³（增大，避免过度平滑）
// 最大平滑系数：慢运动时的最大平滑程度
constexpr double MAX_SMOOTH_ALPHA = 0.15; // 减小，提高响应速度
// 最小阻尼系数：剧烈运动时的最小阻尼
constexpr double MIN_DAMPING = 0.99; // 增大，减少漂移
// 静止判定阈值：加速度方差小于此值判定为静止
constexpr double STATIC_ACC_VAR_THRESHOLD = 0.005; // g²

struct Quaternion {
    double w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
    // ✅ 修复：补回四元数逆函数（解决编译报错）
    Quaternion inverse() const {
        return Quaternion(w, -x, -y, -z);
    }
};

// 四元数乘法（标准）
inline Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {
    Quaternion res;
    res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    res.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    res.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    res.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return res;
}

// 四元数归一化（标准）
inline Quaternion normalize(const Quaternion& q) {
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if(norm < 1e-8) return Quaternion(1,0,0,0);
    return Quaternion(q.w/norm, q.x/norm, q.y/norm, q.z/norm);
}

// 四元数指数映射（精确姿态更新）
inline Quaternion quatExponentialMap(const XYZ& omega, double dt) {
    double theta = sqrt(omega._x*omega._x + omega._y*omega._y + omega._z*omega._z) * dt;
    if (theta < 1e-6) {
        return Quaternion(1, 0.5*omega._x*dt, 0.5*omega._y*dt, 0.5*omega._z*dt);
    }
    double half_theta = theta * 0.5;
    double sin_half = sin(half_theta) / theta;
    return Quaternion(
        cos(half_theta),
        omega._x * sin_half * dt,
        omega._y * sin_half * dt,
        omega._z * sin_half * dt
    );
}

// 用四元数旋转向量（优化实现）
inline XYZ rotateVectorByQuat(const XYZ& v, const Quaternion& q) {
    double qv_x = q.y*v._z - q.z*v._y;
    double qv_y = q.z*v._x - q.x*v._z;
    double qv_z = q.x*v._y - q.y*v._x;
    
    XYZ res;
    res._x = v._x + 2 * (q.w*qv_x + q.y*qv_z - q.z*qv_y);
    res._y = v._y + 2 * (q.w*qv_y + q.z*qv_x - q.x*qv_z);
    res._z = v._z + 2 * (q.w*qv_z + q.x*qv_y - q.y*qv_x);
    return res;
}

// 四元数转RPY（标准Z-Y-X顺序）
inline RPY quatToRpy(const Quaternion& q) {
    RPY rpy;
    double w = q.w, x = q.x, y = q.y, z = q.z;
    rpy._roll  = atan2(2*(w*x + y*z), 1-2*(x*x + y*y));
    double sin_pitch = -2*(x*z - w*y);
    sin_pitch = std::max(-1.0, std::min(sin_pitch, 1.0));
    rpy._pitch = asin(sin_pitch);
    rpy._yaw   = atan2(2*(w*z + x*y), 1-2*(y*y + z*z));
    return rpy;
}

class ImuOdometry
{
public:
    // 所有参数完全不变，默认值一个没改
    ImuOdometry(size_t max_queue = 20,
                const XYZ& gravity = XYZ{0.0, 0.0, -1.0}, // 单位：g
                double ang_deadzone = 0.01, // 单位：rad/s
                double acc_deadzone = 0.05, // 单位：g
                double vel_deadzone = 0.02) // 单位：m/s
        : max_queue_size_(max_queue),
          gravity_world_(gravity),
          angular_vel_deadzone_(ang_deadzone),
          linear_acc_deadzone_(acc_deadzone),
          velocity_deadzone_(vel_deadzone),
          vel_damping_(0.999),
          acc_scale_factor_(1.0),
          enable_centripetal_compensation_(false),
          enable_coriolis_compensation_(true),
          imu_offset_(XYZ{0.011, 0.02329, -0.04412}),
          base_quat_(Quaternion())
    {}

    // 原有接口全部保留，签名完全不变
    void addImuData(const sensor_msgs::Imu& imu_msg) {
        imu_queue_.push(imu_msg);
        while (imu_queue_.size() > max_queue_size_) imu_queue_.pop();
    }

    // ✅ 零延迟滑动窗口积分：每次调用都积分最新的完整窗口
    XYZRPY processImu(const XYZ& initial_vel, const RPY& base_rpy = RPY()) {
        if (imu_queue_.empty()) {
            XYZRPY res;
            res._xyz._x = 0; res._xyz._y = 0; res._xyz._z = 0;
            res._rpy._roll = 0; res._rpy._pitch = 0; res._rpy._yaw = 0;
            return res;
        }

        // ✅ 核心修复：仅在静止时更新base_quat_，运动时保持不变
        updateBaseQuatIfStatic();

        return integrateFullWindow(initial_vel);
    }

    void resetQueue() {
        std::queue<sensor_msgs::Imu> empty_q;
        std::swap(imu_queue_, empty_q);
    }

    void setMaxQueueSize(size_t size) { max_queue_size_ = size; }
    void setWorldGravity(const XYZ& gravity) { gravity_world_ = gravity; }
    void setAngularVelDeadzone(double threshold) { angular_vel_deadzone_ = std::fabs(threshold); }
    void setLinearAccDeadzone(double threshold) { linear_acc_deadzone_ = std::fabs(threshold); }
    void setVelocityDeadzone(double threshold) { velocity_deadzone_ = std::fabs(threshold); }
    void setVelocityDamping(double val) {
        vel_damping_ = std::max(0.9, std::min(val, 1.0));
    }
    void setImuOffset(const XYZ& offset) { imu_offset_ = offset; }
    void setAccScaleFactor(double factor) { acc_scale_factor_ = factor; }
    void enableCentripetalCompensation(bool enable) { enable_centripetal_compensation_ = enable; }
    void enableCoriolisCompensation(bool enable) { enable_coriolis_compensation_ = enable; }

private:
    XYZ crossProduct(const XYZ& v1, const XYZ& v2) const {
        XYZ res;
        res._x = v1._y * v2._z - v1._z * v2._y;
        res._y = v1._z * v2._x - v1._x * v2._z;
        res._z = v1._x * v2._y - v1._y * v2._x;
        return res;
    }

    // 平滑死区函数
    void applyDeadzone(XYZ& vec, double threshold) const {
        auto smooth_soft = [th = threshold](double v) -> double {
            double absv = fabs(v);
            if(absv < th * 0.5) return 0.0;
            if(absv > th * 1.5) return v * (absv - th) / absv;
            double t = (absv - th * 0.5) / th;
            double factor = t * t * (3 - 2 * t);
            return v * factor * (absv - th * 0.5) / absv;
        };
        vec._x = smooth_soft(vec._x);
        vec._y = smooth_soft(vec._y);
        vec._z = smooth_soft(vec._z);
    }

    Quaternion alignGravity(const XYZ& imu_acc) const {
        XYZ a = imu_acc;
        double acc_norm = sqrt(a._x*a._x + a._y*a._y + a._z*a._z);
        
        if (acc_norm < 1e-6) return Quaternion();

        a._x /= acc_norm;
        a._y /= acc_norm;
        a._z /= acc_norm;

        XYZ world_up;
        world_up._x = 0.0;
        world_up._y = 0.0;
        world_up._z = 1.0;

        XYZ axis = crossProduct(a, world_up);
        double axis_norm = sqrt(axis._x*axis._x + axis._y*axis._y + axis._z*axis._z);

        if (axis_norm < 1e-6) return Quaternion();

        double dot_product = a._x*world_up._x + a._y*world_up._y + a._z*world_up._z;
        dot_product = std::max(-1.0, std::min(dot_product, 1.0));
        double angle = acos(dot_product);

        double half_angle = angle * 0.5;
        double sin_half = sin(half_angle);
        axis._x /= axis_norm;
        axis._y /= axis_norm;
        axis._z /= axis_norm;

        return Quaternion(
            cos(half_angle),
            axis._x * sin_half,
            axis._y * sin_half,
            axis._z * sin_half
        );
    }

    // ✅ 核心修复：静止检测+自适应base_quat_更新
    void updateBaseQuatIfStatic() {
        // 计算当前窗口前5帧的加速度方差
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;
        std::vector<XYZ> accs;
        int count = 0;
        
        while (!temp_q.empty() && count < 5) {
            const auto& imu = temp_q.front();
            accs.push_back(XYZ{imu.linear_acceleration.x, 
                              imu.linear_acceleration.y, 
                              imu.linear_acceleration.z});
            count++;
            temp_q.pop();
        }
        
        if (count < 5) return;

        // 计算平均加速度
        XYZ avg_acc;
        for (const auto& acc : accs) {
            avg_acc._x += acc._x;
            avg_acc._y += acc._y;
            avg_acc._z += acc._z;
        }
        avg_acc._x /= count;
        avg_acc._y /= count;
        avg_acc._z /= count;

        // 计算加速度方差
        double var = 0.0;
        for (const auto& acc : accs) {
            var += (acc._x - avg_acc._x)*(acc._x - avg_acc._x);
            var += (acc._y - avg_acc._y)*(acc._y - avg_acc._y);
            var += (acc._z - avg_acc._z)*(acc._z - avg_acc._z);
        }
        var /= (3 * count);

        // 只有在静止时才更新base_quat_
        if (var < STATIC_ACC_VAR_THRESHOLD) {
            base_quat_ = alignGravity(avg_acc);
            ROS_DEBUG("静止状态，更新重力对齐");
        }
    }

    // ✅ 零延迟全窗口积分：每次都积分最新的完整20帧
    XYZRPY integrateFullWindow(const XYZ& initial_vel)
    {
        // 积分状态（世界坐标系下）
        XYZ pos = {0, 0, 0};
        XYZ vel = initial_vel;
        Quaternion quat = base_quat_;
        
        // 存储上一帧数据用于RK4积分
        XYZ prev_acc_world = {0, 0, 0};
        XYZ prev_omega_body = {0, 0, 0};
        double last_ts = 0.0;
        double total_dt = 0.0;
        bool first_frame = true;
        
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;
        double max_acc_dot = 0.0;
        double max_vel_dot = 0.0;
        XYZ prev_vel = initial_vel;

        while (!temp_q.empty())
        {
            const auto& imu = temp_q.front();
            temp_q.pop();
            const double curr_ts = imu.header.stamp.toSec();
            
            if (last_ts == 0.0) { 
                last_ts = curr_ts; 
                prev_omega_body._x = imu.angular_velocity.x;
                prev_omega_body._y = imu.angular_velocity.y;
                prev_omega_body._z = imu.angular_velocity.z;
                applyDeadzone(prev_omega_body, angular_vel_deadzone_);
                continue; 
            }
            
            double dt = curr_ts - last_ts;
            dt = std::max(1e-6, std::min(dt, 0.01));
            total_dt += dt;

            // 1. 姿态更新（指数映射）
            XYZ omega_body = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};
            applyDeadzone(omega_body, angular_vel_deadzone_);
            
            XYZ avg_omega;
            avg_omega._x = (prev_omega_body._x + omega_body._x) * 0.5;
            avg_omega._y = (prev_omega_body._y + omega_body._y) * 0.5;
            avg_omega._z = (prev_omega_body._z + omega_body._z) * 0.5;
            
            Quaternion dq = quatExponentialMap(avg_omega, dt);
            quat = normalize(quat * dq);

            // 2. 加速度转换到世界坐标系
            XYZ acc_body = {imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z};
            applyDeadzone(acc_body, linear_acc_deadzone_);
            acc_body._x *= G_TO_MPS2 * acc_scale_factor_;
            acc_body._y *= G_TO_MPS2 * acc_scale_factor_;
            acc_body._z *= G_TO_MPS2 * acc_scale_factor_;

            // 3. 运动学模型补偿（向心+科里奥利）
            if (enable_centripetal_compensation_ || enable_coriolis_compensation_) {
                XYZ omega_cross_r = crossProduct(avg_omega, imu_offset_);
                
                if (enable_centripetal_compensation_) {
                    XYZ centripetal_acc = crossProduct(avg_omega, omega_cross_r);
                    acc_body._x -= centripetal_acc._x;
                    acc_body._y -= centripetal_acc._y;
                    acc_body._z -= centripetal_acc._z;
                }
                
                if (enable_coriolis_compensation_) {
                    XYZ coriolis_acc = crossProduct(2.0 * avg_omega, rotateVectorByQuat(vel, quat.inverse()));
                    acc_body._x -= coriolis_acc._x;
                    acc_body._y -= coriolis_acc._y;
                    acc_body._z -= coriolis_acc._z;
                }
            }

            // 4. 转换到世界坐标系并补偿重力
            XYZ acc_world = rotateVectorByQuat(acc_body, quat);
            acc_world._x += gravity_world_._x * G_TO_MPS2;
            acc_world._y += gravity_world_._y * G_TO_MPS2;
            acc_world._z += gravity_world_._z * G_TO_MPS2;

            // 5. RK4积分速度
            XYZ k1 = prev_acc_world;
            XYZ k2;
            k2._x = (prev_acc_world._x + acc_world._x) * 0.5;
            k2._y = (prev_acc_world._y + acc_world._y) * 0.5;
            k2._z = (prev_acc_world._z + acc_world._z) * 0.5;
            XYZ k3 = k2;
            XYZ k4 = acc_world;
            
            XYZ delta_v;
            delta_v._x = (k1._x + 2*k2._x + 2*k3._x + k4._x) * dt / 6.0;
            delta_v._y = (k1._y + 2*k2._y + 2*k3._y + k4._y) * dt / 6.0;
            delta_v._z = (k1._z + 2*k2._z + 2*k3._z + k4._z) * dt / 6.0;
            
            XYZ new_vel = vel + delta_v;

            // 6. RK4积分位置
            XYZ k1_p = vel;
            XYZ k2_p;
            k2_p._x = vel._x + delta_v._x * 0.5;
            k2_p._y = vel._y + delta_v._y * 0.5;
            k2_p._z = vel._z + delta_v._z * 0.5;
            XYZ k3_p = k2_p;
            XYZ k4_p;
            k4_p._x = vel._x + delta_v._x;
            k4_p._y = vel._y + delta_v._y;
            k4_p._z = vel._z + delta_v._z;
            
            XYZ delta_p;
            delta_p._x = (k1_p._x + 2*k2_p._x + 2*k3_p._x + k4_p._x) * dt / 6.0;
            delta_p._y = (k1_p._y + 2*k2_p._y + 2*k3_p._y + k4_p._y) * dt / 6.0;
            delta_p._z = (k1_p._z + 2*k2_p._z + 2*k3_p._z + k4_p._z) * dt / 6.0;
            
            XYZ new_pos = pos + delta_p;

            // 计算动态参数用于自适应平滑
            if (!first_frame) {
                double acc_dot = fabs(acc_world._x - prev_acc_world._x) / dt;
                acc_dot = std::max(acc_dot, fabs(acc_world._y - prev_acc_world._y) / dt);
                acc_dot = std::max(acc_dot, fabs(acc_world._z - prev_acc_world._z) / dt);
                max_acc_dot = std::max(max_acc_dot, acc_dot);

                double vel_dot = fabs(new_vel._x - prev_vel._x) / dt;
                vel_dot = std::max(vel_dot, fabs(new_vel._y - prev_vel._y) / dt);
                vel_dot = std::max(vel_dot, fabs(new_vel._z - prev_vel._z) / dt);
                max_vel_dot = std::max(max_vel_dot, vel_dot);
            }

            // 更新状态
            pos = new_pos;
            vel = new_vel;
            prev_acc_world = acc_world;
            prev_omega_body = omega_body;
            prev_vel = vel;
            last_ts = curr_ts;
            first_frame = false;
        }

        // 7. 自适应后处理（仅对最终结果平滑，不影响响应）
        if (!first_frame) {
            double smooth_alpha = MAX_SMOOTH_ALPHA * std::max(0.0, 1.0 - max_acc_dot / DYNAMIC_THRESHOLD);
            double dynamic_damping = vel_damping_;
            
            double damping_reduction = 0.009 * std::min(1.0, max_vel_dot / (DYNAMIC_THRESHOLD * 0.5));
            dynamic_damping = std::max(MIN_DAMPING, vel_damping_ - damping_reduction);
            
            vel = vel * dynamic_damping;
        }

        // 应用速度死区
        applyDeadzone(vel, velocity_deadzone_);

        // 计算相对姿态
        Quaternion rel_quat = base_quat_.inverse() * quat;

        XYZRPY relative_pose;
        relative_pose._xyz = pos;
        relative_pose._rpy = quatToRpy(rel_quat);
        return relative_pose;
    }

private:
    std::queue<sensor_msgs::Imu> imu_queue_;
    size_t max_queue_size_;
    XYZ gravity_world_; // 单位：g
    double angular_vel_deadzone_; // 单位：rad/s
    double linear_acc_deadzone_; // 单位：g
    double velocity_deadzone_; // 单位：m/s
    double vel_damping_;
    double acc_scale_factor_; // 加速度计刻度因子
    bool enable_centripetal_compensation_; // 向心加速度补偿开关
    bool enable_coriolis_compensation_; // 科里奥利加速度补偿开关
    XYZ imu_offset_; // IMU相对于旋转中心的安装偏移(m)
    
    // ✅ 固定base_quat_，仅在静止时更新
    Quaternion base_quat_;
};

}

#endif
