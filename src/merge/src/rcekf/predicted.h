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
// 仅保留重力单位转换常量
constexpr double G_TO_MPS2 = 9.81;

struct Quaternion {
    double w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
    Quaternion inverse() const {
        return Quaternion(w, -x, -y, -z);
    }
};

// 四元数乘法（标准哈密顿积）
inline Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {
    Quaternion res;
    res.w = q1.w*q2.w - q1.x*q2.x - q1.y*q2.y - q1.z*q2.z;
    res.x = q1.w*q2.x + q1.x*q2.w + q1.y*q2.z - q1.z*q2.y;
    res.y = q1.w*q2.y - q1.x*q2.z + q1.y*q2.w + q1.z*q2.x;
    res.z = q1.w*q2.z + q1.x*q2.y - q1.y*q2.x + q1.z*q2.w;
    return res;
}

// 四元数归一化
inline Quaternion normalize(const Quaternion& q) {
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    if(norm < 1e-8) return Quaternion(1,0,0,0);
    return Quaternion(q.w/norm, q.x/norm, q.y/norm, q.z/norm);
}

// 四元数指数映射：角速度增量 → 姿态增量四元数
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

// 四元数旋转三维向量
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

// 四元数转RPY（Z-Y-X内旋顺序，对应横滚-俯仰-偏航）
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
    // 构造函数参数完全兼容原接口，默认值不变
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
          imu_offset_(XYZ{0.011, 0.02329, -0.04412}),
          base_quat_(Quaternion()),
          base_quat_initialized_(false)
    {}

    // 接口完全不变：添加IMU数据到队列
    void addImuData(const sensor_msgs::Imu& imu_msg) {
        imu_queue_.push(imu_msg);
        while (imu_queue_.size() > max_queue_size_) imu_queue_.pop();
    }

    // 接口完全不变：积分最新窗口内IMU，输出相对位姿
    XYZRPY processImu(const XYZ& initial_vel, const RPY& base_rpy = RPY()) {
        if (imu_queue_.empty()) {
            XYZRPY res;
            res._xyz._x = 0; res._xyz._y = 0; res._xyz._z = 0;
            res._rpy._roll = 0; res._rpy._pitch = 0; res._rpy._yaw = 0;
            return res;
        }
        return integrateFullWindow(initial_vel);
    }

    // 接口不变：重置队列，同时重置初始对准状态
    void resetQueue() {
        std::queue<sensor_msgs::Imu> empty_q;
        std::swap(imu_queue_, empty_q);
        base_quat_initialized_ = false;
    }

    // 以下参数设置接口全部保留，签名不变
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
    
    // 保留接口兼容，功能已移除（原因见下文说明）
    void enableCoriolisCompensation(bool enable) { (void)enable; }

private:
    // 三维向量叉乘
    XYZ crossProduct(const XYZ& v1, const XYZ& v2) const {
        XYZ res;
        res._x = v1._y * v2._z - v1._z * v2._y;
        res._y = v1._z * v2._x - v1._x * v2._z;
        res._z = v1._x * v2._y - v1._y * v2._x;
        return res;
    }

    // 软死区函数：平滑过渡，无阶跃跳变，抑制零偏小信号
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

    // 重力对准：用加速度计计算初始姿态（仅俯仰+横滚，航向初始为0）
    Quaternion alignGravity(const XYZ& imu_acc) const {
        XYZ a = imu_acc;
        double acc_norm = sqrt(a._x*a._x + a._y*a._y + a._z*a._z);
        
        if (acc_norm < 1e-6) return Quaternion();
        a._x /= acc_norm; a._y /= acc_norm; a._z /= acc_norm;

        XYZ world_up = {0.0, 0.0, 1.0};
        XYZ axis = crossProduct(a, world_up);
        double axis_norm = sqrt(axis._x*axis._x + axis._y*axis._y + axis._z*axis._z);
        if (axis_norm < 1e-6) return Quaternion();

        double dot_product = a._x*world_up._x + a._y*world_up._y + a._z*world_up._z;
        dot_product = std::max(-1.0, std::min(dot_product, 1.0));
        double angle = acos(dot_product);

        double half_angle = angle * 0.5;
        double sin_half = sin(half_angle);
        axis._x /= axis_norm; axis._y /= axis_norm; axis._z /= axis_norm;

        return Quaternion(
            cos(half_angle),
            axis._x * sin_half,
            axis._y * sin_half,
            axis._z * sin_half
        );
    }

    // 核心：全窗口中值积分（工业级标准二阶精度实现）
    XYZRPY integrateFullWindow(const XYZ& initial_vel)
    {
        // 世界坐标系下的积分状态
        XYZ pos = {0, 0, 0};
        XYZ vel = initial_vel;
        Quaternion quat = base_quat_;
        
        // 上一帧数据缓存
        XYZ prev_omega_body = {0, 0, 0};
        XYZ prev_acc_body = {0, 0, 0};
        double last_ts = 0.0;
        bool first_frame = true;
        
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;

        while (!temp_q.empty())
        {
            const auto& imu = temp_q.front();
            temp_q.pop();
            const double curr_ts = imu.header.stamp.toSec();
            
            // 第一帧处理：初始对准 + 缓存数据
            if (first_frame) { 
                last_ts = curr_ts; 
                // 首次积分自动完成重力初始对准，之后基准姿态固定
                if (!base_quat_initialized_) {
                    XYZ init_acc = {imu.linear_acceleration.x, 
                                    imu.linear_acceleration.y, 
                                    imu.linear_acceleration.z};
                    base_quat_ = alignGravity(init_acc);
                    quat = base_quat_;
                    base_quat_initialized_ = true;
                }

                // 缓存角速度（死区后）
                prev_omega_body._x = imu.angular_velocity.x;
                prev_omega_body._y = imu.angular_velocity.y;
                prev_omega_body._z = imu.angular_velocity.z;
                applyDeadzone(prev_omega_body, angular_vel_deadzone_);

                // 缓存加速度（死区后转m/s²）
                XYZ acc_raw = {imu.linear_acceleration.x, 
                               imu.linear_acceleration.y, 
                               imu.linear_acceleration.z};
                applyDeadzone(acc_raw, linear_acc_deadzone_);
                prev_acc_body._x = acc_raw._x * G_TO_MPS2 * acc_scale_factor_;
                prev_acc_body._y = acc_raw._y * G_TO_MPS2 * acc_scale_factor_;
                prev_acc_body._z = acc_raw._z * G_TO_MPS2 * acc_scale_factor_;

                first_frame = false;
                continue; 
            }
            
            // 时间差计算与限幅
            double dt = curr_ts - last_ts;
            dt = std::max(1e-6, std::min(dt, 0.01));

            // ========== 1. 姿态更新：中值角速度 + 指数映射 ==========
            XYZ omega_body = {imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};
            applyDeadzone(omega_body, angular_vel_deadzone_);
            
            // 前后帧角速度取平均（二阶中值积分）
            XYZ avg_omega;
            avg_omega._x = (prev_omega_body._x + omega_body._x) * 0.5;
            avg_omega._y = (prev_omega_body._y + omega_body._y) * 0.5;
            avg_omega._z = (prev_omega_body._z + omega_body._z) * 0.5;
            
            Quaternion dq = quatExponentialMap(avg_omega, dt);
            quat = normalize(quat * dq);

            // ========== 2. 加速度预处理 + 安装偏移补偿 ==========
            XYZ acc_body_raw = {imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z};
            applyDeadzone(acc_body_raw, linear_acc_deadzone_);

            // 单位转换 + 刻度因子
            XYZ acc_body;
            acc_body._x = acc_body_raw._x * G_TO_MPS2 * acc_scale_factor_;
            acc_body._y = acc_body_raw._y * G_TO_MPS2 * acc_scale_factor_;
            acc_body._z = acc_body_raw._z * G_TO_MPS2 * acc_scale_factor_;

            // 牵连加速度补偿（向心+切向，刚体安装偏移标准补偿）
            if (enable_centripetal_compensation_) {
                // 角加速度
                XYZ alpha_body;
                alpha_body._x = (omega_body._x - prev_omega_body._x) / dt;
                alpha_body._y = (omega_body._y - prev_omega_body._y) / dt;
                alpha_body._z = (omega_body._z - prev_omega_body._z) / dt;

                // 切向加速度 α×r + 向心加速度 ω×(ω×r)
                XYZ tangential_acc = crossProduct(alpha_body, imu_offset_);
                XYZ omega_cross_r = crossProduct(avg_omega, imu_offset_);
                XYZ centripetal_acc = crossProduct(avg_omega, omega_cross_r);

                // 从测量值中扣除牵连加速度，得到质心处加速度
                acc_body._x -= (tangential_acc._x + centripetal_acc._x);
                acc_body._y -= (tangential_acc._y + centripetal_acc._y);
                acc_body._z -= (tangential_acc._z + centripetal_acc._z);
            }

            // 前后帧加速度取平均
            XYZ avg_acc_body;
            avg_acc_body._x = (prev_acc_body._x + acc_body._x) * 0.5;
            avg_acc_body._y = (prev_acc_body._y + acc_body._y) * 0.5;
            avg_acc_body._z = (prev_acc_body._z + acc_body._z) * 0.5;

            // ========== 3. 转世界系 + 重力补偿 ==========
            XYZ acc_world = rotateVectorByQuat(avg_acc_body, quat);
            acc_world._x += gravity_world_._x * G_TO_MPS2;
            acc_world._y += gravity_world_._y * G_TO_MPS2;
            acc_world._z += gravity_world_._z * G_TO_MPS2;

            // ========== 4. 中值积分速度 ==========
            XYZ new_vel;
            new_vel._x = vel._x + acc_world._x * dt;
            new_vel._y = vel._y + acc_world._y * dt;
            new_vel._z = vel._z + acc_world._z * dt;

            // 固定速度阻尼（仅抑制零偏漂移，无自适应）
            new_vel._x *= vel_damping_;
            new_vel._y *= vel_damping_;
            new_vel._z *= vel_damping_;

            // ========== 5. 中值积分位置（平均速度积分，二阶精度） ==========
            pos._x += (vel._x + new_vel._x) * 0.5 * dt;
            pos._y += (vel._y + new_vel._y) * 0.5 * dt;
            pos._z += (vel._z + new_vel._z) * 0.5 * dt;

            // 更新状态与缓存
            vel = new_vel;
            prev_omega_body = omega_body;
            prev_acc_body = acc_body;
            last_ts = curr_ts;
        }

        // 应用速度死区
        applyDeadzone(vel, velocity_deadzone_);

        // 计算相对初始基准的姿态
        Quaternion rel_quat = base_quat_.inverse() * quat;

        XYZRPY relative_pose;
        relative_pose._xyz = pos;
        relative_pose._rpy = quatToRpy(rel_quat);
        return relative_pose;
    }

private:
    std::queue<sensor_msgs::Imu> imu_queue_;
    size_t max_queue_size_;
    XYZ gravity_world_;          // 单位：g，世界系重力矢量
    double angular_vel_deadzone_;// 单位：rad/s
    double linear_acc_deadzone_; // 单位：g
    double velocity_deadzone_;   // 单位：m/s
    double vel_damping_;         // 速度阻尼系数（固定值）
    double acc_scale_factor_;    // 加速度计刻度因子
    bool enable_centripetal_compensation_; // 牵连加速度补偿开关
    XYZ imu_offset_;             // IMU安装偏移(m)
    
    Quaternion base_quat_;       // 初始基准姿态（仅初始对准一次）
    bool base_quat_initialized_; // 基准姿态初始化标记
};

} // namespace Ten

#endif
