#ifndef __PREDICTED_H_
#define __PREDICTED_H_

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <queue>
#include <cmath>
#include <vector>
#include "./../method_math.h"

// 四元数运算（必备）
namespace Ten {
struct Quaternion {
    double w, x, y, z;
    Quaternion() : w(1), x(0), y(0), z(0) {}
    Quaternion(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
};

// ✅ 修复：加 inline
inline Quaternion normalize(const Quaternion& q) {
    double norm = sqrt(q.w*q.w + q.x*q.x + q.y*q.y + q.z*q.z);
    return Quaternion(q.w/norm, q.x/norm, q.y/norm, q.z/norm);
}

// ✅ 修复：加 inline
inline Quaternion quatDerivative(const Quaternion& q, const XYZ& body_omega) {
    double w = q.w, x = q.x, y = q.y, z = q.z;
    double wx = body_omega._x, wy = body_omega._y, wz = body_omega._z;
    Quaternion dq;
    dq.w = 0.5 * (-x*wx - y*wy - z*wz);
    dq.x = 0.5 * (w*wx + y*wz - z*wy);
    dq.y = 0.5 * (w*wy - x*wz + z*wx);
    dq.z = 0.5 * (w*wz + x*wy - y*wx);
    return dq;
}

// ✅ 修复：加 inline
inline void quatToRot(const Quaternion& q, std::vector<std::vector<double>>& rot) {
    double w = q.w, x = q.x, y = q.y, z = q.z;
    rot[0][0] = 1-2*y*y-2*z*z; rot[0][1] = 2*x*y-2*z*w;   rot[0][2] = 2*x*z+2*y*w;
    rot[1][0] = 2*x*y+2*z*w;   rot[1][1] = 1-2*x*x-2*z*z; rot[1][2] = 2*y*z-2*x*w;
    rot[2][0] = 2*x*z-2*y*w;   rot[2][1] = 2*y*z+2*x*w;   rot[2][2] = 1-2*x*x-2*y*y;
}

// ✅ 修复：加 inline
inline Quaternion rpyToQuat(const RPY& rpy) {
    double cr = cos(rpy._roll*0.5), sr = sin(rpy._roll*0.5);
    double cp = cos(rpy._pitch*0.5), sp = sin(rpy._pitch*0.5);
    double cy = cos(rpy._yaw*0.5), sy = sin(rpy._yaw*0.5);
    Quaternion q;
    q.w = cr*cp*cy + sr*sp*sy;
    q.x = sr*cp*cy - cr*sp*sy;
    q.y = cr*sp*cy + sr*cp*sy;
    q.z = cr*cp*sy - sr*sp*cy;
    return q;
}

// ✅ 修复：加 inline
inline RPY quatToRpy(const Quaternion& q) {
    RPY rpy;
    double w = q.w, x = q.x, y = q.y, z = q.z;
    rpy._roll  = atan2(2*(w*x + y*z), 1-2*(x*x + y*y));
    rpy._pitch = asin(-2*(x*z - w*y));
    rpy._yaw   = atan2(2*(w*z + x*y), 1-2*(y*y + z*z));
    return rpy;
}

class ImuOdometry
{
public:
    ImuOdometry(size_t max_queue = 20, 
                const XYZ& gravity = XYZ{0.0, 0.0,  0.9810},
                double ang_deadzone = 0.02,
                double acc_deadzone = 0.02,
                double vel_deadzone = 0.02)
        : max_queue_size_(max_queue),
          gravity_world_(gravity),
          angular_vel_deadzone_(ang_deadzone),
          linear_acc_deadzone_(acc_deadzone),
          velocity_deadzone_(vel_deadzone)
    {
        current_rot_.assign(3, std::vector<double>(3, 0.0));
        base_quat_ = Quaternion();
        rel_quat_ = Quaternion();
    }

    void addImuData(const sensor_msgs::Imu& imu_msg) {
        cacheImuData(imu_msg);
    }

    XYZRPY processImu(const XYZ& initial_vel, const RPY& base_rpy) {
        base_quat_ = rpyToQuat(base_rpy);
        rel_quat_ = Quaternion();
        return integrateWindow(initial_vel);
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

private:
    void cacheImuData(const sensor_msgs::Imu& imu_msg) {
        imu_queue_.push(imu_msg);
        while (imu_queue_.size() > max_queue_size_) imu_queue_.pop();
    }

    XYZ crossProduct(const XYZ& v1, const XYZ& v2) const {
        XYZ res;
        res._x = v1._y * v2._z - v1._z * v2._y;
        res._y = v1._z * v2._x - v1._x * v2._z;
        res._z = v1._x * v2._y - v1._y * v2._x;
        return res;
    }

    void applyDeadzone(XYZ& vec, double threshold) const {
        if (std::fabs(vec._x) < threshold) vec._x = 0.0;
        if (std::fabs(vec._y) < threshold) vec._y = 0.0;
        if (std::fabs(vec._z) < threshold) vec._z = 0.0;
    }

    void transformImuToWorld(double x, double y, double z, XYZ& out) {
        out._x = current_rot_[0][0]*x + current_rot_[0][1]*y + current_rot_[0][2]*z;
        out._y = current_rot_[1][0]*x + current_rot_[1][1]*y + current_rot_[1][2]*z;
        out._z = current_rot_[2][0]*x + current_rot_[2][1]*y + current_rot_[2][2]*z;
    }

    XYZRPY integrateWindow(const XYZ& initial_vel)
    {
        XYZ rel_pos;
        XYZ filtered_vel = initial_vel;
        applyDeadzone(filtered_vel, velocity_deadzone_);
        XYZ rel_vel = filtered_vel;

        double last_ts = 0.0;
        std::queue<sensor_msgs::Imu> temp_q = imu_queue_;

        while (!temp_q.empty())
        {
            const auto imu = temp_q.front();
            temp_q.pop();
            const double curr_ts = imu.header.stamp.toSec();
            if (last_ts == 0.0) { last_ts = curr_ts; continue; }
            double dt = curr_ts - last_ts;

            XYZ body_omega{imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z};
            applyDeadzone(body_omega, angular_vel_deadzone_);

            Quaternion dq = quatDerivative(rel_quat_, body_omega);
            rel_quat_.w += dq.w * dt;
            rel_quat_.x += dq.x * dt;
            rel_quat_.y += dq.y * dt;
            rel_quat_.z += dq.z * dt;
            rel_quat_ = normalize(rel_quat_);

            Quaternion total_quat = base_quat_;
            quatToRot(total_quat, current_rot_);

            XYZ linear_acc;
            // std::cout << "imu.linear_acceleration.x: " << imu.linear_acceleration.x << std::endl;
            // std::cout << "imu.linear_acceleration.y: " << imu.linear_acceleration.y << std::endl;
            // std::cout << "imu.linear_acceleration.z: " << imu.linear_acceleration.z << std::endl;
            transformImuToWorld(imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z, linear_acc);

            XYZ omega_world;
            transformImuToWorld(body_omega._x, body_omega._y, body_omega._z, omega_world);
            XYZ omega_cross_r = crossProduct(omega_world, rel_pos);
            XYZ centripetal_acc = crossProduct(omega_world, omega_cross_r);
            centripetal_acc = -centripetal_acc;

            linear_acc._x -= (gravity_world_._x + centripetal_acc._x);
            linear_acc._y -= (gravity_world_._y + centripetal_acc._y);
            linear_acc._z -= (gravity_world_._z + centripetal_acc._z);

            applyDeadzone(linear_acc, linear_acc_deadzone_);

            rel_vel._x += linear_acc._x * dt;
            rel_vel._y += linear_acc._y * dt;
            rel_vel._z += linear_acc._z * dt;

            rel_pos._x += rel_vel._x * dt;
            rel_pos._y += rel_vel._y * dt;
            rel_pos._z += rel_vel._z * dt;

            last_ts = curr_ts;
        }

        XYZRPY relative_pose;
        relative_pose._xyz = rel_pos;
        relative_pose._rpy = quatToRpy(rel_quat_);
        return relative_pose;
    }

private:
    std::queue<sensor_msgs::Imu> imu_queue_;
    size_t max_queue_size_;
    XYZ gravity_world_;
    double angular_vel_deadzone_, linear_acc_deadzone_, velocity_deadzone_;
    std::vector<std::vector<double>> current_rot_;
    Quaternion base_quat_;
    Quaternion rel_quat_;
};

}

#endif
