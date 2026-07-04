#ifndef __RCEKF_H_
#define __RCEKF_H_

#include "./../method_math.h"
#include <Eigen/Dense>
#include <cmath>
#include "./../parameter/parameter.h"

namespace Ten
{
    struct PV
    {
        Ten::XYZRPY pose;
        Ten::XYZRPY velocity;
    };

class PoseVelocityKalmanFilter {
    private:
        Eigen::VectorXd X_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd A_;
        Eigen::MatrixXd H_;
        Eigen::MatrixXd Q_;  // 现在会随dt动态更新+包含协方差
        Eigen::MatrixXd R_;
        Eigen::MatrixXd I_;
        bool is_initialized_;

        // 过程噪声基础系数（你原本的参数，保留）
        const double q_pos   = _q_pos_;   // 位置过程噪声系数
        const double q_att   = _q_att_;   // 姿态过程噪声系数
        const double q_vel   = _q_vel_;   // 线速度过程噪声系数
        const double q_ang   = _q_ang_;   // 角速度过程噪声系数

        void normalizeAngle(double& angle) const {
            angle = std::fmod(angle + M_PI, 2 * M_PI);
            angle = (angle < 0) ? (angle + 2 * M_PI) : angle;
            angle -= M_PI;
        }

        // ==============================================
        // 【核心修正1】更新状态转移矩阵 A
        // ==============================================
        void updateStateTransitionMatrix(double dt) {
            A_ = Eigen::MatrixXd::Identity(12, 12);
            A_(0, 6) = dt; A_(1, 7) = dt; A_(2, 8) = dt;
            A_(3, 9) = dt; A_(4, 10) = dt; A_(5, 11) = dt;
        }

        // ==============================================
        // 【核心修正2】动态计算 Q 矩阵（含协方差！）
        // ==============================================
        void updateProcessNoiseQ(double dt) {
            Q_ = Eigen::MatrixXd::Zero(12, 12);
            double dt2 = dt * dt;
            double dt3 = dt2 * dt;
            // ==================== XYZ 位置 + 线速度 ====================
            // X轴
            Q_(0,0) = q_pos * dt3 / 3;
            Q_(0,6) = q_pos * dt2 / 2;
            Q_(6,0) = q_pos * dt2 / 2;
            Q_(6,6) = q_vel * dt;
            // Y轴
            Q_(1,1) = q_pos * dt3 / 3;
            Q_(1,7) = q_pos * dt2 / 2;
            Q_(7,1) = q_pos * dt2 / 2;
            Q_(7,7) = q_vel * dt;
            // Z轴
            Q_(2,2) = q_pos * dt3 / 3;
            Q_(2,8) = q_pos * dt2 / 2;
            Q_(8,2) = q_pos * dt2 / 2;
            Q_(8,8) = q_vel * dt;

            // ==================== RPY 姿态 + 角速度 ====================
            // Roll
            Q_(3,3)  = q_att * dt3 / 3;
            Q_(3,9)  = q_att * dt2 / 2;
            Q_(9,3)  = q_att * dt2 / 2;
            Q_(9,9)  = q_ang * dt;
            // Pitch
            Q_(4,4)  = q_att * dt3 / 3;
            Q_(4,10) = q_att * dt2 / 2;
            Q_(10,4) = q_att * dt2 / 2;
            Q_(10,10)= q_ang * dt;
            // Yaw
            Q_(5,5)  = q_att * dt3 / 3;
            Q_(5,11) = q_att * dt2 / 2;
            Q_(11,5) = q_att * dt2 / 2;
            Q_(11,11)= q_ang * dt;
        }

        

        Eigen::VectorXd pvToObservation(const Ten::PV& pv) const {
            Eigen::VectorXd Z(12);
            Z << pv.pose._xyz._x,    pv.pose._xyz._y,    pv.pose._xyz._z,
                 pv.pose._rpy._roll, pv.pose._rpy._pitch, pv.pose._rpy._yaw,
                 pv.velocity._xyz._x, pv.velocity._xyz._y, pv.velocity._xyz._z,
                 pv.velocity._rpy._roll, pv.velocity._rpy._pitch, pv.velocity._rpy._yaw;
            return Z;
        }

        Ten::PV stateToPV() const {
            Ten::PV pv;
            pv.pose._xyz._x = X_(0);
            pv.pose._xyz._y = X_(1);
            pv.pose._xyz._z = X_(2);
            pv.pose._rpy._roll = X_(3);
            pv.pose._rpy._pitch = X_(4);
            pv.pose._rpy._yaw = X_(5);
            pv.velocity._xyz._x = X_(6);
            pv.velocity._xyz._y = X_(7);
            pv.velocity._xyz._z = X_(8);
            pv.velocity._rpy._roll = X_(9);
            pv.velocity._rpy._pitch = X_(10);
            pv.velocity._rpy._yaw = X_(11);
            return pv;
        }

        void initFilter(const Eigen::VectorXd& initial_Z, double dt) {
            X_ = initial_Z;

            // 初始化P矩阵
            P_ = Eigen::MatrixXd::Identity(12,12) * 100.0;
            I_ = Eigen::MatrixXd::Identity(12,12);
            H_ = I_;

            // 初始化Q（第一次计算）
            updateProcessNoiseQ(dt);
            updateStateTransitionMatrix(dt);

            // R矩阵保留你实测的值（完全正确）
            R_ = Eigen::MatrixXd::Zero(12, 12);
            R_(0,0) = 0.00000454737561734413; R_(1,1) = 0.00000670229600157794; R_(2,2) = 0.000000966788404964076;
            R_(3,3) = 0.00000037132252406422; R_(4,4) = 0.000000409242392379194; R_(5,5) = 0.000000403970994121073;
            R_(6,6) = 0.0000244158199177451; R_(7,7) = 0.0000202568419409614; R_(8,8) = 0.00000730310708314043;
            R_(9,9) = 0.00000929506936757248; R_(10,10)=0.00000637065338028182; R_(11,11)=0.00000159615555503468;

            is_initialized_ = true;
        }

    public:
        PoseVelocityKalmanFilter() : is_initialized_(false) {
            X_.resize(12); P_.resize(12,12); A_.resize(12,12);
            H_.resize(12,12); Q_.resize(12,12); R_.resize(12,12); I_.resize(12,12);
        }

        Ten::PV process(Ten::PV& pv, double dt) {
            Eigen::VectorXd Z = pvToObservation(pv);
            if (!is_initialized_) {
                initFilter(Z, dt);
                return stateToPV();
            }

            // 【核心修正3】每次滤波都更新 A 和 Q！
            updateStateTransitionMatrix(dt);
            updateProcessNoiseQ(dt);

            // 卡尔曼预测
            X_ = A_ * X_;
            P_ = A_ * P_ * A_.transpose() + Q_;

            // 卡尔曼更新
            Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
            Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();
            X_ = X_ + K * (Z - H_ * X_);
            P_ = (I_ - K * H_) * P_;

            normalizeAngle(X_(3));
            normalizeAngle(X_(4));
            normalizeAngle(X_(5));

            //解决yaw漂移问题，就是不用卡尔曼
            Ten::PV pv_ekf = stateToPV();
            pv_ekf.pose._rpy._yaw = pv.pose._rpy._yaw;
            return pv_ekf;
        }

        void reset() { is_initialized_ = false; }
    };
}

#endif
