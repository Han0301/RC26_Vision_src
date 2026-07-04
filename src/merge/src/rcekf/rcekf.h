#ifndef __RCEKF_H_
#define __RCEKF_H_

#include "./../method_math.h"
#include <Eigen/Dense>
#include <cmath>
#include "./../parameter/parameter.h"


// ==============================================================================================
//                           12维 PV-EKF 位姿速度卡尔曼滤波【全矩阵展开注释版】
// 状态索引定义(0~11):
// [0:x, 1:y, 2:z | 3:roll, 4:pitch, 5:yaw | 6:vx,7:vy,8:vz |9:ω_r,10:ω_p,11:ω_y]
// ==============================================================================================
// 一、状态向量 X ∈ R(12×1)
// X = [
//      x,
//      y,
//      z,
//      roll,
//      pitch,
//      yaw,
//      vx,
//      vy,
//      vz,
//      ω_roll,
//      ω_pitch,
//      ω_yaw
//     ]^T
//
// 二、状态转移矩阵 A ∈ R(12×12) 【CV匀速模型，仅对角线=1、6个位置项=dt，其余全0】
// A =
// [ 1, 0, 0, 0, 0, 0, dt,  0,   0,   0,    0,    0 ]
// [ 0, 1, 0, 0, 0, 0,  0, dt,   0,   0,    0,    0 ]
// [ 0, 0, 1, 0, 0, 0,  0,  0, dt,   0,    0,    0 ]
// [ 0, 0, 0, 1, 0, 0,  0,  0,   0, dt,    0,    0 ]
// [ 0, 0, 0, 0, 1, 0,  0,  0,   0,   0,   dt,    0 ]
// [ 0, 0, 0, 0, 0, 1,  0,  0,   0,   0,    0,   dt ]
// [ 0, 0, 0, 0, 0, 0,  1,  0,   0,   0,    0,    0 ]
// [ 0, 0, 0, 0, 0, 0,  0,  1,   0,   0,    0,    0 ]
// [ 0, 0, 0, 0, 0, 0,  0,  0,   1,   0,    0,    0 ]
// [ 0, 0, 0, 0, 0, 0,  0,  0,   0,   1,    0,    0 ]
// [ 0, 0, 0, 0, 0, 0,  0,  0,   0,   0,    1,    0 ]
// [ 0, 0, 0, 0, 0, 0,  0,  0,   0,   0,    0,    1 ]
//
// 三、过程噪声协方差 Q ∈ R(12×12) 【6组独立2×2子块，所有未标注位置元素全为0】
// 单轴子块通用公式：Q_sub = [ q_base·dt³/3 , q_base·dt²/2 ]
//                           [ q_base·dt²/2 , q_var·dt     ]
// X/Y/Z轴: q_base=q_pos，速度项=q_vel；R/P/Y轴: q_base=q_att，角速度项=q_ang
// 行\列|     0           1           2           3            4            5           6           7           8           9           10          11
// ----|-----------------------------------------------------------------------------------------------------------------------------
// 0   | q_pos·dt³/3,     0,          0,          0,           0,           0,    q_pos·dt²/2,     0,          0,          0,           0,           0
// 1   |      0,     q_pos·dt³/3,     0,          0,           0,           0,         0,    q_pos·dt²/2,     0,          0,           0,           0
// 2   |      0,          0,     q_pos·dt³/3,     0,           0,           0,         0,          0,    q_pos·dt²/2,     0,           0,           0
// 3   |      0,          0,          0,     q_att·dt³/3,      0,           0,         0,          0,          0,    q_att·dt²/2,      0,           0
// 4   |      0,          0,          0,          0,      q_att·dt³/3,      0,         0,          0,          0,          0,    q_att·dt²/2,      0
// 5   |      0,          0,          0,          0,           0,      q_att·dt³/3,    0,          0,          0,          0,           0,    q_att·dt²/2
// 6   | q_pos·dt²/2,     0,          0,          0,           0,           0,     q_vel·dt,      0,          0,          0,           0,           0
// 7   |      0,     q_pos·dt²/2,     0,          0,           0,           0,         0,     q_vel·dt,      0,          0,           0,           0
// 8   |      0,          0,     q_pos·dt²/2,     0,           0,           0,         0,          0,     q_vel·dt,      0,           0,           0
// 9   |      0,          0,          0,     q_att·dt²/2,      0,           0,         0,          0,          0,     q_ang·dt,      0,           0
//10   |      0,          0,          0,          0,      q_att·dt²/2,      0,         0,          0,          0,          0,     q_ang·dt,      0
//11   |      0,          0,          0,          0,           0,      q_att·dt²/2,    0,          0,          0,          0,           0,     q_ang·dt
//
// 四、观测矩阵 H ∈ R(12×12) = I₁₂(12阶单位阵)，对角线全1，其余元素=0
//
// 五、观测噪声 R ∈ R(12×12)：对角阵，仅对角线赋值实测噪声，非对角全部=0
//
// 六、滤波误差协方差 P ∈ R(12×12)：初始化对角=100，迭代自动更新
//
// ====================== 卡尔曼标准迭代公式 ======================
// 【预测步】
// Xₖ₋ = A * Xₖ₋₁
// Pₖ₋ = A * Pₖ₋₁ * Aᵀ + Q
// 【更新步】
// S = H * Pₖ₋ * Hᵀ + R
// K = Pₖ₋ * Hᵀ * S⁻¹
// Xₖ = Xₖ₋ + K*(Z - H*Xₖ₋)
// Pₖ = (I - K*H) * Pₖ₋
// ==============================================================================================




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
