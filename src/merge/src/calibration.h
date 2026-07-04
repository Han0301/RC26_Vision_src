#ifndef __CALIBRATION_H_
#define __CALIBRATION_H_
#include <ros/ros.h>
#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <cmath>

namespace Ten
{

    class Ten_calibration
    {
    public:
        Ten_calibration(const Ten_calibration& calib) = delete;
        Ten_calibration& operator=(const Ten_calibration& calib) = delete;
        Ten_calibration(){}
        
        /**
         * @brief 设置雷达和相机自己的变换矩阵
         * @param lidar ：4×4的变换矩阵
         * @param camera ：4×4的变换矩阵
         */
        void set_lidar_and_camera_T(Eigen::Matrix4d lidar, Eigen::Matrix4d camera)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            lidars_T_.push_back(lidar);
            cameras_T_.push_back(camera);
        }

        /**
         * @brief 设置粗变换矩阵
         * @param c ：4×4的初始变换矩阵
         */
        void set_c(Eigen::Matrix4d c)
        {
            std::lock_guard<std::mutex> lock(mtx_);
            lidar_to_camera_ = c;
        }


        /**
         * @brief 获取雷达到相机的变换
         * @return Eigen::Matrix4d: 变换矩阵
         */
        Eigen::Matrix4d get_calibration()
        {
            double error = solveSimilarityMatrixC(lidars_T_, cameras_T_, lidar_to_camera_);
            if(error > 0)
            {
                return lidar_to_camera_;
            }
            else 
            {
                return Eigen::Matrix4d::Identity();
            }
        }


    private:
    std::vector<Eigen::Matrix4d> lidars_T_;
    std::vector<Eigen::Matrix4d> cameras_T_;
    Eigen::Matrix4d lidar_to_camera_ = Eigen::Matrix4d::Identity();
    mutable std::mutex mtx_;

    // ======================================
    // 核心工具函数：把任意 3x3 矩阵投影为最近的旋转矩阵
    // ======================================
    Eigen::Matrix3d projectToRotationMatrix(const Eigen::Matrix3d& M) {
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Matrix3d R = svd.matrixU() * svd.matrixV().transpose();
        
        // 保证行列式为 1（防止反射）
        if (R.determinant() < 0) {
            Eigen::Matrix3d V = svd.matrixV();
            V.col(2) *= -1;
            R = svd.matrixU() * V.transpose();
        }
        return R;
    }

    // ======================================
    // 求解函数（移除 Ceres，改用极分解）
    // ======================================
    double solveSimilarityMatrixC(const std::vector<Eigen::Matrix4d>& A_vec,
        const std::vector<Eigen::Matrix4d>& B_vec,
        Eigen::Matrix4d& C_opt)
    {
    // 1. 入参校验
    if (A_vec.empty() || B_vec.empty() || A_vec.size() != B_vec.size())
    {
        ROS_ERROR("solveSimilarityMatrixC: 输入参数非法！");
        return -1.0;
    }
    const int mat_size = 4;
    const int vec_size = mat_size * mat_size;
    const size_t pair_num = A_vec.size();

    // 2. 线性求解初始值（完全保留你原来的逻辑）
    Eigen::MatrixXd M(0, vec_size);
    for (size_t idx = 0; idx < pair_num; ++idx)
    {
    const Eigen::Matrix4d& A = A_vec[idx];
    const Eigen::Matrix4d& B = B_vec[idx];
    Eigen::MatrixXd mat_constraint(vec_size, vec_size);
    mat_constraint.setZero();
    for (int i = 0; i < mat_size; ++i)
    {
    for (int j = 0; j < mat_size; ++j)
    {
    int row_idx = i * mat_size + j;
    for (int k = 0; k < mat_size; ++k)
    {
    mat_constraint(row_idx, i * mat_size + k) += A(k, j);
    mat_constraint(row_idx, k * mat_size + j) -= B(i, k);
    }
    }
    }
    M.conservativeResize(M.rows() + vec_size, vec_size);
    M.bottomRows(vec_size) = mat_constraint;
    }

    Eigen::MatrixXd MtM = M.transpose() * M;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eig_solver(MtM);
    if (eig_solver.info() != Eigen::Success)
    {
        ROS_ERROR("solveSimilarityMatrixC: 特征值分解失败！");
        return -1.0;
    }
    Eigen::VectorXd eigen_values = eig_solver.eigenvalues();
    Eigen::MatrixXd eigen_vectors = eig_solver.eigenvectors();
    int min_eig_idx = 0;
    for (int i = 1; i < eigen_values.size(); ++i)
    {
    if (eigen_values(i) < eigen_values(min_eig_idx))
    {
    min_eig_idx = i;
    }
    }
    Eigen::VectorXd c_vec = eigen_vectors.col(min_eig_idx);
    double norm = c_vec.norm();
    if (std::fabs(norm) < 1e-8)
    {
        ROS_ERROR("solveSimilarityMatrixC: 零解！");
        return -1.0;
    }
    c_vec /= norm;
    Eigen::Map<Eigen::Matrix4d> C_temp(c_vec.data(), mat_size, mat_size);
    C_opt = C_temp;

    // ======================================
    // 【核心替换】极分解投影：保证旋转矩阵合法
    // ======================================
    {
        // 1. 提取线性解的旋转部分
        Eigen::Matrix3d R_linear = C_opt.block<3, 3>(0, 0);
        // 2. 提取线性解的平移部分
        Eigen::Vector3d t_linear = C_opt.block<3, 1>(0, 3);
        
        // 3. 【关键】把 R_linear 投影为最近的正交旋转矩阵
        Eigen::Matrix3d R_proj = projectToRotationMatrix(R_linear);
        
        // 4. 重组为合法的 SE(3) 矩阵
        C_opt.setIdentity();
        C_opt.block<3, 3>(0, 0) = R_proj;
        C_opt.block<3, 1>(0, 3) = t_linear;
    }

    // 3. 计算最终误差
    double total_error = 0.0;
    for (size_t idx = 0; idx < pair_num; ++idx)
    {
    const Eigen::Matrix4d& A = A_vec[idx];
    const Eigen::Matrix4d& B = B_vec[idx];
    Eigen::Matrix4d error_mat = C_opt * A - B * C_opt;
    total_error += error_mat.norm();
    }
    return total_error / pair_num;
    }

};

}

#endif
